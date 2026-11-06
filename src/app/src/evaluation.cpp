// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include "evaluation.hpp"

#include "run_dir.hpp"

#include <SimpleSLAM/config/calibration_io.hpp>
#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/eval/association.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <format>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

#ifndef SIMPLESLAM_VERSION
#define SIMPLESLAM_VERSION "unknown"
#endif

namespace simpleslam::app {

namespace {

using run_dir::jsonNumber;
using run_dir::jsonString;

std::optional<std::int64_t> parseNanoseconds(const YAML::Node& node) {
    if (!node || !node.IsScalar()) return std::nullopt;
    const std::string text = node.Scalar();
    std::int64_t value = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size()) return std::nullopt;
    return value;
}

/// 一个插件参数的规范化文本：协议的比较与 JSON 都用它，往返无损。
std::string valueText(const config::PluginValue& value) {
    return std::visit(
        [](const auto& v) -> std::string {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, bool>) {
                return v ? "true" : "false";
            } else if constexpr (std::is_same_v<T, std::string>) {
                return v;
            } else {
                return std::format("{}", v);
            }
        },
        value);
}

std::string valueJson(const config::PluginValue& value) {
    if (const auto* text = std::get_if<std::string>(&value)) return jsonString(*text);
    if (const auto* real = std::get_if<double>(&value)) return jsonNumber(*real);
    return valueText(value);
}

/// 评测函数的协议：类型与参数，子节点的键带上前缀。门禁拿它与基线核对。
void flattenProtocol(const config::PluginSpec& spec, const std::string& prefix,
                     eval::Protocol& out) {
    out[prefix + "type"] = spec.type;
    for (const auto& [key, value] : spec.params) out[prefix + key] = valueText(value);
    for (const auto& [key, child] : spec.children) flattenProtocol(child, prefix + key + ".", out);
}

std::string paramsJson(const config::PluginSpec& spec) {
    std::string out = "{";
    for (const auto& [key, value] : spec.params) {
        out += (out.size() == 1 ? "" : ", ") + jsonString(key) + ": " + valueJson(value);
    }
    for (const auto& [key, child] : spec.children) {
        out += (out.size() == 1 ? "" : ", ") + jsonString(key) + ": {\"type\": "
               + jsonString(child.type) + ", \"params\": " + paramsJson(child) + "}";
    }
    return out + "}";
}

std::string cellText(const eval::Cell& cell) {
    if (const auto* integer = std::get_if<std::int64_t>(&cell)) return std::to_string(*integer);
    const double real = std::get<double>(cell);
    return std::isfinite(real) ? std::format("{}", real) : std::string();  // 没有值就留空
}

std::string tableFile(const std::string& evaluator, const std::string& table) {
    return "evaluation/" + evaluator + "." + table + ".csv";
}

/// 名字只收字母、数字与 `_`；允许分段时还收 `.`，但不在两头、不连写。
bool plainName(std::string_view name, bool segments) {
    if (name.empty() || name.front() == '.' || name.back() == '.') return false;
    if (name.find("..") != std::string_view::npos) return false;
    return std::all_of(name.begin(), name.end(), [&](char c) {
        const bool plain = std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
        return plain || (segments && c == '.');
    });
}

/// 评测函数给的名字进指标名、文件名与 CSV 的表头，表格的每一行要与表头对齐：
/// 写任何文件之前核对，写错了指名是哪个评测函数。
void checkResult(const std::string& evaluator, const eval::Evaluation& result) {
    const auto fail = [&](const std::string& what) {
        throw std::runtime_error(std::format("评测函数 {} 给的结果不合格：{}", evaluator, what));
    };
    std::vector<std::string_view> scalars;
    for (const auto& scalar : result.scalars) {
        if (!plainName(scalar.name, true)) {
            fail(std::format("标量名 {} 只能用字母、数字、_ 与分段的 .", scalar.name));
        }
        if (scalar.name == "samples") fail("标量名 samples 留给样本数");
        if (std::find(scalars.begin(), scalars.end(), scalar.name) != scalars.end()) {
            fail(std::format("标量名 {} 重复", scalar.name));
        }
        scalars.push_back(scalar.name);
    }
    std::vector<std::string_view> tables;
    for (const auto& table : result.tables) {
        if (!plainName(table.name, false)) {
            fail(std::format("表名 {} 只能用字母、数字与 _：它是文件名的一段", table.name));
        }
        if (std::find(tables.begin(), tables.end(), table.name) != tables.end()) {
            fail(std::format("表名 {} 重复", table.name));
        }
        tables.push_back(table.name);
        for (const auto& column : table.columns) {
            if (!plainName(column.name, false)) {
                fail(std::format("表 {} 的列名 {} 只能用字母、数字与 _", table.name, column.name));
            }
        }
        for (const auto& row : table.rows) {
            if (row.size() != table.columns.size()) {
                fail(std::format("表 {} 有一行的格数是 {}，列数是 {}", table.name, row.size(),
                                 table.columns.size()));
            }
        }
    }
}

void writeTable(const fs::path& dir, const std::string& evaluator, const eval::Table& table) {
    std::ostringstream text;
    for (std::size_t i = 0; i < table.columns.size(); ++i) {
        text << (i == 0 ? "" : ",") << table.columns[i].name;
    }
    text << '\n';
    for (const auto& row : table.rows) {
        for (std::size_t i = 0; i < row.size(); ++i) {
            text << (i == 0 ? "" : ",") << cellText(row[i]);
        }
        text << '\n';
    }
    run_dir::writeText(dir / tableFile(evaluator, table.name), text.str());
}

/// 给人看的一行：样本数与进汇总表的那几项；没测出东西时写最后一条说明。
std::string summaryLine(const NamedEvaluation& named) {
    const auto& result = named.result;
    std::string line = std::format("评测 {}（{}）: ", named.name, named.spec.type);
    if (result.samples == 0) {
        return line + "无结果" + (result.notes.empty() ? "" : "——" + result.notes.back());
    }
    line += std::format("{} 个样本", result.samples);
    for (const auto& scalar : result.scalars) {
        if (scalar.label.empty()) continue;
        line += std::format("  {} {:.4f}", scalar.label, scalar.value);
        if (!scalar.unit.empty()) line += " " + scalar.unit;
    }
    return line;
}

std::string evaluationJson(const NamedEvaluation& named, const fs::path& dir) {
    const auto& result = named.result;
    std::ostringstream out;
    out << "    " << jsonString(named.name)
        << ": {\n      \"type\": " << jsonString(named.spec.type)
        << ",\n      \"params\": " << paramsJson(named.spec)
        << ",\n      \"software_version\": " << jsonString(SIMPLESLAM_VERSION)
        << ",\n      \"status\": " << jsonString(result.samples > 0 ? "ok" : "no_result")
        << ",\n      \"samples\": " << result.samples << ",\n      \"inputs\": {";
    for (std::size_t i = 0; i < named.inputs.size(); ++i) {
        out << (i == 0 ? "" : ", ") << jsonString(named.inputs[i]) << ": "
            << jsonString(eval::trajectoryFileDigest((dir / named.inputs[i]).string()));
    }
    out << "},\n      \"scalars\": {";
    for (std::size_t i = 0; i < result.scalars.size(); ++i) {
        const auto& scalar = result.scalars[i];
        out << (i == 0 ? "\n" : ",\n") << "        " << jsonString(scalar.name)
            << ": {\"value\": " << jsonNumber(scalar.value)
            << ", \"unit\": " << jsonString(scalar.unit)
            << ", \"label\": " << jsonString(scalar.label) << "}";
    }
    out << (result.scalars.empty() ? "" : "\n      ") << "},\n      \"tables\": {";
    for (std::size_t i = 0; i < result.tables.size(); ++i) {
        const auto& table = result.tables[i];
        out << (i == 0 ? "" : ", ") << jsonString(table.name)
            << ": {\"file\": " << jsonString(tableFile(named.name, table.name))
            << ", \"columns\": [";
        for (std::size_t j = 0; j < table.columns.size(); ++j) {
            out << (j == 0 ? "" : ", ") << "{\"name\": " << jsonString(table.columns[j].name)
                << ", \"unit\": " << jsonString(table.columns[j].unit) << "}";
        }
        out << "]}";
    }
    out << "},\n      \"notes\": [";
    for (std::size_t i = 0; i < result.notes.size(); ++i) {
        out << (i == 0 ? "" : ", ") << jsonString(result.notes[i]);
    }
    out << "]\n    }";
    return out.str();
}

std::string gateJson(const EvaluationOutcome& outcome) {
    const auto& verdict = outcome.verdict;
    std::ostringstream out;
    out << "{\"outcome\": " << jsonString(eval::toString(verdict.outcome))
        << ", \"detail\": " << jsonString(verdict.detail)
        << ", \"baselines\": " << jsonString(outcome.baselines) << ", \"checks\": [";
    for (std::size_t i = 0; i < verdict.checks.size(); ++i) {
        const auto& check = verdict.checks[i];
        out << (i == 0 ? "" : ", ") << "{\"metric\": " << jsonString(check.metric) << ", \""
            << (check.upper ? "max" : "min") << "\": " << jsonNumber(check.limit)
            << ", \"value\": " << jsonNumber(check.measured)
            << ", \"passed\": " << (check.passed ? "true" : "false") << "}";
    }
    out << "]}";
    return out.str();
}

std::string trajectoriesJson(const fs::path& dir) {
    std::vector<fs::path> files;
    if (fs::is_directory(dir / "trajectory")) {
        for (const auto& entry : fs::directory_iterator(dir / "trajectory")) {
            if (entry.path().extension() == ".tum") files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    std::string out = "{";
    for (const auto& file : files) {
        out += (out.size() == 1 ? "\n" : ",\n") + std::string("    ")
               + jsonString(file.stem().string()) + ": {\"file\": "
               + jsonString("trajectory/" + file.filename().string()) + ", \"digest\": "
               + jsonString(eval::trajectoryFileDigest(file.string())) + "}";
    }
    return out + (files.empty() ? "}" : "\n  }");
}

}  // namespace

RunDirInputs::RunDirInputs(fs::path dir) : dir_(std::move(dir)) {
    const fs::path identity = dir_ / "identity.yaml";
    try {
        const YAML::Node root = YAML::LoadFile(identity.string());
        if (root["effective_hash"]) effective_hash_ = root["effective_hash"].as<std::string>();
        const YAML::Node effective = root["effective"];
        const auto begin = parseNanoseconds(effective["source.evaluation_window_begin_ns"]);
        const auto end = parseNanoseconds(effective["source.evaluation_window_end_ns"]);
        if (begin && end) window_ = fw::TimeRange{fw::Timestamp{*begin}, fw::Timestamp{*end}};
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("读不懂任务记录的身份 " + identity.string() + ": " + e.what());
    }
}

eval::ComparablePair RunDirInputs::pair(std::string_view estimate) const {
    const std::string estimate_file = "trajectory/" + std::string(estimate) + ".tum";
    const std::string truth_file = "trajectory/groundtruth.tum";
    if (!fs::exists(dir_ / estimate_file)) {
        throw std::runtime_error("任务记录里没有估计轨迹 " + (dir_ / estimate_file).string());
    }
    if (!fs::exists(dir_ / truth_file)) {
        throw std::runtime_error("任务记录 " + dir_.string()
                                 + " 里没有真值：数据没有自带真值，evaluation.groundtruth 也没给");
    }
    constexpr auto kTum = eval::TrajectoryFormat::kTum;
    eval::ComparablePair pair{
        .estimate = eval::readTrajectory((dir_ / estimate_file).string(), kTum),
        .reference = eval::readTrajectory((dir_ / truth_file).string(), kTum),
        .notes = {}};
    for (const auto& file : {estimate_file, truth_file}) {
        if (std::find(used_.begin(), used_.end(), file) == used_.end()) used_.push_back(file);
    }
    // 真值写进任务记录时已经换算到比较用的传感器上；估计跟踪 body，不同就按生效外参换算
    const std::string& target = pair.reference.tracked();
    const std::string& tracked = pair.estimate.tracked();
    if (!target.empty() && !tracked.empty() && tracked != target) {
        if (tracked != "body") {
            throw std::runtime_error("估计跟踪的不是 body（" + tracked + "），无从换算到 "
                                     + target);
        }
        const auto calibration = config::loadCalibrationFile((dir_ / "calibration.yaml").string());
        const auto T_body_target = calibration.T_body_sensor(target);
        if (!T_body_target) {
            throw std::runtime_error("比较要用 " + target
                                     + " 的外参，但任务记录里生效的标定没有它");
        }
        pair.estimate = eval::rightMultiplied(pair.estimate, *T_body_target, target);
        pair.notes.push_back("估计从 body 按生效外参换算到 " + target);
    }
    return pair;
}

std::optional<fw::TimeRange> RunDirInputs::targetWindow() const { return window_; }

std::vector<std::string> RunDirInputs::takeUsed() const { return std::exchange(used_, {}); }

Completeness completenessOf(const fs::path& dir) {
    return Completeness{.state = run_dir::readComplete(dir) ? eval::RunCompleteness::kComplete
                                                            : eval::RunCompleteness::kIncomplete,
                        .source = "本次运行",
                        .declaration = {}};
}

bool needsGroundTruth(const ResolvedTask& task) {
    return std::any_of(task.evaluators.begin(), task.evaluators.end(), [](const auto& entry) {
        return entry.second.make()->needs().groundtruth;
    });
}

EvaluationOutcome evaluateRunDir(const fs::path& dir, const ResolvedTask& task,
                                 eval::RunCompleteness completeness, std::ostream& out) {
    const auto& c = task.config;
    const RunDirInputs inputs(dir);

    // 先全部算完、过完门禁，再动任务记录：中途出错时，上一次的结论与表格原样留着
    EvaluationOutcome outcome;
    eval::GateRequest request;
    for (const auto& [name, slot] : task.evaluators) {
        const auto evaluator = slot.make();
        NamedEvaluation named{.name = name,
                              .spec = slot.resolved(),
                              .result = evaluator->evaluate(inputs),
                              .inputs = inputs.takeUsed()};
        checkResult(name, named.result);
        out << summaryLine(named) << '\n';
        for (const auto& scalar : named.result.scalars) {
            request.metrics[name + "." + scalar.name] = scalar.value;
        }
        request.metrics[name + ".samples"] = static_cast<double>(named.result.samples);
        flattenProtocol(named.spec, "", request.evaluators[name]);
        outcome.evaluations.push_back(std::move(named));
    }

    std::vector<eval::DatasetBaseline> baselines;
    outcome.baselines = c.evaluation.baselines;
    if (!outcome.baselines.empty()) {
        std::vector<std::string> notes;
        baselines = eval::loadBaselines(outcome.baselines, &notes);
        for (const auto& note : notes) out << "注意: " << note << '\n';
    }
    request.dataset = c.task.name;
    request.config_hash = inputs.effectiveHash();
    if (const fs::path truth = dir / "trajectory" / "groundtruth.tum"; fs::exists(truth)) {
        request.groundtruth_digest = eval::trajectoryFileDigest(truth.string());
    }
    request.completeness = completeness;
    request.allow_incomplete = c.evaluation.allow_incomplete;
    request.baselines = outcome.baselines.empty() ? nullptr : &baselines;
    outcome.verdict = eval::gate(request);
    outcome.exit_code = eval::exitCodeFor(outcome.verdict);

    fs::remove_all(dir / "evaluation");  // 上一次评测的表格不留
    fs::create_directories(dir / "evaluation");
    for (const auto& named : outcome.evaluations) {
        for (const auto& table : named.result.tables) writeTable(dir, named.name, table);
    }
    return outcome;
}

ResultIdentity identityOf(const ResolvedTask& task) {
    const auto& c = task.config;
    return ResultIdentity{.task = c.task.name,
                          .mode = c.task.mode,
                          .task_hash = config::configHash(c),
                          .system_hash = config::systemHash(c)};
}

void writeResult(const fs::path& dir, const ResultIdentity& identity,
                 const Completeness& completeness, const EvaluationOutcome* evaluated) {
    const std::string status =
        evaluated == nullptr ? "done" : eval::toString(evaluated->verdict.outcome);
    const std::string detail = evaluated == nullptr ? "" : evaluated->verdict.detail;
    const int exit_code = evaluated == nullptr ? 0 : evaluated->exit_code;
    std::ostringstream text;
    text << "{\n  \"schema\": " << run_dir::kResultSchema << ",\n  \"kind\": \"run\",\n  \"task\": "
         << jsonString(identity.task) << ",\n  \"mode\": " << jsonString(identity.mode)
         << ",\n  \"software_version\": " << jsonString(SIMPLESLAM_VERSION)
         << ",\n  \"task_hash\": " << jsonString(identity.task_hash)
         << ",\n  \"system_hash\": " << jsonString(identity.system_hash)
         << ",\n  \"complete\": "
         << (completeness.state == eval::RunCompleteness::kComplete
                 ? "true"
                 : (completeness.state == eval::RunCompleteness::kIncomplete ? "false" : "null"))
         << ",\n  \"completeness\": " << jsonString(eval::toString(completeness.state))
         << ",\n  \"completeness_source\": " << jsonString(completeness.source)
         << ",\n  \"declaration\": " << jsonString(completeness.declaration)
         << ",\n  \"status\": " << jsonString(status) << ",\n  \"detail\": " << jsonString(detail)
         << ",\n  \"exit_code\": " << exit_code
         << ",\n  \"trajectories\": " << trajectoriesJson(dir) << ",\n  \"evaluations\": {";
    if (evaluated != nullptr) {
        for (std::size_t i = 0; i < evaluated->evaluations.size(); ++i) {
            text << (i == 0 ? "\n" : ",\n") << evaluationJson(evaluated->evaluations[i], dir);
        }
        if (!evaluated->evaluations.empty()) text << "\n  ";
    }
    text << "},\n  \"gate\": " << (evaluated == nullptr ? "null" : gateJson(*evaluated)) << "\n}\n";
    run_dir::writeText(dir / "result.json", text.str());
}

}  // namespace simpleslam::app
