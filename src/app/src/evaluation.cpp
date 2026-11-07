// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include "evaluation.hpp"

#include "run_dir.hpp"

#include <SimpleSLAM/config/calibration_io.hpp>
#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/eval/association.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>

#include <algorithm>
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

/// 一个插件参数的文本，往返无损。
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

}  // namespace

namespace {

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

}  // namespace

std::string summaryLine(const std::string& name, const std::string& type,
                        const eval::Evaluation& result) {
    std::string line = std::format("评测 {}（{}）: ", name, type);
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

namespace {

std::string evaluationJson(const NamedEvaluation& named) {
    const auto& result = named.result;
    std::ostringstream out;
    out << "    " << jsonString(named.name)
        << ": {\n      \"type\": " << jsonString(named.spec.type)
        << ",\n      \"params\": " << paramsJson(named.spec)
        << ",\n      \"status\": " << jsonString(result.samples > 0 ? "ok" : "no_result")
        << ",\n      \"samples\": " << result.samples << ",\n      \"inputs\": [";
    for (std::size_t i = 0; i < named.inputs.size(); ++i) {
        out << (i == 0 ? "" : ", ") << jsonString(named.inputs[i]);
    }
    out << "],\n      \"scalars\": {";
    for (std::size_t i = 0; i < result.scalars.size(); ++i) {
        const auto& scalar = result.scalars[i];
        out << (i == 0 ? "\n" : ",\n") << "        " << jsonString(scalar.name)
            << ": {\"value\": " << jsonNumber(scalar.value)
            << ", \"unit\": " << jsonString(scalar.unit)
            << ", \"label\": " << jsonString(scalar.label);
        if (scalar.weight) out << ", \"weight\": " << jsonNumber(*scalar.weight);
        out << "}";
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

}  // namespace

RunDirInputs::RunDirInputs(fs::path dir, std::optional<fw::TimeRange> window)
    : dir_(std::move(dir)), window_(window) {}

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
    use(estimate_file);
    use(truth_file);
    // 真值写进任务记录时已经换算到比较用的传感器上；估计跟踪 body，不同就按生效外参换算
    toReferenceSensor(pair, [this] {
        return config::loadCalibrationFile((dir_ / "calibration.yaml").string());
    });
    return pair;
}

std::optional<fw::TimeRange> RunDirInputs::targetWindow() const { return window_; }

fw::Trajectory RunDirInputs::estimate(std::string_view name) const {
    const std::string file = "trajectory/" + std::string(name) + ".tum";
    if (!fs::exists(dir_ / file)) {
        throw std::runtime_error("任务记录里没有估计轨迹 " + (dir_ / file).string());
    }
    use(file);
    return eval::readTrajectory((dir_ / file).string(), eval::TrajectoryFormat::kTum);
}

std::vector<eval::MetricSample> RunDirInputs::metrics(std::string_view key) const {
    auto samples = run_dir::readMetrics(dir_, key);
    use("metrics.csv");
    return samples;
}

std::vector<eval::EventSample> RunDirInputs::events(std::string_view key) const {
    auto samples = run_dir::readEvents(dir_, key);
    use("events.jsonl");
    return samples;
}

void RunDirInputs::use(const std::string& file) const {
    if (std::find(used_.begin(), used_.end(), file) == used_.end()) used_.push_back(file);
}

void toReferenceSensor(eval::ComparablePair& pair,
                       const std::function<fw::CalibrationData()>& calibration) {
    const std::string& target = pair.reference.tracked();
    const std::string& tracked = pair.estimate.tracked();
    if (target.empty() || tracked.empty() || tracked == target) return;
    if (tracked != "body") {
        throw std::runtime_error("估计跟踪的不是 body（" + tracked + "），无从换算到 " + target);
    }
    const auto T_body_target = calibration().T_body_sensor(target);
    if (!T_body_target) {
        throw std::runtime_error("比较要用 " + target + " 的外参，但生效的标定没有它");
    }
    pair.estimate = eval::rightMultiplied(pair.estimate, *T_body_target, target);
    pair.notes.push_back("估计从 body 按生效外参换算到 " + target);
}

std::vector<std::string> RunDirInputs::takeUsed() const { return std::exchange(used_, {}); }

bool needsGroundTruth(const ResolvedTask& task) {
    return std::any_of(task.evaluators.begin(), task.evaluators.end(), [](const auto& entry) {
        return entry.second.make()->needs().groundtruth;
    });
}

int exitCodeFor(std::optional<bool> complete) { return complete.value_or(true) ? 0 : 4; }

EvaluationOutcome evaluateRunDir(const fs::path& dir, const ResolvedTask& task,
                                 std::optional<fw::TimeRange> window, std::ostream& out) {
    const RunDirInputs inputs(dir, window);

    // 先全部算完、核对完，再动任务记录：中途出错时，上一次的结论与表格原样留着
    EvaluationOutcome outcome;
    for (const auto& [name, slot] : task.evaluators) {
        const auto evaluator = slot.make();
        NamedEvaluation named{.name = name,
                              .spec = slot.resolved(),
                              .result = evaluator->evaluate(inputs),
                              .inputs = inputs.takeUsed()};
        eval::checkEvaluation(name, named.result);
        out << summaryLine(named.name, named.spec.type, named.result) << '\n';
        outcome.evaluations.push_back(std::move(named));
    }

    fs::remove_all(dir / "evaluation");  // 上一次评测的表格不留
    fs::create_directories(dir / "evaluation");
    for (const auto& named : outcome.evaluations) {
        for (const auto& table : named.result.tables) writeTable(dir, named.name, table);
    }
    return outcome;
}

ResultHeader headerOf(const ResolvedTask& task) {
    const auto& c = task.config;
    return ResultHeader{.task = c.task.name, .mode = c.task.mode};
}

void writeResult(const fs::path& dir, const ResultHeader& header, std::optional<bool> complete,
                 const EvaluationOutcome* evaluated) {
    std::ostringstream text;
    text << "{\n  \"schema\": " << run_dir::kResultSchema << ",\n  \"kind\": \"run\",\n  \"task\": "
         << jsonString(header.task) << ",\n  \"mode\": " << jsonString(header.mode)
         << ",\n  \"software_version\": " << jsonString(SIMPLESLAM_VERSION)
         << ",\n  \"complete\": " << (complete ? (*complete ? "true" : "false") : "null")
         << ",\n  \"exit_code\": " << exitCodeFor(complete) << ",\n  \"evaluations\": {";
    if (evaluated != nullptr) {
        for (std::size_t i = 0; i < evaluated->evaluations.size(); ++i) {
            text << (i == 0 ? "\n" : ",\n") << evaluationJson(evaluated->evaluations[i]);
        }
        if (!evaluated->evaluations.empty()) text << "\n  ";
    }
    text << "}\n}\n";
    run_dir::writeText(dir / "result.json", text.str());
}

}  // namespace simpleslam::app
