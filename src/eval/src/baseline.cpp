// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/eval/baseline.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <type_traits>
#include <variant>

namespace simpleslam::eval {

namespace {

double nodeAs(const YAML::Node& node, double fallback) {
    return node && node.IsScalar() ? node.as<double>() : fallback;
}

}  // namespace

std::string validateBaseline(const DatasetBaseline& baseline) {
    if (baseline.bounds.empty()) return "基线没有任何门限，比较不出结论";
    if (!std::isfinite(baseline.min_pairs) || baseline.min_pairs < 1.0) {
        return "min_pairs 必须是不小于 1 的有限数";
    }
    for (const auto& bound : baseline.bounds) {
        if (bound.metric.empty()) return "门限缺少指标名";
        if (std::isnan(bound.max_value) || std::isnan(bound.min_value)) {
            // NaN 门限的任何比较都是假，会静静地放行一切
            return "门限 " + bound.metric + " 含 NaN";
        }
        if (!bound.hasUpper() && !bound.hasLower()) {
            return "门限 " + bound.metric + " 既没有上限也没有下限";
        }
        if (bound.hasUpper() && bound.hasLower() && bound.min_value > bound.max_value) {
            return "门限 " + bound.metric + " 的下限大于上限";
        }
    }
    return {};
}

namespace {

/// 评测协议的字段表：报告与基线的读写都走它，往返覆盖每一个正式字段。
///
/// 读写两边各写一遍字段名，迟早会有一边漏掉一个——写出去的字段读不回来，
/// 带着真实协议的基线就成了一份读不了的文件。
using ProtocolMember =
    std::variant<std::string EvaluationProtocol::*, double EvaluationProtocol::*,
                 bool EvaluationProtocol::*>;

struct ProtocolField {
    const char* key;
    ProtocolMember member;
};

const std::array<ProtocolField, 10>& protocolFields() {
    static const std::array<ProtocolField, 10> fields{{
        {"alignment", &EvaluationProtocol::alignment},
        {"alignment_requested", &EvaluationProtocol::alignment_requested},
        {"association", &EvaluationProtocol::association},
        {"association_tolerance_s", &EvaluationProtocol::association_tolerance_s},
        {"max_interpolation_gap_s", &EvaluationProtocol::max_interpolation_gap_s},
        {"rpe_delta_kind", &EvaluationProtocol::rpe_delta_kind},
        {"rpe_delta", &EvaluationProtocol::rpe_delta},
        {"rpe_normalized", &EvaluationProtocol::rpe_normalized},
        {"window_begin_s", &EvaluationProtocol::window_begin_s},
        {"window_end_s", &EvaluationProtocol::window_end_s},
    }};
    return fields;
}

void emitProtocol(YAML::Emitter& out, const EvaluationProtocol& protocol) {
    out << YAML::BeginMap;
    for (const auto& field : protocolFields()) {
        out << YAML::Key << field.key << YAML::Value;
        std::visit(
            [&](auto member) {
                using Value = std::remove_cvref_t<decltype(protocol.*member)>;
                if constexpr (std::is_same_v<Value, double>) {
                    // 时间窗是绝对秒数，默认精度会丢掉小数位，读回来就与原值不等
                    out << YAML::DoublePrecision(17) << protocol.*member;
                } else {
                    out << protocol.*member;
                }
            },
            field.member);
    }
    out << YAML::EndMap;
}

EvaluationProtocol parseProtocol(const YAML::Node& node, const std::string& where) {
    if (!node.IsMap()) throw BaselineError("评测协议必须是映射: " + where);
    EvaluationProtocol parsed;
    for (const auto& item : node) {
        const auto key = item.first.as<std::string>();
        const auto& fields = protocolFields();
        const auto field = std::find_if(fields.begin(), fields.end(), [&](const ProtocolField& f) {
            return key == f.key;
        });
        if (field == fields.end()) throw BaselineError("评测协议里有未知键 " + key + ": " + where);
        std::visit(
            [&](auto member) {
                using Value = std::remove_cvref_t<decltype(parsed.*member)>;
                parsed.*member = item.second.as<Value>();
            },
            field->member);
    }
    return parsed;
}

}  // namespace

BaselineVerdict checkMeasurementValidity(const std::map<std::string, double>& measured,
                                         double min_pairs,
                                         const std::vector<std::string>& required_families) {
    BaselineVerdict verdict;
    for (const auto& [key, value] : measured) {
        if (!std::isfinite(value)) verdict.invalid_metrics.push_back(key);
    }
    if (!verdict.invalid_metrics.empty()) {
        verdict.outcome = BaselineOutcome::kInvalid;
        verdict.detail = "存在非有限指标";
        return verdict;
    }
    // 样本数按指标族分别核验：一族有样本不能替另一族作数
    for (const auto& family : required_families) {
        const auto it = measured.find(family + ".pairs");
        const double pairs = it == measured.end() ? 0.0 : it->second;
        if (pairs >= min_pairs) continue;
        verdict.outcome = BaselineOutcome::kInvalid;
        verdict.detail = family + " 的有效关联只有 "
                         + std::to_string(static_cast<long long>(pairs)) + " 对，低于 "
                         + std::to_string(static_cast<long long>(min_pairs));
        return verdict;
    }
    return verdict;
}

namespace {

/// 基线实际用到了哪些指标族。只有它们才必须有样本。
std::vector<std::string> requiredFamilies(const DatasetBaseline& baseline) {
    std::vector<std::string> families;
    for (const auto& bound : baseline.bounds) {
        const auto dot = bound.metric.find('.');
        if (dot == std::string::npos) continue;
        const std::string family = bound.metric.substr(0, dot);
        if (family != "ate" && family != "rpe") continue;
        if (std::find(families.begin(), families.end(), family) == families.end()) {
            families.push_back(family);
        }
    }
    return families;
}

}  // namespace

BaselineVerdict checkBaseline(const DatasetBaseline& baseline,
                              const std::map<std::string, double>& measured,
                              const std::string& config_hash,
                              const EvaluationProtocol* protocol) {
    if (const auto problem = validateBaseline(baseline); !problem.empty()) {
        BaselineVerdict broken;
        broken.outcome = BaselineOutcome::kInvalid;
        broken.detail = "基线本身不合法：" + problem;
        return broken;
    }

    BaselineVerdict verdict;
    if (!baseline.config_hash.empty()) {
        // 基线绑定了身份，调用方就必须给出身份。缺身份不是"默认相符"，
        // 而是"无从判断这次结果能不能和它比"。
        if (config_hash.empty() || baseline.config_hash != config_hash) {
            verdict.config_mismatch = true;
        }
    }

    // ── 先判有效性。无法评价不能当成通过，也不能当成失败 ──
    // 基线用到哪些指标族，哪些族就必须真的有样本：ATE 有三对、RPE 一对都没有时，
    // 一条 RPE 门限不能凭 rpe=0 过线。
    if (auto validity =
            checkMeasurementValidity(measured, baseline.min_pairs, requiredFamilies(baseline));
        validity.outcome != BaselineOutcome::kPass) {
        return validity;
    }

    for (const auto& bound : baseline.bounds) {
        const auto it = measured.find(bound.metric);
        if (it == measured.end()) {
            verdict.missing_metrics.push_back(bound.metric);
            continue;
        }
        if (bound.hasUpper() && it->second > bound.max_value) {
            verdict.failures.push_back(BaselineFailure{bound.metric, it->second, bound.max_value,
                                                       true});
        }
        if (bound.hasLower() && it->second < bound.min_value) {
            verdict.failures.push_back(BaselineFailure{bound.metric, it->second, bound.min_value,
                                                       false});
        }
    }

    if (!verdict.missing_metrics.empty()) {
        verdict.outcome = BaselineOutcome::kInvalid;  // 基线要的指标没测出来 = 没结论
        verdict.detail = "缺少基线要求的指标";
        return verdict;
    }

    // 可比性先于门限：Fail 的含义是"有效、可比较、没过线"。不可比较时照样附上数值
    // 比较的结果（failures），但主结论只能是 Incomparable。
    if (baseline.protocol && (protocol == nullptr || !baseline.protocol->sameMeasurement(*protocol))) {
        verdict.outcome = BaselineOutcome::kIncomparable;
        verdict.detail = "评测协议与基线不同（对齐、关联、RPE 间隔或时间窗）";
        return verdict;
    }
    if (verdict.config_mismatch) {
        verdict.outcome = BaselineOutcome::kIncomparable;
        verdict.detail = config_hash.empty() ? "基线绑定了配置哈希，本次没有提供"
                                             : "配置哈希与基线不符";
        return verdict;
    }
    verdict.outcome = verdict.failures.empty() ? BaselineOutcome::kPass : BaselineOutcome::kFail;
    return verdict;
}

BaselineVerdict gate(const GateRequest& request) {
    // 有效性：零关联、非有限值都是"这次没测出东西"。基线用到的指标族必须有样本。
    std::vector<std::string> families{"ate"};
    const DatasetBaseline* matched = nullptr;
    if (request.baselines != nullptr && !request.dataset.empty()) {
        for (const auto& baseline : *request.baselines) {
            if (baseline.dataset == request.dataset) {
                matched = &baseline;
                break;
            }
        }
    }
    if (matched != nullptr) {
        for (const auto& bound : matched->bounds) {
            const auto dot = bound.metric.find('.');
            if (dot == std::string::npos) continue;
            const std::string family = bound.metric.substr(0, dot);
            if ((family == "ate" || family == "rpe")
                && std::find(families.begin(), families.end(), family) == families.end()) {
                families.push_back(family);
            }
        }
    }
    auto verdict = checkMeasurementValidity(request.metrics,
                                            matched != nullptr ? matched->min_pairs : 1.0,
                                            families);
    if (verdict.outcome != BaselineOutcome::kPass) return verdict;

    // 完整性：不完整的运行默认不下结论
    const RunCompleteness completeness = request.completeness;
    if (completeness == RunCompleteness::kIncomplete && !request.allow_incomplete) {
        verdict.outcome = BaselineOutcome::kInvalid;
        verdict.detail = "运行不完整（输入、处理或输出有缺口），严格模式不下结论";
        return verdict;
    }

    // 没有基线：有效就是这次评测的全部结论。说不清是否全程的，要自己说出来
    if (request.baselines == nullptr) {
        switch (completeness) {
            case RunCompleteness::kIncomplete:
                verdict.detail = "运行不完整：这是显式允许的前缀结果，不代表全程";
                break;
            case RunCompleteness::kUnknown:
                verdict.detail = "运行完整性未知：误差测量有效，但说不清它是否覆盖全程";
                break;
            case RunCompleteness::kDeclaredComplete:
                verdict.detail = "运行完整性为人工声明，不是运行结局给出的证据";
                break;
            case RunCompleteness::kComplete:
                break;
        }
        return verdict;
    }

    // 有基线：说不清是否全程的数字不能拿去和基线比高低
    if (completeness == RunCompleteness::kUnknown) {
        verdict.outcome = BaselineOutcome::kIncomparable;
        verdict.detail =
            "运行完整性未知，不能与基线比较（需要绑定了这条轨迹的运行报告，或人工声明完整）";
        return verdict;
    }

    // 身份匹配：不给数据集名就无从匹配，也不默认取第一条
    if (request.dataset.empty()) {
        verdict.outcome = BaselineOutcome::kIncomparable;
        verdict.detail = "没有指定数据集，无法匹配基线";
        return verdict;
    }
    if (matched == nullptr) {
        verdict.outcome = BaselineOutcome::kIncomparable;
        verdict.detail = "基线文件里没有 " + request.dataset + " 这一条";
        return verdict;
    }
    // 前缀运行只能与前缀基线比，完整运行只能与完整基线比
    const bool prefix_run = completeness == RunCompleteness::kIncomplete;
    if (prefix_run != matched->prefix) {
        verdict.outcome = BaselineOutcome::kIncomparable;
        verdict.detail = prefix_run ? "不完整的运行不能与完整运行的基线比较"
                                    : "完整运行不能与前缀基线比较";
        return verdict;
    }
    auto compared = checkBaseline(*matched, request.metrics, request.config_hash, &request.protocol);
    if (completeness == RunCompleteness::kDeclaredComplete) {
        compared.detail += compared.detail.empty() ? "" : "；";
        compared.detail += "运行完整性为人工声明，不是运行结局给出的证据";
    }
    return compared;
}

std::optional<RunCompleteness> parseRunCompleteness(std::string_view text) {
    for (const auto value : {RunCompleteness::kComplete, RunCompleteness::kIncomplete,
                             RunCompleteness::kUnknown, RunCompleteness::kDeclaredComplete}) {
        if (text == toString(value)) return value;
    }
    return std::nullopt;
}

int exitCodeFor(const BaselineVerdict& verdict, bool allow_incomparable) {
    switch (verdict.outcome) {
        case BaselineOutcome::kPass: return 0;
        case BaselineOutcome::kFail: return 3;
        case BaselineOutcome::kInvalid: return 4;
        case BaselineOutcome::kIncomparable: return allow_incomparable ? 0 : 5;
    }
    return 4;
}

std::vector<DatasetBaseline> loadBaselines(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw BaselineError("读取基线文件失败 " + path + ": " + e.what());
    }
    const YAML::Node list = root["baselines"] ? root["baselines"] : root;
    if (!list.IsSequence()) throw BaselineError("基线文件的顶层必须是序列或含 baselines 键: " + path);

    std::vector<DatasetBaseline> baselines;
    std::set<std::string> seen;
    for (const auto& entry : list) {
        if (!entry.IsMap()) throw BaselineError("基线条目必须是映射: " + path);
        // 未知键一律拒绝：拼错的 "min_pair" 静静地被忽略，门限就松了
        for (const auto& item : entry) {
            static const std::set<std::string> known{"dataset",   "config_hash", "recorded_at",
                                                     "note",      "min_pairs",   "bounds",
                                                     "protocol",  "prefix"};
            const auto key = item.first.as<std::string>();
            if (known.count(key) == 0) throw BaselineError("基线条目里有未知键 " + key + ": " + path);
        }
        DatasetBaseline baseline;
        baseline.dataset = entry["dataset"] ? entry["dataset"].as<std::string>() : std::string{};
        if (baseline.dataset.empty()) throw BaselineError("基线缺少 dataset 字段: " + path);
        if (!seen.insert(baseline.dataset).second) {
            throw BaselineError("基线文件里 " + baseline.dataset + " 出现了两次: " + path);
        }
        if (entry["prefix"]) baseline.prefix = entry["prefix"].as<bool>();
        if (const YAML::Node protocol = entry["protocol"]) {
            baseline.protocol = parseProtocol(protocol, path);
        }
        if (entry["config_hash"]) baseline.config_hash = entry["config_hash"].as<std::string>();
        if (entry["recorded_at"]) baseline.recorded_at = entry["recorded_at"].as<std::string>();
        if (entry["note"]) baseline.note = entry["note"].as<std::string>();
        if (entry["min_pairs"]) baseline.min_pairs = entry["min_pairs"].as<double>();

        const YAML::Node bounds = entry["bounds"];
        if (!bounds || !bounds.IsMap()) {
            throw BaselineError("基线 " + baseline.dataset + " 缺少 bounds 映射: " + path);
        }
        for (const auto& item : bounds) {
            MetricBound bound;
            bound.metric = item.first.as<std::string>();
            const YAML::Node& value = item.second;
            if (value.IsScalar()) {
                bound.max_value = value.as<double>();  // 标量默认按上限理解
            } else if (value.IsMap()) {
                for (const auto& limit : value) {
                    const auto key = limit.first.as<std::string>();
                    if (key != "max" && key != "min") {
                        throw BaselineError("门限 " + bound.metric + " 里有未知键 " + key);
                    }
                }
                bound.max_value = nodeAs(value["max"], bound.max_value);
                bound.min_value = nodeAs(value["min"], bound.min_value);
            } else {
                throw BaselineError("门限 " + bound.metric + " 必须是标量或 {max, min} 映射");
            }
            baseline.bounds.push_back(std::move(bound));
        }
        // 入仓即校验：写错的基线会静静地放行一切，越早报越好
        if (const auto problem = validateBaseline(baseline); !problem.empty()) {
            throw BaselineError("基线 " + baseline.dataset + " 不合法（" + problem + "）: " + path);
        }
        baselines.push_back(std::move(baseline));
    }
    return baselines;
}

void writeBaselines(const std::string& path, const std::vector<DatasetBaseline>& baselines) {
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "baselines" << YAML::Value << YAML::BeginSeq;
    for (const auto& baseline : baselines) {
        out << YAML::BeginMap;
        out << YAML::Key << "dataset" << YAML::Value << baseline.dataset;
        if (!baseline.config_hash.empty()) {
            out << YAML::Key << "config_hash" << YAML::Value << baseline.config_hash;
        }
        if (!baseline.recorded_at.empty()) {
            out << YAML::Key << "recorded_at" << YAML::Value << baseline.recorded_at;
        }
        if (!baseline.note.empty()) out << YAML::Key << "note" << YAML::Value << baseline.note;
        out << YAML::Key << "min_pairs" << YAML::Value << baseline.min_pairs;
        if (baseline.prefix) out << YAML::Key << "prefix" << YAML::Value << true;
        if (baseline.protocol) {
            out << YAML::Key << "protocol" << YAML::Value;
            emitProtocol(out, *baseline.protocol);
        }
        out << YAML::Key << "bounds" << YAML::Value << YAML::BeginMap;
        for (const auto& bound : baseline.bounds) {
            out << YAML::Key << bound.metric << YAML::Value;
            if (bound.hasUpper() && !bound.hasLower()) {
                out << bound.max_value;
            } else {
                out << YAML::BeginMap;
                if (bound.hasUpper()) out << YAML::Key << "max" << YAML::Value << bound.max_value;
                if (bound.hasLower()) out << YAML::Key << "min" << YAML::Value << bound.min_value;
                out << YAML::EndMap;
            }
        }
        out << YAML::EndMap << YAML::EndMap;
    }
    out << YAML::EndSeq << YAML::EndMap;

    std::ofstream file(path);
    if (!file) throw BaselineError("打不开基线输出文件: " + path);
    file << out.c_str() << '\n';
    if (!file) throw BaselineError("写基线文件失败: " + path);
}

std::map<std::string, double> toMetricMap(const AteResult& ate, const RpeResult& rpe) {
    std::map<std::string, double> metrics;
    for (const auto& [key, value] : flatten(ate, rpe)) metrics.emplace(key, value);
    return metrics;
}

std::string renderReport(const EvaluationReport& report) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "dataset" << YAML::Value << report.dataset;
    out << YAML::Key << "config_hash" << YAML::Value << report.config_hash;
    out << YAML::Key << "code_revision" << YAML::Value << report.code_revision;
    out << YAML::Key << "created_at" << YAML::Value << report.created_at;
    out << YAML::Key << "alignment" << YAML::Value << report.alignment;
    out << YAML::Key << "input_complete" << YAML::Value << report.input_complete;
    // 完整性三态与出处；人工声明的原话留在这里
    out << YAML::Key << "run_completeness" << YAML::Value << toString(report.completeness);
    if (!report.completeness_source.empty()) {
        out << YAML::Key << "completeness_source" << YAML::Value << report.completeness_source;
    }
    if (!report.completeness_declaration.empty()) {
        out << YAML::Key << "completeness_declaration" << YAML::Value
            << report.completeness_declaration;
    }
    // 这份报告为哪条轨迹作证：别处拿它当运行凭据时核对内容摘要
    if (!report.trajectory_digest.empty()) {
        out << YAML::Key << "trajectory_file" << YAML::Value << report.trajectory_file;
        out << YAML::Key << "trajectory_digest" << YAML::Value << report.trajectory_digest;
    }
    // 评测协议随结果保存：同一份配置换个容差或 RPE 间隔，结论就不可直接互比
    out << YAML::Key << "protocol" << YAML::Value;
    emitProtocol(out, report.protocol);
    if (!report.run_problems.empty()) {
        out << YAML::Key << "run_problems" << YAML::Value << YAML::BeginSeq;
        for (const auto& problem : report.run_problems) out << problem;
        out << YAML::EndSeq;
    }
    if (!report.notes.empty()) {
        out << YAML::Key << "notes" << YAML::Value << YAML::BeginSeq;
        for (const auto& note : report.notes) out << note;
        out << YAML::EndSeq;
    }
    if (!report.effective.empty()) {
        out << YAML::Key << "effective" << YAML::Value << YAML::BeginMap;
        for (const auto& [key, value] : report.effective) out << YAML::Key << key << YAML::Value << value;
        out << YAML::EndMap;
    }
    out << YAML::Key << "metrics" << YAML::Value << YAML::BeginMap;
    for (const auto& [key, value] : report.metrics) {
        out << YAML::Key << key << YAML::Value << value;
    }
    out << YAML::EndMap;
    out << YAML::Key << "verdict" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "outcome" << YAML::Value << toString(report.verdict.outcome);
    out << YAML::Key << "passed" << YAML::Value << report.verdict.passed();
    out << YAML::Key << "config_mismatch" << YAML::Value << report.verdict.config_mismatch;
    if (!report.verdict.detail.empty()) {
        out << YAML::Key << "detail" << YAML::Value << report.verdict.detail;
    }
    out << YAML::Key << "invalid_metrics" << YAML::Value << YAML::BeginSeq;
    for (const auto& metric : report.verdict.invalid_metrics) out << metric;
    out << YAML::EndSeq;
    out << YAML::Key << "failures" << YAML::Value << YAML::BeginSeq;
    for (const auto& failure : report.verdict.failures) {
        out << YAML::BeginMap;
        out << YAML::Key << "metric" << YAML::Value << failure.metric;
        out << YAML::Key << "measured" << YAML::Value << failure.measured;
        out << YAML::Key << (failure.upper ? "max" : "min") << YAML::Value << failure.bound;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "missing_metrics" << YAML::Value << YAML::BeginSeq;
    for (const auto& metric : report.verdict.missing_metrics) out << metric;
    out << YAML::EndSeq;
    out << YAML::EndMap << YAML::EndMap;
    return out.c_str();
}

void writeReport(const std::string& path, const EvaluationReport& report) {
    std::ofstream file(path);
    if (!file) throw BaselineError("打不开报告输出文件: " + path);
    file << renderReport(report) << '\n';
    if (!file) throw BaselineError("写报告文件失败: " + path);
}

RunManifest readRunManifest(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw BaselineError("读取运行报告失败 " + path + ": " + e.what());
    }
    if (!root.IsMap()) throw BaselineError("运行报告的顶层必须是映射: " + path);
    RunManifest manifest;
    const YAML::Node completeness = root["run_completeness"];
    if (!completeness) throw BaselineError("运行报告里没有 run_completeness: " + path);
    const auto parsed = parseRunCompleteness(completeness.as<std::string>());
    if (!parsed) {
        throw BaselineError("运行报告里的 run_completeness 认不出来: "
                            + completeness.as<std::string>());
    }
    manifest.completeness = *parsed;
    if (root["trajectory_file"]) manifest.trajectory_file = root["trajectory_file"].as<std::string>();
    if (root["trajectory_digest"]) {
        manifest.trajectory_digest = root["trajectory_digest"].as<std::string>();
    }
    if (const YAML::Node problems = root["run_problems"]) {
        for (const auto& problem : problems) manifest.run_problems.push_back(problem.as<std::string>());
    }
    if (const YAML::Node protocol = root["protocol"]) manifest.protocol = parseProtocol(protocol, path);
    return manifest;
}

}  // namespace simpleslam::eval
