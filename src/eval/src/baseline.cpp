// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/eval/baseline.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <format>
#include <fstream>
#include <map>
#include <set>
#include <string_view>

namespace simpleslam::eval {

namespace {

double nodeAs(const YAML::Node& node, double fallback) {
    return node && node.IsScalar() ? node.as<double>() : fallback;
}

/// 门限用到了哪些评测函数：指标名的第一段。只有它们才必须有样本。
std::vector<std::string> referencedEvaluators(const DatasetBaseline& baseline) {
    std::vector<std::string> names;
    for (const auto& bound : baseline.bounds) {
        const std::string name = bound.metric.substr(0, bound.metric.find('.'));
        if (std::find(names.begin(), names.end(), name) == names.end()) names.push_back(name);
    }
    return names;
}

std::string protocolText(const Protocol& protocol) {
    std::string text;
    for (const auto& [key, value] : protocol) {
        text += (text.empty() ? "" : ", ") + key + "=" + value;
    }
    return text;
}

}  // namespace

std::vector<BaselineCheck> BaselineVerdict::failures() const {
    std::vector<BaselineCheck> out;
    for (const auto& check : checks) {
        if (!check.passed) out.push_back(check);
    }
    return out;
}

std::string validateBaseline(const DatasetBaseline& baseline) {
    if (baseline.bounds.empty()) return "基线没有任何门限，比较不出结论";
    if (!std::isfinite(baseline.min_samples) || baseline.min_samples < 1.0) {
        return "min_samples 必须是不小于 1 的有限数";
    }
    for (const auto& bound : baseline.bounds) {
        if (bound.metric.empty()) return "门限缺少指标名";
        if (bound.metric.find('.') == std::string::npos) {
            return "门限 " + bound.metric + " 要写完整的指标名：<评测函数>.<量>.<统计>";
        }
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

BaselineVerdict checkMeasurementValidity(const std::map<std::string, double>& measured,
                                         double min_samples,
                                         const std::vector<std::string>& required) {
    BaselineVerdict verdict;
    for (const auto& [key, value] : measured) {
        if (!std::isfinite(value)) verdict.invalid_metrics.push_back(key);
    }
    if (!verdict.invalid_metrics.empty()) {
        verdict.outcome = BaselineOutcome::kInvalid;
        verdict.detail = "存在非有限指标";
        return verdict;
    }
    // 样本数按评测函数分别核验：一个有样本不能替另一个作数
    for (const auto& name : required) {
        const auto it = measured.find(name + ".samples");
        const double samples = it == measured.end() ? 0.0 : it->second;
        if (samples >= min_samples) continue;
        verdict.outcome = BaselineOutcome::kInvalid;
        verdict.detail = name + " 的有效样本只有 "
                         + std::to_string(static_cast<long long>(samples)) + " 个，低于 "
                         + std::to_string(static_cast<long long>(min_samples));
        return verdict;
    }
    return verdict;
}

BaselineVerdict checkBaseline(const DatasetBaseline& baseline,
                              const std::map<std::string, double>& measured,
                              const std::string& config_hash,
                              const std::map<std::string, Protocol>* evaluators,
                              const std::string& groundtruth_digest) {
    if (const auto problem = validateBaseline(baseline); !problem.empty()) {
        BaselineVerdict broken;
        broken.outcome = BaselineOutcome::kInvalid;
        broken.detail = "基线本身不合法：" + problem;
        return broken;
    }

    // ── 先判有效性。无法评价不能当成通过，也不能当成失败 ──
    const auto required = referencedEvaluators(baseline);
    if (auto validity = checkMeasurementValidity(measured, baseline.min_samples, required);
        validity.outcome != BaselineOutcome::kPass) {
        return validity;
    }

    BaselineVerdict verdict;
    if (!baseline.config_hash.empty()) {
        // 基线绑定了身份，调用方就必须给出身份。缺身份不是"默认相符"，
        // 而是"无从判断这次结果能不能和它比"。
        if (config_hash.empty() || baseline.config_hash != config_hash) {
            verdict.config_mismatch = true;
        }
    }

    for (const auto& bound : baseline.bounds) {
        const auto it = measured.find(bound.metric);
        if (it == measured.end()) {
            verdict.missing_metrics.push_back(bound.metric);
            continue;
        }
        if (bound.hasUpper()) {
            verdict.checks.push_back(BaselineCheck{.metric = bound.metric,
                                                   .measured = it->second,
                                                   .limit = bound.max_value,
                                                   .upper = true,
                                                   .passed = it->second <= bound.max_value});
        }
        if (bound.hasLower()) {
            verdict.checks.push_back(BaselineCheck{.metric = bound.metric,
                                                   .measured = it->second,
                                                   .limit = bound.min_value,
                                                   .upper = false,
                                                   .passed = it->second >= bound.min_value});
        }
    }

    if (!verdict.missing_metrics.empty()) {
        verdict.outcome = BaselineOutcome::kInvalid;  // 基线要的指标没测出来 = 没结论
        verdict.detail = "缺少基线要求的指标";
        return verdict;
    }

    // 可比性先于门限：Fail 的含义是"有效、可比较、没过线"
    for (const auto& name : required) {
        const auto recorded = baseline.evaluators.find(name);
        if (recorded == baseline.evaluators.end()) continue;  // 基线没钉这一个的协议
        const Protocol* current = nullptr;
        if (evaluators != nullptr) {
            if (const auto it = evaluators->find(name); it != evaluators->end()) {
                current = &it->second;
            }
        }
        if (current == nullptr || *current != recorded->second) {
            verdict.outcome = BaselineOutcome::kIncomparable;
            verdict.detail = name + " 的评测协议与基线不同（基线：" + protocolText(recorded->second)
                             + "；本次：" + (current == nullptr ? "没有" : protocolText(*current))
                             + "）";
            return verdict;
        }
    }
    if (!baseline.groundtruth_digest.empty() && baseline.groundtruth_digest != groundtruth_digest) {
        verdict.outcome = BaselineOutcome::kIncomparable;
        verdict.detail = groundtruth_digest.empty() ? "基线绑定了真值，本次没有提供真值摘要"
                                                    : "真值与基线录制时不同";
        return verdict;
    }
    if (verdict.config_mismatch) {
        verdict.outcome = BaselineOutcome::kIncomparable;
        verdict.detail = config_hash.empty() ? "基线绑定了有效身份，本次没有提供"
                                             : "有效身份与基线不符";
        return verdict;
    }
    verdict.outcome =
        verdict.failures().empty() ? BaselineOutcome::kPass : BaselineOutcome::kFail;
    return verdict;
}

BaselineVerdict gate(const GateRequest& request) {
    const DatasetBaseline* matched = nullptr;
    if (request.baselines != nullptr && !request.dataset.empty()) {
        for (const auto& baseline : *request.baselines) {
            if (baseline.dataset == request.dataset) {
                matched = &baseline;
                break;
            }
        }
    }

    // 有效性：非有限值是"这次没测出东西"；有匹配的基线时，门限用到的评测函数都要有样本，
    // 没有时至少要有一个评测函数测出了东西
    BaselineVerdict verdict;
    if (matched != nullptr) {
        verdict = checkMeasurementValidity(request.metrics, matched->min_samples,
                                           referencedEvaluators(*matched));
    } else {
        verdict = checkMeasurementValidity(request.metrics, 1.0, {});
        const bool measured_any =
            std::any_of(request.metrics.begin(), request.metrics.end(), [](const auto& metric) {
                return metric.first.ends_with(".samples") && metric.second >= 1.0;
            });
        if (verdict.outcome == BaselineOutcome::kPass && !measured_any) {
            verdict.outcome = BaselineOutcome::kInvalid;
            verdict.detail = "没有一个评测函数测出东西";
        }
    }
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
            "运行完整性未知，不能与基线比较（需要绑定了这条轨迹的运行结果，或人工声明完整）";
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
    auto compared = checkBaseline(*matched, request.metrics, request.config_hash,
                                  &request.evaluators, request.groundtruth_digest);
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

std::optional<BaselineOutcome> parseBaselineOutcome(std::string_view text) {
    for (const auto value : {BaselineOutcome::kPass, BaselineOutcome::kFail,
                             BaselineOutcome::kInvalid, BaselineOutcome::kIncomparable}) {
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

namespace {

/// v2 基线里的指标名换成现在的名字。
std::string upgradeMetric(const std::string& name) {
    std::string out = name;
    if (const auto at = out.find(".rotation_deg."); at != std::string::npos) {
        out.replace(at, std::string_view(".rotation_deg.").size(), ".rotation.");
    }
    if (out.ends_with(".pairs")) out.replace(out.size() - 6, 6, ".samples");
    if (out == "ate.max_reference_gap_s") out = "ate.max_reference_gap";
    return out;
}

/// v2 的平铺协议换成按评测函数的协议：对齐给 ate，关联的三项给 ate 与 rpe，RPE 的间隔给
/// rpe。数值写成与评测函数参数相同的最短往返形式，没写的取 v2 的默认值；v2 比的是实际用的
/// 对齐方式，现在比请求的，所以先取 alignment_requested。
std::map<std::string, Protocol> upgradeProtocol(const YAML::Node& old,
                                                std::vector<std::string>& upgraded) {
    const auto text = [&](const char* key, const char* fallback) {
        const YAML::Node node = old[key];
        const std::string value = node ? node.as<std::string>() : std::string();
        return value.empty() ? std::string(fallback) : value;
    };
    const auto number = [&](const char* key, double fallback) {
        const YAML::Node node = old[key];
        return std::format("{}", node ? node.as<double>() : fallback);
    };
    Protocol shared{{"association", text("association", "nearest")},
                    {"tolerance_s", number("association_tolerance_s", 0.02)},
                    {"max_interpolation_gap_s", number("max_interpolation_gap_s", 0.15)},
                    {"estimate", "odometry"}};
    std::map<std::string, Protocol> out;
    Protocol ate = shared;
    ate["type"] = "ate";
    ate["alignment"] = text("alignment_requested", text("alignment", "se3").c_str());
    out["ate"] = std::move(ate);
    if (const std::string kind = text("rpe_delta_kind", ""); !kind.empty()) {
        const double delta = old["rpe_delta"] ? old["rpe_delta"].as<double>() : 0.0;
        Protocol rpe = shared;
        rpe["type"] = "rpe";
        rpe["delta_m"] = std::format("{}", kind == "distance" ? delta : 100.0);
        rpe["delta_s"] = std::format("{}", kind == "time" ? delta : 0.0);
        out["rpe"] = std::move(rpe);
    }
    upgraded.push_back(std::string("protocol 换成按评测函数的 evaluators（ate")
                       + (out.count("rpe") != 0 ? "、rpe" : "") + "）");
    const auto nonzero = [&](const char* key) { return old[key] && old[key].as<double>() != 0.0; };
    if (nonzero("window_begin_s") || nonzero("window_end_s")) {
        upgraded.emplace_back("时间窗不再核对：目标窗口现在属于有效输入");
    }
    return out;
}

}  // namespace

std::vector<DatasetBaseline> loadBaselines(const std::string& path,
                                           std::vector<std::string>* notes) {
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
        // 未知键一律拒绝：拼错的 "min_sample" 静静地被忽略，门限就松了
        for (const auto& item : entry) {
            static const std::set<std::string> known{
                "dataset", "config_hash", "groundtruth_digest", "recorded_at", "note",
                "min_samples", "bounds", "evaluators", "prefix",
                "min_pairs", "protocol"};  // 后两个是 v2 的写法，下面换算
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
        if (entry["config_hash"]) baseline.config_hash = entry["config_hash"].as<std::string>();
        if (entry["groundtruth_digest"]) {
            baseline.groundtruth_digest = entry["groundtruth_digest"].as<std::string>();
        }
        if (entry["recorded_at"]) baseline.recorded_at = entry["recorded_at"].as<std::string>();
        if (entry["note"]) baseline.note = entry["note"].as<std::string>();
        if (entry["min_samples"]) baseline.min_samples = entry["min_samples"].as<double>();
        std::vector<std::string> upgraded;  // 这一条基线换算过的 v2 写法
        if (entry["min_pairs"]) {
            if (entry["min_samples"]) {
                throw BaselineError("基线 " + baseline.dataset
                                    + " 新旧写法混用：写了 min_samples，又写了 min_pairs: " + path);
            }
            baseline.min_samples = entry["min_pairs"].as<double>();
            upgraded.emplace_back("min_pairs 改叫 min_samples");
        }
        if (const YAML::Node protocol = entry["protocol"]) {
            if (entry["evaluators"]) {
                throw BaselineError("基线 " + baseline.dataset
                                    + " 新旧写法混用：写了 evaluators，又写了 protocol: " + path);
            }
            if (!protocol.IsMap()) {
                throw BaselineError("基线 " + baseline.dataset + " 的 protocol 必须是映射: "
                                    + path);
            }
            baseline.evaluators = upgradeProtocol(protocol, upgraded);
        }
        if (const YAML::Node evaluators = entry["evaluators"]) {
            if (!evaluators.IsMap()) {
                throw BaselineError("基线 " + baseline.dataset + " 的 evaluators 必须是映射: "
                                    + path);
            }
            for (const auto& item : evaluators) {
                if (!item.second.IsMap()) {
                    throw BaselineError("基线里评测函数 " + item.first.as<std::string>()
                                        + " 的协议必须是映射: " + path);
                }
                Protocol protocol;
                for (const auto& field : item.second) {
                    protocol[field.first.as<std::string>()] = field.second.as<std::string>();
                }
                baseline.evaluators[item.first.as<std::string>()] = std::move(protocol);
            }
        }

        const YAML::Node bounds = entry["bounds"];
        if (!bounds || !bounds.IsMap()) {
            throw BaselineError("基线 " + baseline.dataset + " 缺少 bounds 映射: " + path);
        }
        for (const auto& item : bounds) {
            MetricBound bound;
            const auto written = item.first.as<std::string>();
            bound.metric = upgradeMetric(written);
            if (bound.metric != written) upgraded.push_back(written + " 改叫 " + bound.metric);
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
        if (!upgraded.empty() && notes != nullptr) {
            std::string note = "基线 " + baseline.dataset + " 用了 v2 的写法，已按新写法理解：";
            for (std::size_t i = 0; i < upgraded.size(); ++i) {
                note += (i == 0 ? "" : "；") + upgraded[i];
            }
            if (!baseline.config_hash.empty()) {
                note += "。它的 config_hash 按 v2 的口径算，与现在的有效身份对不上，"
                        "门禁会判不可比较：改成任务记录 identity.yaml 里的 effective_hash";
            }
            notes->push_back(std::move(note));
        }
        baselines.push_back(std::move(baseline));
    }
    return baselines;
}

void writeBaselines(const std::string& path, const std::vector<DatasetBaseline>& baselines) {
    YAML::Emitter out;
    out.SetDoublePrecision(17);
    out << YAML::BeginMap << YAML::Key << "baselines" << YAML::Value << YAML::BeginSeq;
    for (const auto& baseline : baselines) {
        out << YAML::BeginMap;
        out << YAML::Key << "dataset" << YAML::Value << baseline.dataset;
        if (!baseline.config_hash.empty()) {
            out << YAML::Key << "config_hash" << YAML::Value << baseline.config_hash;
        }
        if (!baseline.groundtruth_digest.empty()) {
            out << YAML::Key << "groundtruth_digest" << YAML::Value
                << baseline.groundtruth_digest;
        }
        if (!baseline.recorded_at.empty()) {
            out << YAML::Key << "recorded_at" << YAML::Value << baseline.recorded_at;
        }
        if (!baseline.note.empty()) out << YAML::Key << "note" << YAML::Value << baseline.note;
        out << YAML::Key << "min_samples" << YAML::Value << baseline.min_samples;
        if (baseline.prefix) out << YAML::Key << "prefix" << YAML::Value << true;
        if (!baseline.evaluators.empty()) {
            out << YAML::Key << "evaluators" << YAML::Value << YAML::BeginMap;
            for (const auto& [name, protocol] : baseline.evaluators) {
                out << YAML::Key << name << YAML::Value << YAML::BeginMap;
                for (const auto& [key, value] : protocol) {
                    out << YAML::Key << key << YAML::Value << YAML::DoubleQuoted << value;
                }
                out << YAML::EndMap;
            }
            out << YAML::EndMap;
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

}  // namespace simpleslam::eval
