// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/framework/digest.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

namespace simpleslam::config {

namespace {

std::string formatDouble(double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
    return buffer;
}

std::string quoteJson(const std::string& text) {
    std::string out = "\"";
    for (const char c : text) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\t': out += "\\t"; break;
            case '\r': out += "\\r"; break;
            default: out += c; break;
        }
    }
    out += '"';
    return out;
}

/// 规范化导出时字符串一律加引号并转义。
///
/// 不加引号是不可逆的：`name: "a # b"` 导出成 `name: a # b`，再加载只剩 `a`
/// 。规范化导出是配置哈希的输入，也是"把生效配置落盘"
/// 的载体，它必须满足 load(dump(c)) == c。一律加引号比"看着需要才加"更可靠，
/// 因为"需要"的判据会随 YAML 版本变。
std::string quoteYaml(const std::string& text) {
    std::string out = "\"";
    for (const char c : text) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += c; break;
        }
    }
    out += '"';
    return out;
}

std::vector<std::string> splitPath(const std::string& path) {
    std::vector<std::string> parts;
    std::string current;
    for (const char c : path) {
        if (c == '.') {
            parts.push_back(current);
            current.clear();
        } else {
            current += c;
        }
    }
    parts.push_back(current);
    return parts;
}

/// 按点分路径取 YAML 节点；任一层缺失返回空节点。
YAML::Node nodeAt(const YAML::Node& root, const std::string& path) {
    YAML::Node current = YAML::Clone(root);
    for (const auto& part : splitPath(path)) {
        if (!current || !current.IsMap()) return YAML::Node(YAML::NodeType::Undefined);
        current = current[part];
        if (!current) return YAML::Node(YAML::NodeType::Undefined);
    }
    return current;
}

/// 收集 YAML 里出现的全部叶子路径，用来检出未知键。
void collectLeaves(const YAML::Node& node, const std::string& prefix,
                   std::vector<std::string>& out) {
    // 空映射也算一个叶子：否则 `misspelled: {}` 里的拼写错误不会被发现
    if (node.IsMap() && node.size() > 0) {
        for (const auto& item : node) {
            const std::string key = item.first.as<std::string>();
            collectLeaves(item.second, prefix.empty() ? key : prefix + "." + key, out);
        }
        return;
    }
    if (!prefix.empty()) out.push_back(prefix);
}

// ── 逐类型的读取：不做隐式转换 ──

/// 带引号的标量是字符串，不是数字或布尔。yaml-cpp 用非特定标签 "!" 标记它，
/// 所以 `"4"` 与 `4` 在这里是两回事——否则配置里的引号会悄悄失去意义。
bool isQuoted(const YAML::Node& node) { return node.Tag() == "!"; }

void requireUnquoted(const YAML::Node& node, const std::string& path, const std::string& origin,
                     const char* expected) {
    if (isQuoted(node)) {
        throw ConfigError(origin + ": " + path + " 需要" + expected + "，得到带引号的字符串");
    }
}

void assign(bool& target, const YAML::Node& node, const std::string& path,
            const std::string& origin) {
    if (!node.IsScalar()) throw ConfigError(origin + ": " + path + " 需要 true/false");
    requireUnquoted(node, path, origin, "true/false");
    const std::string text = node.Scalar();
    if (text == "true" || text == "True" || text == "yes" || text == "on") {
        target = true;
    } else if (text == "false" || text == "False" || text == "no" || text == "off") {
        target = false;
    } else {
        throw ConfigError(origin + ": " + path + " 需要 true/false，得到 " + text);
    }
}

void assign(std::int64_t& target, const YAML::Node& node, const std::string& path,
            const std::string& origin) {
    if (!node.IsScalar()) throw ConfigError(origin + ": " + path + " 需要整数");
    requireUnquoted(node, path, origin, "整数");
    const std::string text = node.Scalar();
    try {
        std::size_t consumed = 0;
        const long long value = std::stoll(text, &consumed);
        if (consumed != text.size()) throw std::invalid_argument("trailing");
        target = static_cast<std::int64_t>(value);
    } catch (const std::exception&) {
        throw ConfigError(origin + ": " + path + " 需要整数，得到 " + text);
    }
}

void assign(double& target, const YAML::Node& node, const std::string& path,
            const std::string& origin) {
    if (!node.IsScalar()) throw ConfigError(origin + ": " + path + " 需要实数");
    requireUnquoted(node, path, origin, "实数");
    const std::string text = node.Scalar();
    try {
        std::size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (consumed != text.size()) throw std::invalid_argument("trailing");
        // nan / inf 能被 strtod 接受，但它们不是有效的配置值
        if (!std::isfinite(value)) throw std::invalid_argument("not finite");
        target = value;
    } catch (const std::exception&) {
        throw ConfigError(origin + ": " + path + " 需要有限实数，得到 " + text);
    }
}

void assign(std::string& target, const YAML::Node& node, const std::string& path,
            const std::string& origin) {
    if (!node.IsScalar()) throw ConfigError(origin + ": " + path + " 需要字符串");
    target = node.Scalar();
}

const char* typeName(const bool&) { return "bool"; }
const char* typeName(const std::int64_t&) { return "int"; }
const char* typeName(const double&) { return "double"; }
const char* typeName(const std::string&) { return "string"; }

std::string defaultText(const bool& v) { return v ? "true" : "false"; }
std::string defaultText(const std::int64_t& v) { return std::to_string(v); }
std::string defaultText(const double& v) { return formatDouble(v); }
std::string defaultText(const std::string& v) { return v; }

/// 规范化导出用的文本：数值同上，字符串加引号。
std::string canonicalText(const bool& v) { return defaultText(v); }
std::string canonicalText(const std::int64_t& v) { return defaultText(v); }
std::string canonicalText(const double& v) { return defaultText(v); }
std::string canonicalText(const std::string& v) { return quoteYaml(v); }

void requireOneOf(const std::string& path, const std::string& value,
                  std::initializer_list<const char*> allowed) {
    for (const char* candidate : allowed) {
        if (value == candidate) return;
    }
    std::string list;
    for (const char* candidate : allowed) {
        list += (list.empty() ? "" : " / ");
        list += candidate;
    }
    throw ConfigError(path + " 只能是 " + list + "，得到 " + value);
}

void requireAtLeast(const std::string& path, std::int64_t value, std::int64_t minimum) {
    if (value < minimum) {
        throw ConfigError(path + " 不能小于 " + std::to_string(minimum) + "，得到 "
                          + std::to_string(value));
    }
}

void requireNonNegative(const std::string& path, double value) {
    if (!(value >= 0.0)) throw ConfigError(path + " 不能为负，得到 " + formatDouble(value));
}

/// 配置里的时长上限。纳秒整型能表示的范围远大于它；超过它的时长在配置里只可能是
/// 写错了单位或多写了几个零，放进去会在换算成纳秒时溢出。
constexpr double kMaxDurationSeconds = 1e6;

void requireDuration(const std::string& path, double seconds) {
    requireNonNegative(path, seconds);
    if (seconds > kMaxDurationSeconds) {
        throw ConfigError(path + " 超出时长上限 " + formatDouble(kMaxDurationSeconds) + " 秒，得到 "
                          + formatDouble(seconds));
    }
}

}  // namespace

std::vector<FieldSchema> schema() {
    SlamConfig defaults;
    std::vector<FieldSchema> fields;
    visitConfig(defaults, [&](const char* path, const auto& value, const char* doc,
                              Tunable tunable) {
        fields.push_back(FieldSchema{path, typeName(value), defaultText(value), doc,
                                     tunable == Tunable::kYes});
    });
    return fields;
}

std::string schemaJson() {
    std::ostringstream out;
    out << "{\n  \"fields\": [\n";
    const auto fields = schema();
    for (std::size_t i = 0; i < fields.size(); ++i) {
        const auto& field = fields[i];
        out << "    {\"path\": " << quoteJson(field.path) << ", \"type\": "
            << quoteJson(field.type) << ", \"default\": " << quoteJson(field.default_value)
            << ", \"doc\": " << quoteJson(field.doc) << ", \"tunable\": "
            << (field.tunable ? "true" : "false") << "}";
        if (i + 1 < fields.size()) out << ',';
        out << '\n';
    }
    out << "  ]\n}\n";
    return out.str();
}

namespace {

/// 解析文本：未知键与类型不符在这里报错；值有没有意义留给整体校验。
SlamConfig parseConfig(const std::string& text, const std::string& origin) {
    YAML::Node root;
    try {
        root = YAML::Load(text);
    } catch (const YAML::Exception& e) {
        throw ConfigError(origin + ": YAML 解析失败: " + e.what());
    }
    if (!root || root.IsNull()) return SlamConfig{};
    if (!root.IsMap()) throw ConfigError(origin + ": 配置的顶层必须是映射");

    // 未知键先查：拼错的键静默忽略，比报错难查得多
    std::set<std::string> known;
    for (const auto& field : schema()) known.insert(field.path);
    std::vector<std::string> present;
    collectLeaves(root, "", present);
    for (const auto& path : present) {
        if (known.count(path) == 0) throw ConfigError(origin + ": 未知配置项 " + path);
    }

    SlamConfig config;
    visitConfig(config, [&](const char* path, auto& value, const char*, Tunable) {
        const YAML::Node node = nodeAt(root, path);
        if (!node || node.IsNull()) return;  // 缺省即默认值
        assign(value, node, path, origin);
    });
    return config;
}

/// 应用一条"路径=值"覆盖，按 schema 的类型解析。不单独校验：见 applyOverrides()。
void applyAssignment(SlamConfig& config, const std::string& assignment) {
    const auto equals = assignment.find('=');
    if (equals == std::string::npos || equals == 0) {
        throw ConfigError("覆盖项要写成 路径=值，得到 " + assignment);
    }
    const std::string path = assignment.substr(0, equals);
    const std::string text = assignment.substr(equals + 1);

    bool found = false;
    visitConfig(config, [&](const char* field_path, auto& value, const char*, Tunable) {
        if (found || path != field_path) return;
        found = true;
        YAML::Node node = YAML::Load(text.empty() ? std::string("\"\"") : text);
        if (!node.IsScalar()) node = YAML::Node(text);
        assign(value, node, path, "命令行覆盖");
    });
    if (!found) throw ConfigError("未知配置项 " + path);
}

}  // namespace

SlamConfig loadConfigString(const std::string& text, const std::string& origin,
                            const std::vector<std::string>& overrides) {
    auto config = parseConfig(text, origin);
    applyOverrides(config, overrides);  // 类型对了不等于值有意义：覆盖全部到位之后整体校验
    return config;
}

void validateConfig(const SlamConfig& config) {
    // 有限性：类型检查拦得住 "abc"，拦不住 1e400 这种溢出成 inf 的写法
    visitConfig(config, [](const char* path, const auto& value, const char*, Tunable) {
        if constexpr (std::is_same_v<std::decay_t<decltype(value)>, double>) {
            if (!std::isfinite(value)) {
                throw ConfigError(std::string(path) + " 必须是有限实数");
            }
        }
    });

    requireAtLeast("runtime.spin_budget", config.runtime.spin_budget, 1);
    requireAtLeast("packetizer.max_imu_backlog", config.packetizer.max_imu_backlog, 1);
    requireAtLeast("odometry.max_iterations", config.odometry.max_iterations, 1);
    requireAtLeast("backend.max_iterations", config.backend.max_iterations, 1);
    requireAtLeast("dataset.skip_scans", config.dataset.skip_scans, 0);
    requireAtLeast("dataset.max_scans", config.dataset.max_scans, 0);

    if (config.recording.enabled && config.recording.output.empty()) {
        // 打开了录制却没说录到哪：不能悄悄不录
        throw ConfigError("recording.enabled 为真时必须给出 recording.output");
    }
    requireDuration("packetizer.post_support_s", config.packetizer.post_support_s);
    requireDuration("packetizer.pre_support_s", config.packetizer.pre_support_s);
    requireDuration("packetizer.attachment_tolerance_s",
                       config.packetizer.attachment_tolerance_s);
    requireDuration("packetizer.max_imu_gap_s", config.packetizer.max_imu_gap_s);
    requireDuration("evaluation.max_interpolation_gap_s",
                       config.evaluation.max_interpolation_gap_s);
    requireNonNegative("odometry.keyframe_distance_m", config.odometry.keyframe_distance_m);
    requireNonNegative("odometry.keyframe_rotation_deg", config.odometry.keyframe_rotation_deg);
    requireDuration("odometry.keyframe_interval_s", config.odometry.keyframe_interval_s);
    requireNonNegative("odometry.voxel_size_m", config.odometry.voxel_size_m);
    requireDuration("evaluation.association_tolerance_s",
                       config.evaluation.association_tolerance_s);
    requireNonNegative("evaluation.rpe_delta_m", config.evaluation.rpe_delta_m);
    requireDuration("evaluation.rpe_delta_s", config.evaluation.rpe_delta_s);

    if (!(config.backend.odometry_translation_weight > 0.0)) {
        throw ConfigError("backend.odometry_translation_weight 必须为正");
    }
    if (!(config.backend.odometry_rotation_weight > 0.0)) {
        throw ConfigError("backend.odometry_rotation_weight 必须为正");
    }

    requireOneOf("dataset.kind", config.dataset.kind,
                 {"synthetic", "kitti_odometry", "kitti_raw", "replay"});
    requireOneOf("dataset.groundtruth_frame", config.dataset.groundtruth_frame,
                 {"body", "camera"});
    requireOneOf("packetizer.source_policy", config.packetizer.source_policy,
                 {"source", "config"});
    requireOneOf("evaluation.format", config.evaluation.format, {"tum", "kitti"});
    requireOneOf("evaluation.association", config.evaluation.association,
                 {"nearest", "interpolate"});
    requireOneOf("evaluation.alignment", config.evaluation.alignment,
                 {"none", "first_pose", "se3", "sim3"});
}

SlamConfig loadConfigFile(const std::string& path, const std::vector<std::string>& overrides) {
    std::ifstream file(path);
    if (!file) throw ConfigError("打不开配置文件: " + path);
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return loadConfigString(buffer.str(), path, overrides);
}

std::string dumpConfig(const SlamConfig& config) {
    // 规范化：按 schema 顺序、按节分组、数值格式固定。哈希基于它，所以不能随手改。
    std::ostringstream out;
    std::string current_section;
    visitConfig(config, [&](const char* path, const auto& value, const char*, Tunable) {
        const auto parts = splitPath(path);
        if (parts.size() == 1) {
            if (!current_section.empty()) {
                out << '\n';
                current_section.clear();
            }
            out << parts[0] << ": " << canonicalText(value) << '\n';
            return;
        }
        if (parts[0] != current_section) {
            out << '\n' << parts[0] << ":\n";
            current_section = parts[0];
        }
        out << "  " << parts[1] << ": " << canonicalText(value) << '\n';
    });
    return out.str();
}

void writeConfigFile(const std::string& path, const SlamConfig& config) {
    std::ofstream file(path);
    if (!file) throw ConfigError("打不开配置输出文件: " + path);
    file << dumpConfig(config);
    if (!file) throw ConfigError("写配置文件失败: " + path);
}

std::string digestHex(const std::string& text) { return fw::digestHex(text); }

std::string configHash(const SlamConfig& config) { return digestHex(dumpConfig(config)); }

std::string effectiveHash(const std::map<std::string, std::string>& effective) {
    // 长度前缀编码：键或值里带 '=' 或换行（例如来源名里的路径）也不会让两份不同的
    // 输入拼成同一个串
    std::string canonical;
    for (const auto& [key, value] : effective) {
        canonical += std::to_string(key.size()) + ':' + key + std::to_string(value.size()) + ':'
                     + value + '\n';
    }
    return digestHex(canonical);
}

void applyOverrides(SlamConfig& config, const std::vector<std::string>& assignments) {
    for (const auto& assignment : assignments) applyAssignment(config, assignment);
    validateConfig(config);
}

std::string configParamName(const std::string& path) {
    std::string name = "/config/";
    for (const char c : path) name += (c == '.' ? '/' : c);
    return name;
}

void installConfigValidator(comm::ParameterStore& store, const SlamConfig& base) {
    store.setValidator([base](const comm::ParamSnapshot& candidate, std::string& detail) {
        SlamConfig probe = base;
        readFromParameters(probe, candidate);
        try {
            validateConfig(probe);
        } catch (const ConfigError& e) {
            detail = e.what();
            return false;
        }
        return true;
    });
}

void applyToParameters(const SlamConfig& config, comm::ParameterStore& store) {
    visitConfig(config, [&](const char* path, const auto& value, const char*, Tunable tunable) {
        if (tunable != Tunable::kYes) return;  // 不可调的字段不进运行期参数：改了也不会生效
        store.preset(configParamName(path), comm::ParamValue{value});
    });
    installConfigValidator(store, config);
}

void readFromParameters(SlamConfig& config, const comm::ParamSnapshot& snapshot) {
    visitConfig(config, [&](const char* path, auto& value, const char*, Tunable tunable) {
        if (tunable != Tunable::kYes) return;
        using ValueType = std::decay_t<decltype(value)>;
        if (auto stored = snapshot.get<ValueType>(configParamName(path))) value = *stored;
    });
}

}  // namespace simpleslam::config
