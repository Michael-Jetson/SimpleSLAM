// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/config/loader.hpp>


#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <system_error>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

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
/// 。规范化导出是"把生效配置落盘"的载体（任务记录里的 `task.yaml`），
/// 它必须满足 load(dump(c)) == c。一律加引号比"看着需要才加"更可靠，
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


void assign(std::vector<std::string>& target, const YAML::Node& node, const std::string& path,
            const std::string& origin) {
    if (!node.IsSequence()) throw ConfigError(origin + ": " + path + " 需要列表");
    target.clear();
    for (const auto& item : node) {
        if (!item.IsScalar()) throw ConfigError(origin + ": " + path + " 的元素要是字符串");
        target.push_back(item.Scalar());
    }
}

std::string joinPath(const std::string& prefix, const std::string& key) {
    return prefix.empty() ? key : prefix + "." + key;
}

/// 插件参数的标量：带引号的是字符串；true/false 是布尔；能完整读成整数的是整数，
/// 能完整读成实数的是实数；其余是字符串。字段到底要什么类型由目录按实现的字段清单查，
/// 这里只如实记下写法——`"00"` 与 `00` 因此是两回事。
PluginValue scalarValue(const YAML::Node& node) {
    const std::string text = node.Scalar();
    if (isQuoted(node) || text.empty()) return text;
    if (text == "true") return true;
    if (text == "false") return false;
    const char* first = text.data() + (text.front() == '+' ? 1 : 0);
    const char* last = text.data() + text.size();
    std::int64_t integer = 0;
    if (const auto [end, error] = std::from_chars(first, last, integer);
        error == std::errc() && end == last) {
        return integer;
    }
    double real = 0.0;
    if (const auto [end, error] = std::from_chars(first, last, real);
        error == std::errc() && end == last) {
        return real;
    }
    return text;
}

PluginSpec parsePlugin(const YAML::Node& node, const std::string& path,
                       const std::string& origin) {
    if (!node.IsMap()) {
        throw ConfigError(origin + ": " + path + " 是插件节点，要写成映射，至少写出 type");
    }
    PluginSpec spec;
    for (const auto& item : node) {
        const std::string key = item.first.as<std::string>();
        const YAML::Node& value = item.second;
        const std::string sub = joinPath(path, key);
        if (key == "type") {
            if (!value.IsScalar()) throw ConfigError(origin + ": " + sub + " 要写成实现的类型名");
            spec.type = value.Scalar();
        } else if (value.IsMap()) {
            spec.children.emplace(key, parsePlugin(value, sub, origin));
        } else if (value.IsScalar()) {
            spec.params.emplace(key, scalarValue(value));
        } else if (!value.IsNull()) {
            throw ConfigError(origin + ": " + sub + " 插件参数不支持列表");
        }
    }
    return spec;
}

void assign(PluginSpec& target, const YAML::Node& node, const std::string& path,
            const std::string& origin) {
    target = parsePlugin(node, path, origin);
}

/// 一组带名字的插件节点：映射里每一项是一个插件节点；空映射表示一个都没有。
void assign(PluginMap& target, const YAML::Node& node, const std::string& path,
            const std::string& origin) {
    if (node.IsNull()) {
        target.clear();
        return;
    }
    if (!node.IsMap()) throw ConfigError(origin + ": " + path + " 要写成映射：名字 → 插件节点");
    target.clear();
    for (const auto& item : node) {
        const std::string name = item.first.as<std::string>();
        target.emplace(name, parsePlugin(item.second, joinPath(path, name), origin));
    }
}

const char* typeName(const bool&) { return "bool"; }
const char* typeName(const std::int64_t&) { return "int"; }
const char* typeName(const double&) { return "double"; }
const char* typeName(const std::string&) { return "string"; }
const char* typeName(const std::vector<std::string>&) { return "list"; }
const char* typeName(const PluginSpec&) { return "slot"; }
const char* typeName(const PluginMap&) { return "slots"; }

std::string listText(const std::vector<std::string>& items) {
    std::string out = "[";
    for (std::size_t i = 0; i < items.size(); ++i) {
        out += (i == 0 ? "" : ", ") + quoteYaml(items[i]);
    }
    return out + "]";
}

std::string defaultText(const bool& v) { return v ? "true" : "false"; }
std::string defaultText(const std::int64_t& v) { return std::to_string(v); }
std::string defaultText(const double& v) { return formatDouble(v); }
std::string defaultText(const std::string& v) { return v; }
std::string defaultText(const std::vector<std::string>& v) { return listText(v); }
std::string defaultText(const PluginSpec& v) { return v.type; }
std::string defaultText(const PluginMap& v) {
    std::string out = "{";
    for (const auto& [name, spec] : v) {
        out += (out.size() == 1 ? "" : ", ") + name + ": " + spec.type;
    }
    return out + "}";
}

/// 规范化导出用的文本：数值同上，字符串加引号。
std::string canonicalText(const bool& v) { return defaultText(v); }
std::string canonicalText(const std::int64_t& v) { return defaultText(v); }
std::string canonicalText(const double& v) { return defaultText(v); }
std::string canonicalText(const std::string& v) { return quoteYaml(v); }
std::string canonicalText(const std::vector<std::string>& v) { return listText(v); }
std::string canonicalText(const PluginValue& v) {
    return std::visit([](const auto& value) { return canonicalText(value); }, v);
}

/// 插件节点：type 在前，参数与子节点各按键名排序。还原后的节点补齐了默认值，
/// 写没写默认值因此不影响导出。
void emitPlugin(std::ostream& out, const PluginSpec& spec, std::size_t indent) {
    const std::string pad(indent, ' ');
    out << pad << "type: " << quoteYaml(spec.type) << '\n';
    for (const auto& [key, value] : spec.params) {
        out << pad << key << ": " << canonicalText(value) << '\n';
    }
    for (const auto& [key, child] : spec.children) {
        out << pad << key << ":\n";
        emitPlugin(out, child, indent + 2);
    }
}

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
        throw ConfigError(path + " 超出时长上限 " + formatDouble(kMaxDurationSeconds)
                          + " 秒，得到 " + formatDouble(seconds));
    }
}

template <class T>
inline constexpr bool kOpaque = std::is_same_v<T, PluginSpec> || std::is_same_v<T, PluginMap>
                                || std::is_same_v<T, std::vector<std::string>>;

/// 字段路径，以及其中不往下展开的那些（插件节点、列表）。
struct FieldPaths {
    std::set<std::string> all;
    std::set<std::string> opaque;
};

FieldPaths fieldPaths() {
    TaskConfig defaults;
    FieldPaths paths;
    visitConfig(defaults, [&](const char* path, const auto& value, const char*, Tunable) {
        paths.all.insert(path);
        if constexpr (kOpaque<std::decay_t<decltype(value)>>) paths.opaque.insert(path);
    });
    return paths;
}

/// 收集 YAML 里出现的全部叶子路径，用来检出未知键。插件节点与列表是一个叶子：
/// 插件节点内部的键归目录查。空映射也算一个叶子，否则 `misspelled: {}` 查不出来。
void collectLeaves(const YAML::Node& node, const std::string& prefix, const FieldPaths& paths,
                   std::vector<std::string>& out) {
    if (node.IsMap() && node.size() > 0 && paths.opaque.count(prefix) == 0) {
        for (const auto& item : node) {
            collectLeaves(item.second, joinPath(prefix, item.first.as<std::string>()), paths, out);
        }
        return;
    }
    if (!prefix.empty()) out.push_back(prefix);
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

YAML::Node parseYaml(const std::string& text, const std::string& origin) {
    try {
        return YAML::Load(text);
    } catch (const YAML::Exception& e) {
        throw ConfigError(origin + ": YAML 解析失败: " + e.what());
    }
}

std::string readFile(const std::string& path, const char* what) {
    std::ifstream file(path);
    if (!file) throw ConfigError(std::string("打不开") + what + ": " + path);
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

/// 覆盖项的值按 YAML 读：`0.8`、`true`、`[a, b]`、`{type: x}` 各是各的类型。
YAML::Node parseOverrideValue(const std::string& text) {
    try {
        YAML::Node node = YAML::Load(text.empty() ? std::string("\"\"") : text);
        if (!node || node.IsNull()) return YAML::Node(text);
        return node;
    } catch (const YAML::Exception&) {
        return YAML::Node(text);
    }
}

/// 覆盖插件节点内部：`<槽>.type=X` 换实现（参数回到新实现的默认值），
/// `<槽>.<参数>=v` 改参数，`<槽>.<子槽>.…` 往下走。
void applyToPlugin(PluginSpec& spec, const std::vector<std::string>& rest, const YAML::Node& value,
                   const std::string& path) {
    const std::string& key = rest.front();
    if (rest.size() > 1) {
        spec.params.erase(key);
        applyToPlugin(spec.children[key], {rest.begin() + 1, rest.end()}, value, path);
        return;
    }
    if (key == "type") {
        if (!value.IsScalar()) throw ConfigError(path + " 要写成实现的类型名");
        if (spec.type != value.Scalar()) {
            spec = PluginSpec{.type = value.Scalar(), .params = {}, .children = {}};
        }
        return;
    }
    if (value.IsMap()) {
        spec.params.erase(key);
        spec.children[key] = parsePlugin(value, path, "覆盖项");
        return;
    }
    if (!value.IsScalar()) throw ConfigError(path + " 插件参数不支持列表");
    spec.children.erase(key);
    spec.params[key] = scalarValue(value);
}

void applyAssignment(TaskConfig& config, const std::string& assignment) {
    const auto equals = assignment.find('=');
    if (equals == std::string::npos || equals == 0) {
        throw ConfigError("覆盖项要写成 路径=值，得到 " + assignment);
    }
    const std::string path = assignment.substr(0, equals);
    const std::string text = assignment.substr(equals + 1);
    const YAML::Node value = parseOverrideValue(text);

    bool found = false;
    visitConfig(config, [&](const char* field_path, auto& field, const char*, Tunable) {
        if (found) return;
        using T = std::decay_t<decltype(field)>;
        const std::string name = field_path;
        if (path == name) {
            found = true;
            if constexpr (std::is_same_v<T, PluginSpec>) {
                if (!value.IsMap()) {
                    throw ConfigError("覆盖项 " + path + " 是插件节点：写 " + path + ".type=… 或 "
                                      + path + "={type: …}");
                }
                field = parsePlugin(value, path, "覆盖项");
            } else if constexpr (std::is_same_v<T, PluginMap>) {
                if (!value.IsMap()) {
                    throw ConfigError("覆盖项 " + path + " 是一组插件节点：写 " + path
                                      + "={名字: {type: …}}，或 " + path + ".<名字>.type=…");
                }
                assign(field, value, path, "覆盖项");
            } else if constexpr (std::is_same_v<T, std::string>) {
                field = value.IsScalar() ? value.Scalar() : text;
            } else {
                assign(field, value, path, "覆盖项");
            }
            return;
        }
        const bool below = path.size() > name.size() && path.compare(0, name.size(), name) == 0
                           && path[name.size()] == '.';
        if constexpr (std::is_same_v<T, PluginSpec>) {
            if (below) {
                found = true;
                applyToPlugin(field, splitPath(path.substr(name.size() + 1)), value, path);
            }
        } else if constexpr (std::is_same_v<T, PluginMap>) {
            if (below) {
                // `<字段>.<名字>=…` 写整个条目，`<字段>.<名字>.…` 往条目里写；没有的条目新建
                found = true;
                auto rest = splitPath(path.substr(name.size() + 1));
                const std::string entry = rest.front();
                rest.erase(rest.begin());
                if (rest.empty()) {
                    if (!value.IsMap()) throw ConfigError("覆盖项 " + path + " 要写成 {type: …}");
                    field[entry] = parsePlugin(value, path, "覆盖项");
                } else {
                    applyToPlugin(field[entry], rest, value, path);
                }
            }
        }
    });
    if (!found) throw ConfigError("未知配置项 " + path);
}

/// 规范化导出的主体：按字段清单的顺序嵌套输出。
std::string dump(const TaskConfig& config) {
    std::ostringstream out;
    std::vector<std::string> open;
    bool first_section = true;
    visitConfig(config, [&](const char* path, const auto& value, const char*, Tunable) {
        auto parts = splitPath(path);
        const std::string key = parts.back();
        parts.pop_back();
        std::size_t common = 0;
        while (common < open.size() && common < parts.size() && open[common] == parts[common]) {
            ++common;
        }
        open.resize(common);
        for (std::size_t i = common; i < parts.size(); ++i) {
            if (i == 0 && !first_section) out << '\n';
            first_section = false;
            out << std::string(2 * i, ' ') << parts[i] << ":\n";
            open.push_back(parts[i]);
        }
        const std::string pad(2 * parts.size(), ' ');
        if constexpr (std::is_same_v<std::decay_t<decltype(value)>, PluginSpec>) {
            out << pad << key << ":\n";
            emitPlugin(out, value, pad.size() + 2);
        } else if constexpr (std::is_same_v<std::decay_t<decltype(value)>, PluginMap>) {
            // 一组插件节点按名字排序；一个都没有写成 {}
            if (value.empty()) {
                out << pad << key << ": {}\n";
            } else {
                out << pad << key << ":\n";
                for (const auto& [name, spec] : value) {
                    out << pad << "  " << name << ":\n";
                    emitPlugin(out, spec, pad.size() + 4);
                }
            }
        } else {
            out << pad << key << ": " << canonicalText(value) << '\n';
        }
    });
    return out.str();
}

/// 插件节点的 JSON：类型、参数（按类型写成 JSON 值）、子节点。给 schema 里子槽的默认节点用。
std::string pluginJson(const PluginSpec& spec) {
    std::string out = "{\"type\": " + quoteJson(spec.type) + ", \"params\": {";
    std::size_t n = 0;
    for (const auto& [key, value] : spec.params) {
        out += (n++ == 0 ? "" : ", ") + quoteJson(key) + ": ";
        out += std::visit(
            [](const auto& v) -> std::string {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, std::string>) {
                    return quoteJson(v);
                } else {
                    return defaultText(v);
                }
            },
            value);
    }
    out += "}, \"children\": {";
    n = 0;
    for (const auto& [key, child] : spec.children) {
        out += (n++ == 0 ? "" : ", ") + quoteJson(key) + ": " + pluginJson(child);
    }
    return out + "}}";
}

}  // namespace

std::vector<FieldSchema> schema() {
    TaskConfig defaults;
    std::vector<FieldSchema> fields;
    visitConfig(defaults, [&](const char* path, const auto& value, const char* doc,
                              Tunable tunable) {
        fields.push_back(FieldSchema{.path = path,
                                     .type = typeName(value),
                                     .default_value = defaultText(value),
                                     .doc = doc,
                                     .tunable = tunable == Tunable::kYes});
    });
    return fields;
}

std::string schemaJson(const std::map<std::string, std::string>& slots,
                       const std::vector<PluginSchema>& plugins) {
    std::ostringstream out;
    out << "{\n  \"fields\": [\n";
    const auto fields = schema();
    for (std::size_t i = 0; i < fields.size(); ++i) {
        const auto& field = fields[i];
        out << "    {\"path\": " << quoteJson(field.path) << ", \"type\": "
            << quoteJson(field.type) << ", \"default\": " << quoteJson(field.default_value)
            << ", \"doc\": " << quoteJson(field.doc) << ", \"tunable\": "
            << (field.tunable ? "true" : "false") << "}" << (i + 1 < fields.size() ? "," : "")
            << '\n';
    }
    out << "  ],\n  \"slots\": {";
    std::size_t n = 0;
    for (const auto& [path, interface_name] : slots) {
        out << (n++ == 0 ? "\n" : ",\n") << "    " << quoteJson(path) << ": "
            << quoteJson(interface_name);
    }
    out << (slots.empty() ? "" : "\n  ") << "},\n  \"plugins\": [";
    for (std::size_t i = 0; i < plugins.size(); ++i) {
        const auto& plugin = plugins[i];
        out << (i == 0 ? "\n" : ",\n") << "    {\"interface\": " << quoteJson(plugin.interface_name)
            << ", \"type\": " << quoteJson(plugin.type) << ", \"fields\": [";
        for (std::size_t j = 0; j < plugin.fields.size(); ++j) {
            const auto& field = plugin.fields[j];
            out << (j == 0 ? "" : ", ") << "{\"name\": " << quoteJson(field.name)
                << ", \"kind\": " << quoteJson(field.kind) << ", \"default\": "
                << quoteJson(field.default_value) << ", \"doc\": " << quoteJson(field.doc);
            if (!field.default_spec.type.empty()) {
                out << ", \"default_spec\": " << pluginJson(field.default_spec);
            }
            out << "}";
        }
        out << "]}";
    }
    out << (plugins.empty() ? "" : "\n  ") << "]\n}\n";
    return out.str();
}

void validateConfig(const TaskConfig& config) {
    // 有限性：类型检查拦得住 "abc"，拦不住 1e400 这种溢出成 inf 的写法
    visitConfig(config, [](const char* path, const auto& value, const char*, Tunable) {
        if constexpr (std::is_same_v<std::decay_t<decltype(value)>, double>) {
            if (!std::isfinite(value)) throw ConfigError(std::string(path) + " 必须是有限实数");
        }
    });
    if (config.task.name.empty()) {
        throw ConfigError("task.name 不能为空：任务记录的目录与结论都用它");
    }
    requireOneOf("task.mode", config.task.mode, {"offline", "online"});

    const auto& system = config.system;
    requireOneOf("system.pipeline", system.pipeline, {"odometry", "mapping", "localization"});
    requireAtLeast("system.runtime.spin_budget", system.runtime.spin_budget, 1);
    requireAtLeast("system.runtime.online_max_in_flight", system.runtime.online_max_in_flight, 0);
    requireOneOf("system.packetizer.source_policy", system.packetizer.source_policy,
                 {"source", "config"});
    requireAtLeast("system.packetizer.max_imu_backlog", system.packetizer.max_imu_backlog, 1);
    requireDuration("system.packetizer.post_support_s", system.packetizer.post_support_s);
    requireDuration("system.packetizer.pre_support_s", system.packetizer.pre_support_s);
    requireDuration("system.packetizer.attachment_tolerance_s",
                    system.packetizer.attachment_tolerance_s);
    requireDuration("system.packetizer.max_imu_gap_s", system.packetizer.max_imu_gap_s);
    const auto& graph = system.modules.graph_optimizer;
    if (!(graph.odometry_translation_weight > 0.0)) {
        throw ConfigError("system.modules.graph_optimizer.odometry_translation_weight 必须为正");
    }
    if (!(graph.odometry_rotation_weight > 0.0)) {
        throw ConfigError("system.modules.graph_optimizer.odometry_rotation_weight 必须为正");
    }

    // 序列名是子任务记录的目录名，也写进子进程的覆盖项：只收不用转义、不会越出任务记录目录的名字
    const auto safe = [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
               || c == '.' || c == '_' || c == '-';
    };
    std::set<std::string> seen;
    for (const auto& sequence : config.data.sequences) {
        if (sequence.empty()) throw ConfigError("data.sequences 里有空的序列名");
        if (!std::all_of(sequence.begin(), sequence.end(), safe) || sequence == "."
            || sequence == "..") {
            throw ConfigError("data.sequences 里的序列名 " + sequence
                              + " 要能当目录名：只用字母、数字与 . _ -，也不能是 . 或 ..");
        }
        if (!seen.insert(sequence).second) {
            throw ConfigError("data.sequences 里的序列名重复：" + sequence);
        }
    }

    // 评测函数的名字是指标名的第一段与表格的文件名：只收字母、数字与 _
    for (const auto& [name, spec] : config.evaluation.evaluators) {
        const bool valid = !name.empty() && std::all_of(name.begin(), name.end(), [](char c) {
            return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
        });
        if (!valid) {
            throw ConfigError("evaluation.evaluators 里的名字 " + name
                              + " 只能用字母、数字与 _：它是指标名的第一段，也是表格的文件名");
        }
        if (spec.type.empty()) {
            throw ConfigError("evaluation.evaluators." + name + " 要写 type，选一个评测函数");
        }
    }
    // 在线评测选的是上面列出的评测函数：同一份参数，跑着评与跑完评才对得上
    const auto& online = config.evaluation.online;
    std::set<std::string> online_names;
    for (const auto& name : online.evaluators) {
        if (!config.evaluation.evaluators.contains(name)) {
            throw ConfigError("evaluation.online.evaluators 里的 " + name
                              + " 不在 evaluation.evaluators 里：先在那里列出它");
        }
        if (!online_names.insert(name).second) {
            throw ConfigError("evaluation.online.evaluators 里 " + name + " 写了两次");
        }
    }
    requireDuration("evaluation.online.interval_s", online.interval_s);
    if (!(online.interval_s > 0.0)) throw ConfigError("evaluation.online.interval_s 必须为正");

    const auto& log = config.outputs.log;
    requireOneOf("outputs.log.console", log.console, {"debug", "info", "warn", "error"});
    requireOneOf("outputs.log.file", log.file, {"debug", "info", "warn", "error"});
    // 键与配置路径同一种写法：段由字母、数字与 _ 组成，用 . 隔开
    const auto valid_key = [](std::string_view key) {
        if (key.empty() || key.front() == '.' || key.back() == '.') return false;
        bool previous_dot = false;
        for (const char c : key) {
            const bool dot = c == '.';
            if (dot && previous_dot) return false;
            if (!dot && !(std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_')) {
                return false;
            }
            previous_dot = dot;
        }
        return true;
    };
    std::set<std::string> keys;
    for (const auto& rule : log.levels) {
        const auto equals = rule.find('=');
        const std::string key = rule.substr(0, equals);
        if (equals == std::string::npos || !valid_key(key)) {
            throw ConfigError("outputs.log.levels 的每条要写成 <来源的键>=<等级>，如 "
                              "odometry.plugin.map=debug，得到 " + rule);
        }
        requireOneOf("outputs.log.levels 里 " + key + " 的等级", rule.substr(equals + 1),
                     {"debug", "info", "warn", "error"});
        if (!keys.insert(key).second) {
            throw ConfigError("outputs.log.levels 里 " + key + " 写了两次");
        }
    }
}

LoadedTask loadTaskString(const std::string& text, const std::string& origin,
                          const std::string& base_dir, const std::vector<std::string>& overrides) {
    YAML::Node root = parseYaml(text, origin);
    if (!root || root.IsNull()) root = YAML::Node(YAML::NodeType::Map);
    if (!root.IsMap()) throw ConfigError(origin + ": 任务文件的顶层必须是映射");

    LoadedTask loaded;
    loaded.source.task_file = origin;
    const YAML::Node& view = root;  // 只读查找：非 const 的 operator[] 可能插入空键

    if (const YAML::Node listed = view["overrides"]) {
        if (!listed.IsSequence()) throw ConfigError(origin + ": overrides 要写成列表");
        for (const auto& item : listed) {
            if (!item.IsScalar()) throw ConfigError(origin + ": overrides 的元素要写成 路径=值");
            loaded.source.overrides.push_back(item.Scalar());
        }
    }
    for (const auto& item : overrides) loaded.source.overrides.push_back(item);

    // system 写的是路径：读那份系统文件，挂到 system 下。写成映射的是任务记录里还原后的任务，原样用
    if (const YAML::Node system = view["system"]; system && system.IsScalar()) {
        std::filesystem::path file(system.Scalar());
        if (file.is_relative()) file = std::filesystem::path(base_dir) / file;
        loaded.source.system_file = file.lexically_normal().string();
        YAML::Node content = parseYaml(readFile(loaded.source.system_file, "系统文件"),
                                       loaded.source.system_file);
        if (!content || content.IsNull()) content = YAML::Node(YAML::NodeType::Map);
        if (!content.IsMap()) {
            throw ConfigError(loaded.source.system_file + ": 系统文件的顶层必须是映射");
        }
        root["system"] = content;
    } else if (system && !system.IsMap() && !system.IsNull()) {
        throw ConfigError(origin + ": system 要写成系统文件的路径");
    }

    // 未知键先查：拼错的键静默忽略，比报错难查得多
    const FieldPaths paths = fieldPaths();
    std::vector<std::string> present;
    collectLeaves(root, "", paths, present);
    for (const auto& path : present) {
        if (path == "overrides" || paths.all.count(path) != 0) continue;
        const bool in_system = path.rfind("system.", 0) == 0 && !loaded.source.system_file.empty();
        const std::string& where = in_system ? loaded.source.system_file : origin;
        throw ConfigError(where + ": 未知配置项 " + path);
    }

    visitConfig(loaded.config, [&](const char* path, auto& value, const char*, Tunable) {
        const YAML::Node node = nodeAt(root, path);
        if (!node || node.IsNull()) return;  // 缺省即默认值
        assign(value, node, path, origin);
    });
    // 类型对了不等于值有意义：覆盖全部到位之后整体校验
    applyOverrides(loaded.config, loaded.source.overrides);
    return loaded;
}

LoadedTask loadTaskFile(const std::string& path, const std::vector<std::string>& overrides) {
    const auto parent = std::filesystem::path(path).parent_path();
    return loadTaskString(readFile(path, "任务文件"), path,
                          parent.empty() ? std::string(".") : parent.string(), overrides);
}

void applyOverrides(TaskConfig& config, const std::vector<std::string>& assignments) {
    for (const auto& assignment : assignments) applyAssignment(config, assignment);
    validateConfig(config);
}

std::string dumpConfig(const TaskConfig& config) { return dump(config); }

void writeConfigFile(const std::string& path, const TaskConfig& config) {
    std::ofstream file(path);
    if (!file) throw ConfigError("打不开配置输出文件: " + path);
    file << dumpConfig(config);
    if (!file) throw ConfigError("写配置文件失败: " + path);
}

}  // namespace simpleslam::config
