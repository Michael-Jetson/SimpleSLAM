// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include "legacy.hpp"

#include <SimpleSLAM/config/error.hpp>

#include <array>
#include <cstdint>
#include <string_view>
#include <tuple>
#include <variant>

namespace simpleslam::config::legacy {

namespace {

constexpr std::string_view kEvaluators = "evaluation.evaluators";

/// v2 评测一节里挪进评测函数参数的字段：旧键、参数名、给 ate 还是 rpe。
struct Moved {
    const char* old_key;
    const char* param;
    bool ate;
    bool rpe;
};

constexpr std::array<Moved, 6> kMoved = {{
    {.old_key = "alignment", .param = "alignment", .ate = true, .rpe = false},
    {.old_key = "association", .param = "association", .ate = true, .rpe = true},
    {.old_key = "association_tolerance_s", .param = "tolerance_s", .ate = true, .rpe = true},
    {.old_key = "max_interpolation_gap_s",
     .param = "max_interpolation_gap_s",
     .ate = true,
     .rpe = true},
    {.old_key = "rpe_delta_m", .param = "delta_m", .ate = false, .rpe = true},
    {.old_key = "rpe_delta_s", .param = "delta_s", .ate = false, .rpe = true},
}};

/// 去掉的键取别的值时怎么办。取旧默认值时一律静默去掉：v2、v3 写的每份 task.yaml 都带着它们。
enum class Otherwise : std::uint8_t {
    kQuiet,  ///< 照样静默去掉：它的作用已经没有了
    kNote,   ///< 去掉，说明一句
    kError,  ///< 报错：忽略它会让人以为它生效了
};

/// v4 去掉的键：没有替代的写法，删掉这一行即可。
struct Removed {
    const char* section;
    const char* key;
    const char* old_default;  ///< 旧默认值：true / false、整数，或空串
    Otherwise otherwise;
    const char* reason;
};

constexpr std::array<Removed, 5> kRemoved = {{
    {.section = "task",
     .key = "seed",
     .old_default = "0",
     .otherwise = Otherwise::kNote,
     .reason = "没有算法读它；要用随机数的算法，把种子写成那个插件的参数"},
    {.section = "evaluation",
     .key = "allow_incomplete",
     .old_default = "false",
     .otherwise = Otherwise::kQuiet,
     .reason = "不完整的运行照常评测，是否完整另记在结论里"},
    {.section = "evaluation",
     .key = "baselines",
     .old_default = "",
     .otherwise = Otherwise::kError,
     .reason = "基线门禁去掉了，精度对比看 ./run.py compare 的对比表"},
    {.section = "outputs",
     .key = "record",
     .old_default = "false",
     .otherwise = Otherwise::kError,
     .reason = "录制去掉了：数据来自文件时重跑即可，实时数据用 rosbag 录"},
    {.section = "outputs",
     .key = "record_diagnostics",
     .old_default = "true",
     .otherwise = Otherwise::kQuiet,
     .reason = "随录制一起去掉了"},
}};

bool isQuoted(const YAML::Node& node) { return node.Tag() == "!"; }

/// 取的是不是旧默认值。按类型比，`False` 与 `false`、`00` 与 `0` 一样；类型不对就不是。
bool isOldDefault(const YAML::Node& node, std::string_view old_default) {
    if (old_default.empty()) return node.IsNull() || (node.IsScalar() && node.Scalar().empty());
    if (!node.IsScalar()) return false;
    try {
        if (old_default == "true" || old_default == "false") {
            return !isQuoted(node) && node.as<bool>() == (old_default == "true");
        }
        return !isQuoted(node) && node.as<std::int64_t>() == std::stoll(std::string(old_default));
    } catch (const YAML::Exception&) {
        return false;
    }
}

/// 一个去掉的键：静默去掉、说明一句或报错。`where` 写在说明与报错的开头。
void retire(const Removed& removed, const YAML::Node& value, const std::string& where,
            std::vector<std::string>& notes) {
    if (isOldDefault(value, removed.old_default) || removed.otherwise == Otherwise::kQuiet) return;
    const std::string path = std::string(removed.section) + "." + removed.key;
    const std::string message = where + path + " 去掉了（" + removed.reason + "），删掉这一行";
    if (removed.otherwise == Otherwise::kError) throw ConfigError(message);
    notes.push_back(message + "即可");
}

/// 映射里的一个键。没有就是空：默认构造的 `YAML::Node` 是已定义的 null，不能拿来表示"没有"。
std::optional<YAML::Node> child(const YAML::Node& parent, const char* key) {
    if (!parent || !parent.IsMap()) return std::nullopt;
    const YAML::Node node = parent[key];
    if (!node) return std::nullopt;
    return node;
}

std::string join(const std::vector<std::string>& items) {
    std::string out;
    for (const auto& item : items) out += (out.empty() ? "" : "、") + item;
    return out;
}

std::string scalarOf(const YAML::Node& node, const std::string& path, const std::string& origin) {
    if (!node.IsScalar()) throw ConfigError(origin + ": " + path + " 要写成标量");
    return node.Scalar();
}

double realOf(const YAML::Node& node, const std::string& path, const std::string& origin) {
    try {
        if (!isQuoted(node)) return node.as<double>();
    } catch (const YAML::Exception&) {
    }
    throw ConfigError(origin + ": " + path + " 要写成实数，得到 " + scalarOf(node, path, origin));
}

/// 说明里新写法的一个评测函数，写成行内映射；参数的文本照抄旧写法。
std::string flowEntry(const std::string& type,
                      const std::vector<std::pair<std::string, std::string>>& params) {
    std::string out = "{type: " + type;
    for (const auto& [key, text] : params) out += ", " + key + ": " + text;
    return out + "}";
}

bool isZero(const PluginValue& value) {
    if (const auto* integer = std::get_if<std::int64_t>(&value)) return *integer == 0;
    if (const auto* real = std::get_if<double>(&value)) return *real == 0.0;
    return false;
}

/// 覆盖项的值按 YAML 读成标量文本，`"run"` 与 `run` 一样。
std::string overrideText(const std::string& text) {
    try {
        const YAML::Node node = YAML::Load(text);
        if (node.IsScalar()) return node.Scalar();
    } catch (const YAML::Exception&) {
    }
    return text;
}

}  // namespace

const std::vector<std::pair<std::string, std::string>>& fields() {
    static const std::vector<std::pair<std::string, std::string>> table = {
        {"task.purpose", "去掉了：评不评测看 evaluation.evaluators，列了评测函数就评，没列就只跑"},
        {"evaluation.format", "outputs.kitti_trajectories（TUM 格式总会写）"},
        {"evaluation.alignment", "evaluation.evaluators.ate.alignment"},
        {"evaluation.association", "evaluation.evaluators 里 ate、rpe 的 association"},
        {"evaluation.association_tolerance_s", "evaluation.evaluators 里 ate、rpe 的 tolerance_s"},
        {"evaluation.max_interpolation_gap_s",
         "evaluation.evaluators 里 ate、rpe 的 max_interpolation_gap_s"},
        {"evaluation.rpe_delta_m", "evaluation.evaluators.rpe.delta_m"},
        {"evaluation.rpe_delta_s", "evaluation.evaluators.rpe.delta_s"},
    };
    static const auto with_removed = [] {
        auto out = table;
        for (const auto& removed : kRemoved) {
            out.emplace_back(std::string(removed.section) + "." + removed.key,
                             std::string("去掉了：") + removed.reason);
        }
        return out;
    }();
    return with_removed;
}

void upgradeTree(YAML::Node& root, const std::string& origin, std::vector<std::string>& notes) {
    const YAML::Node& view = root;  // 只读查找：非 const 的 operator[] 会插入空键
    for (const auto& removed : kRemoved) {
        const auto section = child(view, removed.section);
        const auto value = section ? child(*section, removed.key) : std::nullopt;
        if (!value) continue;
        retire(removed, *value, origin + ": ", notes);
        root[removed.section].remove(removed.key);
        if (root[removed.section].size() == 0) root.remove(removed.section);
    }
    const auto task = child(view, "task");
    const auto evaluation = child(view, "evaluation");
    const auto purpose = task ? child(*task, "purpose") : std::nullopt;
    const auto format = evaluation ? child(*evaluation, "format") : std::nullopt;
    const auto old = [&](const Moved& moved) {
        return evaluation ? child(*evaluation, moved.old_key) : std::nullopt;
    };

    std::vector<std::string> found;
    if (purpose) found.emplace_back("task.purpose");
    if (format) found.emplace_back("evaluation.format");
    for (const auto& moved : kMoved) {
        if (old(moved)) found.push_back(std::string("evaluation.") + moved.old_key);
    }
    if (found.empty()) return;
    if (evaluation && child(*evaluation, "evaluators")) {
        throw ConfigError(origin + ": 新旧两种写法不能混用：写了 evaluation.evaluators，又写了 "
                          + join(found));
    }

    // v2 默认做实验：只写了旧的评测字段、没写 purpose 的任务也评测
    const std::string kind = purpose ? scalarOf(*purpose, "task.purpose", origin) : "experiment";
    if (kind != "experiment" && kind != "run") {
        throw ConfigError(origin + ": task.purpose 只收 experiment / run，得到 " + kind);
    }
    YAML::Node evaluators(YAML::NodeType::Map);
    std::string snippet = "{}";
    std::string remark;
    if (kind == "experiment") {
        YAML::Node ate(YAML::NodeType::Map);
        YAML::Node rpe(YAML::NodeType::Map);
        ate["type"] = "ate";
        rpe["type"] = "rpe";
        std::vector<std::pair<std::string, std::string>> ate_text;
        std::vector<std::pair<std::string, std::string>> rpe_text;
        double delta_m = 100.0;  // v2 的默认值，与现在 rpe 的默认值相同
        double delta_s = 0.0;
        for (const auto& moved : kMoved) {
            const auto found_node = old(moved);
            if (!found_node) continue;
            const YAML::Node& node = *found_node;
            const std::string path = std::string("evaluation.") + moved.old_key;
            const std::string text = isQuoted(node) ? "\"" + scalarOf(node, path, origin) + "\""
                                                    : scalarOf(node, path, origin);
            if (moved.ate) {
                ate[moved.param] = YAML::Clone(node);
                ate_text.emplace_back(moved.param, text);
            }
            if (moved.rpe) {
                rpe[moved.param] = YAML::Clone(node);
                rpe_text.emplace_back(moved.param, text);
            }
            if (std::string_view(moved.param) == "delta_m") delta_m = realOf(node, path, origin);
            if (std::string_view(moved.param) == "delta_s") delta_s = realOf(node, path, origin);
        }
        evaluators["ate"] = ate;
        snippet = "{ate: " + flowEntry("ate", ate_text);
        if (delta_m != 0.0 || delta_s != 0.0) {
            evaluators["rpe"] = rpe;
            snippet += ", rpe: " + flowEntry("rpe", rpe_text);
        } else {
            remark = "（两个 RPE 间隔都为 0，按 v2 的意思不算 RPE）";
        }
        snippet += "}";
    } else if (found.size() > 1 + (format ? 1 : 0)) {
        remark = "（purpose: run 不评测，旧的评测字段一并忽略，v2 也不用它们）";
    }

    bool kitti = false;
    if (format) {
        const std::string text = scalarOf(*format, "evaluation.format", origin);
        if (text != "tum" && text != "kitti") {
            throw ConfigError(origin + ": evaluation.format 只收 tum / kitti，得到 " + text);
        }
        const auto outputs = child(view, "outputs");
        if (outputs && child(*outputs, "kitti_trajectories")) {
            throw ConfigError(origin + ": 新旧两种写法不能混用：写了 outputs.kitti_trajectories，"
                                       "又写了 evaluation.format");
        }
        kitti = text == "kitti";
    }

    // 改树：去掉旧键，写上新写法。只剩空映射的节一并去掉，空映射会被当成未知的叶子
    if (purpose) {
        root["task"].remove("purpose");
        if (root["task"].size() == 0) root.remove("task");
    }
    if (evaluation && evaluation->IsMap()) {
        root["evaluation"].remove("format");
        for (const auto& moved : kMoved) root["evaluation"].remove(moved.old_key);
    }
    root["evaluation"]["evaluators"] = evaluators;
    if (kitti) root["outputs"]["kitti_trajectories"] = true;

    std::string note = origin + ": 用了 v2 的写法（" + join(found)
                       + "），已按下面的新写法理解，请照着改" + remark
                       + "：\n  evaluation:\n    evaluators: " + snippet;
    if (kitti) note += "\n  outputs:\n    kitti_trajectories: true";
    notes.push_back(std::move(note));
}

std::optional<Translated> translateAssignment(const TaskConfig& config, const std::string& path,
                                              const std::string& text,
                                              std::vector<std::string>& notes) {
    const auto note = [&](const std::string& meaning) {
        notes.push_back("覆盖项 " + path + "=" + text + " 是 v2 的写法，按 " + meaning + " 理解");
    };
    for (const auto& removed : kRemoved) {
        if (path != std::string(removed.section) + "." + removed.key) continue;
        YAML::Node value;
        try {
            value = YAML::Load(text);
        } catch (const YAML::Exception&) {
            value = YAML::Node(text);
        }
        retire(removed, value, "覆盖项 ", notes);
        return Translated{};  // 不必改：这个键已经没有了
    }
    if (path == "task.purpose") {
        const std::string kind = overrideText(text);
        if (kind == "run") {
            note("evaluation.evaluators={}（只跑不评测）");
            return Translated{.assignments = {"evaluation.evaluators={}"}, .rpe_deltas = false};
        }
        if (kind != "experiment") {
            throw ConfigError("覆盖项 task.purpose 只收 experiment / run，得到 " + kind);
        }
        if (!config.evaluation.evaluators.empty()) {
            note("评测，任务里已经列了评测函数，不用改");
            return Translated{};
        }
        const std::string assignment = "evaluation.evaluators={ate: {type: ate}, rpe: {type: rpe}}";
        note(assignment);
        return Translated{.assignments = {assignment}, .rpe_deltas = false};
    }
    if (path == "evaluation.format") {
        const std::string format = overrideText(text);
        if (format != "tum" && format != "kitti") {
            throw ConfigError("覆盖项 evaluation.format 只收 tum / kitti，得到 " + format);
        }
        const std::string assignment =
            std::string("outputs.kitti_trajectories=") + (format == "kitti" ? "true" : "false");
        note(assignment);
        return Translated{.assignments = {assignment}, .rpe_deltas = false};
    }
    constexpr std::string_view kPrefix = "evaluation.";
    if (!path.starts_with(kPrefix)) return std::nullopt;
    const std::string key = path.substr(kPrefix.size());
    for (const auto& moved : kMoved) {
        if (key != moved.old_key) continue;
        Translated out;
        out.rpe_deltas = !moved.ate;
        const std::pair<const char*, bool> wanted_by[] = {{"ate", moved.ate}, {"rpe", moved.rpe}};
        for (const auto& [name, wanted] : wanted_by) {
            const auto it = config.evaluation.evaluators.find(name);
            if (!wanted || it == config.evaluation.evaluators.end() || it->second.type != name) {
                continue;
            }
            out.assignments.push_back(std::string(kEvaluators) + "." + name + "." + moved.param
                                      + "=" + text);
        }
        if (out.assignments.empty()) {
            const std::string targets =
                moved.ate && moved.rpe ? "ate 或 rpe" : (moved.ate ? "ate" : "rpe");
            throw ConfigError("覆盖项 " + path + " 是 v2 的写法，对应评测函数 " + targets + " 的 "
                              + moved.param + "，但任务里没有名为 " + targets + " 的评测函数");
        }
        note(join(out.assignments));
        return out;
    }
    return std::nullopt;
}

void dropRpeWithoutDelta(TaskConfig& config, std::vector<std::string>& notes) {
    auto& evaluators = config.evaluation.evaluators;
    const auto it = evaluators.find("rpe");
    if (it == evaluators.end() || it->second.type != "rpe") return;
    const auto& params = it->second.params;
    const auto m = params.find("delta_m");
    const auto s = params.find("delta_s");
    // 没写的 delta_m 是默认的 100 米，不为 0
    if (m != params.end() && isZero(m->second) && (s == params.end() || isZero(s->second))) {
        evaluators.erase(it);
        notes.emplace_back("RPE 的两个间隔都为 0：按 v2 的意思不算 RPE，去掉了评测函数 rpe");
    }
}

}  // namespace simpleslam::config::legacy
