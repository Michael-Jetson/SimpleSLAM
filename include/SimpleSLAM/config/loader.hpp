// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file loader.hpp
/// 任务配置的加载、导出与 schema。全部从 visitConfig 派生，字段只写一次。
///
/// 加载的顺序：读任务文件 → `system` 写的是路径就读那份系统文件（相对任务文件所在目录），
/// 挂到 `system` 下 → 依次应用文件里的 `overrides` 与调用方给的覆盖项 → 严格校验。
/// 插件节点（`PluginSpec`）只在这里读成"类型 + 参数"，按实现的字段清单校验是目录的事。
///
/// 规矩：
/// 1. **未知键是错误**，不是"忽略"。插件节点内部的键由目录在还原时查。
/// 2. **类型不符是错误**，不做隐式转换（`"30"` 不是 30，`1` 不是 true）。
/// 3. **规范化导出与 YAML 里的注释、键序、空白无关**。插件节点先由目录还原（补齐默认值）
///    再导出，写不写默认值得到同一份导出。两份任务有什么不同，比它们的导出。
/// 4. **规范化导出必须语义无损**：`load(dump(c)) == c`。字符串一律加引号转义。
///    导出不含覆盖项和文件出处：导出的是覆盖之后的结果，出处另记在 `TaskSource`。
/// 5. **类型对了不等于值有意义**：枚举拼错、时长为负，都在加载时就报错。
///
/// Python 侧不重写这套 schema：它读 `schemaJson()` 的输出（入口的 schema 命令），
/// 因此两边不会各自漂移。

#include <SimpleSLAM/config/config.hpp>
#include <SimpleSLAM/config/error.hpp>
#include <SimpleSLAM/config/plugin.hpp>

#include <map>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace simpleslam::config {

/// schema 里一个字段的描述。导出给 Python、给文档、给命令行补全。
struct FieldSchema {
    std::string path;
    std::string type;  ///< bool / int / double / string / list / slot / slots / params
    std::string default_value;
    std::string doc;
};

/// 改过写法的配置项：旧写法在加载与覆盖时翻译成新写法，并给出替换用的新写法
/// （`docs/architecture.md`「六、配置层规则」的兼容）。
struct RenamedField {
    std::string_view old_path;
    std::string_view new_path;
};
[[nodiscard]] std::span<const RenamedField> renamedFields();

/// 一个模块用到的参数：schema 里与实现的参数一起导出。
struct ModuleParams {
    std::string module;
    std::vector<comm::ParamUse> params;
};

[[nodiscard]] std::vector<FieldSchema> schema();

/// schema 的 JSON 导出。`slots` 是插件字段到它要求的接口名，`plugins` 是各目录里
/// 每个实现的字段清单与参数，`modules` 是模块自己用到的参数；都由知道目录与模块的一方
/// （入口层）给出，配置层不认识接口与模块。
[[nodiscard]] std::string schemaJson(const std::map<std::string, std::string>& slots = {},
                                     const std::vector<PluginSchema>& plugins = {},
                                     const std::vector<ModuleParams>& modules = {});

/// 整体校验：枚举、范围、有限性。不合法抛 ConfigError。
void validateConfig(const TaskConfig& config);

/// 一个任务的出处：任务文件、它引用的系统文件、施加过的覆盖项（文件里的在前）。
/// 原件存进任务记录的 `source/`；规范化导出不含它：同一份任务换一种写法，导出相同。
struct TaskSource {
    std::string task_file;
    std::string system_file;
    std::vector<std::string> overrides;
};

/// 加载时用到的旧写法。
struct Renames {
    /// 告诉人的话：每处一条，说明替换用的新写法。
    std::vector<std::string> notes;
    /// 经旧写法给了值的参数名（不带 `system.params.`）。还原任务时要用：这次没有组件声明它的，
    /// 照旧写法当初的样子不起作用、也不报错。
    std::set<std::string> params;
};

struct LoadedTask {
    TaskConfig config;
    TaskSource source;
    Renames renames;
};

/// `base_dir` 用来解析 `system` 里的相对路径。
[[nodiscard]] LoadedTask loadTaskString(const std::string& text, const std::string& origin,
                                        const std::string& base_dir = ".",
                                        const std::vector<std::string>& overrides = {});
[[nodiscard]] LoadedTask loadTaskFile(const std::string& path,
                                      const std::vector<std::string>& overrides = {});

/// 应用"路径=值"覆盖项，写法与任务文件里的 `overrides` 相同；改某个插件节点的 `type`
/// 时，这个节点换成新实现的默认参数；参数写成 `system.params.<名字>=值`。然后整体校验。
/// 用了旧写法时记进 `renames`。
void applyOverrides(TaskConfig& config, const std::vector<std::string>& assignments,
                    Renames* renames = nullptr);

/// 规范化导出：按 schema 顺序、按节嵌套、数值格式固定，写不写默认值、键序与注释都不影响它。
/// 任务记录里的 `task.yaml` 就是它，两份任务要不要比、比什么，直接比它。
[[nodiscard]] std::string dumpConfig(const TaskConfig& config);
void writeConfigFile(const std::string& path, const TaskConfig& config);

}  // namespace simpleslam::config
