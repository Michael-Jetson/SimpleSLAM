// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file loader.hpp
/// 任务配置的加载、导出、哈希与 schema。全部从 visitConfig 派生，字段只写一次。
///
/// 加载的顺序：读任务文件 → `system` 写的是路径就读那份系统文件（相对任务文件所在目录），
/// 挂到 `system` 下 → 依次应用文件里的 `overrides` 与调用方给的覆盖项 → 严格校验。
/// 插件节点（`PluginSpec`）只在这里读成"类型 + 参数"，按实现的字段清单校验是目录的事。
///
/// 规矩：
/// 1. **未知键是错误**，不是"忽略"。插件节点内部的键由目录在还原时查。
/// 2. **类型不符是错误**，不做隐式转换（`"30"` 不是 30，`1` 不是 true）。
/// 3. **配置哈希是规范化导出的哈希**，与 YAML 里的注释、键序、空白无关。插件节点要先由
///    目录还原（补齐默认值）再哈希，否则写不写默认值会得出两个哈希。
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
#include <string>
#include <vector>

namespace simpleslam::config {

/// schema 里一个字段的描述。导出给 Python、给文档、给命令行补全。
struct FieldSchema {
    std::string path;
    std::string type;  ///< bool / int / double / string / list / slot
    std::string default_value;
    std::string doc;
    bool tunable = false;
};

[[nodiscard]] std::vector<FieldSchema> schema();

/// schema 的 JSON 导出。`slots` 是插件字段到它要求的接口名，`plugins` 是各目录里
/// 每个实现的字段清单；两者由知道目录的一方（入口层）给出，配置层不认识接口。
[[nodiscard]] std::string schemaJson(const std::map<std::string, std::string>& slots = {},
                                     const std::vector<PluginSchema>& plugins = {});

/// 整体校验：枚举、范围、有限性。不合法抛 ConfigError。
void validateConfig(const TaskConfig& config);

/// 一个任务的出处：任务文件、它引用的系统文件、施加过的覆盖项（文件里的在前）。
/// 进任务档案，不进配置哈希：同一份结果，换一种写法得到的应当是同一个哈希。
struct TaskSource {
    std::string task_file;
    std::string system_file;
    std::vector<std::string> overrides;
};

struct LoadedTask {
    TaskConfig config;
    TaskSource source;
};

/// `base_dir` 用来解析 `system` 里的相对路径。
[[nodiscard]] LoadedTask loadTaskString(const std::string& text, const std::string& origin,
                                        const std::string& base_dir = ".",
                                        const std::vector<std::string>& overrides = {});
[[nodiscard]] LoadedTask loadTaskFile(const std::string& path,
                                      const std::vector<std::string>& overrides = {});

/// 应用"路径=值"覆盖项，写法与任务文件里的 `overrides` 相同；改某个插件节点的 `type`
/// 时，这个节点换成新实现的默认参数。然后整体校验。
void applyOverrides(TaskConfig& config, const std::vector<std::string>& assignments);

/// 规范化导出：按 schema 顺序、按节嵌套、数值格式固定。哈希基于它，所以不能随手改。
[[nodiscard]] std::string dumpConfig(const TaskConfig& config);
void writeConfigFile(const std::string& path, const TaskConfig& config);

/// 整个任务的哈希，不含 `outputs`：产出放在哪里不影响结果，换个档案目录重跑仍是同一个任务。
[[nodiscard]] std::string configHash(const TaskConfig& config);
/// 只覆盖 `system` 子树：标识算法的配置，与数据、评测、产出无关。
[[nodiscard]] std::string systemHash(const TaskConfig& config);

/// FNV-1a 64 的十六进制摘要（与 Python 侧一致）。
[[nodiscard]] std::string digestHex(const std::string& text);

/// 有效输入（键值对）的哈希：长度前缀编码，键或值里带 '=' 或换行也不会撞。
[[nodiscard]] std::string effectiveHash(const std::map<std::string, std::string>& effective);

}  // namespace simpleslam::config
