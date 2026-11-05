// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file loader.hpp
/// 配置的加载、导出、哈希与 schema。全部从 visitConfig 派生，字段只写一次。
///
/// 三条规矩：
/// 1. **未知键是错误**，不是"忽略"。`odometr.max_iterations` 拼错一个字母却静默
///    跑完，比报错难查一百倍。
/// 2. **类型不符是错误**，不做隐式转换（`"30"` 不是 30，`1` 不是 true）。
/// 3. **配置哈希是规范化导出的哈希**，与 YAML 里的注释、键序、空白无关。
///    它进实验回放包与评测报告，因此"这次跑的是哪份配置"可查。
/// 4. **规范化导出必须语义无损**：`load(dump(c)) == c`。字符串一律加引号转义——
///    不加引号时 `name: "a # b"` 会在往返中变成 `a`。
/// 5. **类型对了不等于值有意义**：线程数为负、支撑区间为 nan、枚举拼错，
///    都在加载时就报错，而不是跑到一半才出怪现象。
///
/// Python 侧不重写这套 schema：它读 `schemaJson()` 的输出（simpleslam-config --schema），
/// 因此两边不会各自漂移。

#include <SimpleSLAM/comm/parameters.hpp>
#include <SimpleSLAM/config/config.hpp>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace simpleslam::config {

class ConfigError : public std::runtime_error {
public:
    explicit ConfigError(const std::string& what) : std::runtime_error(what) {}
};

/// schema 里一个字段的描述。导出给 Python、给文档、给命令行补全。
struct FieldSchema {
    std::string path;
    std::string type;  ///< bool / int / double / string
    std::string default_value;
    std::string doc;
    bool tunable = false;
};

[[nodiscard]] std::vector<FieldSchema> schema();
[[nodiscard]] std::string schemaJson();

/// 语义校验：范围、有限性、枚举。加载与命令行覆盖之后都会调用它。
void validateConfig(const SlamConfig& config);

/// 读配置并应用命令行覆盖（`路径=值`），全部到位之后整体校验一次。
///
/// 跨字段的约束（打开录制就要给输出路径）只能看最终的配置：逐项校验会让覆盖的先后
/// 决定成败，文件里写了开关、命令行再补路径的用法也会在读文件时就被拒。
[[nodiscard]] SlamConfig loadConfigString(const std::string& text,
                                          const std::string& origin = "<string>",
                                          const std::vector<std::string>& overrides = {});
[[nodiscard]] SlamConfig loadConfigFile(const std::string& path,
                                        const std::vector<std::string>& overrides = {});

/// 规范化导出：键序固定、无注释、数值格式固定。配置哈希基于它。
[[nodiscard]] std::string dumpConfig(const SlamConfig& config);
void writeConfigFile(const std::string& path, const SlamConfig& config);

/// 配置哈希（FNV-1a 64，十六进制）。进回放包与评测报告。
[[nodiscard]] std::string configHash(const SlamConfig& config);

/// 任意文本的 FNV-1a 64 摘要（十六进制）。配置哈希、标定哈希、有效身份共用它。
[[nodiscard]] std::string digestHex(const std::string& text);

/// **有效身份**：实际生效、影响结论的全部输入的摘要。
///
/// 配置哈希只覆盖配置文件本身：标定文件换了内容而路径不变、数据源实测速率变了从而
/// 推荐策略跟着变，配置哈希都看不见。基线要绑定的是实际用了什么，所以这里对
/// 键值对的有序集合取摘要——键里应当包括配置哈希、标定内容哈希、解析后的打包策略。
/// 编码带长度前缀：不同的集合不会拼成同一个串。
[[nodiscard]] std::string effectiveHash(const std::map<std::string, std::string>& effective);

/// 覆盖：`odometry.max_iterations=40` 形式的命令行覆盖，按 schema 的类型解析。
/// 一组覆盖全部应用之后才整体校验，理由同 loadConfigFile()。
void applyOverrides(SlamConfig& config, const std::vector<std::string>& assignments);

/// 运行期参数名：`odometry.max_iterations` → `/config/odometry/max_iterations`。
[[nodiscard]] std::string configParamName(const std::string& path);

/// 把可调字段预置进参数存储，并装上与启动加载**同源**的校验器。
///
/// 只预置值是不够的：运行期的 commit 只保证类型一致，于是启动时会被 validateConfig
/// 拒绝的值（负的迭代次数、非有限的支撑时长）在运行期能直接改进去。校验器把候选
/// 快照读回配置聚合体再跑同一套 validateConfig，因此"启动能不能这么配"与
/// "运行期能不能改成这样"是同一个判据。校验失败整批拒绝，版本不变。
void applyToParameters(const SlamConfig& config, comm::ParameterStore& store);

/// 只装校验器，不预置值。
///
/// base 提供不可调字段的取值：候选快照里只有可调字段，跨字段校验要用到的那些
/// 不可调字段得从这里来，否则校验的是一份半真半假的配置。
void installConfigValidator(comm::ParameterStore& store, const SlamConfig& base = {});

/// 反向：把参数存储里已提交的可调字段读回配置聚合体。
/// 用于"当前生效的配置是什么"——它与启动时的配置可能已经不同。
void readFromParameters(SlamConfig& config, const comm::ParamSnapshot& snapshot);

}  // namespace simpleslam::config
