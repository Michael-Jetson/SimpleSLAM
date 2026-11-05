// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file bundle.hpp
/// 实验回放包：让"这个结果是怎么来的"成为一个文件，而不是一段记忆。
///
/// 一次实验可复现，需要同时钉住四样东西：
///   1. **配置**——配置哈希（config/loader.hpp 的规范化导出之哈希）；
///   2. **输入**——数据集或录制文件的摘要（内容指纹，不是路径）；
///   3. **随机性**——种子；
///   4. **代码**——版本号或 git 修订。
/// 少任何一样，"复现"都只是"再跑一次，看起来差不多"。
///
/// 回放包本身是一个小 YAML 文件，与录制文件放在一起。

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace simpleslam::record {

class BundleError : public std::runtime_error {
public:
    explicit BundleError(const std::string& what) : std::runtime_error(what) {}
};

/// 一个输入的内容指纹。记路径没有用：同一个路径下的文件会被换掉。
struct InputDigest {
    std::string role;    ///< 例如 "recording" / "groundtruth" / "dataset"
    std::string path;    ///< 记录当时的路径，便于人查，但不参与判等
    std::string digest;  ///< 内容指纹（FNV-1a 64，十六进制）
    std::uint64_t bytes = 0;
};

struct ExperimentBundle {
    std::string name;
    std::string created_at;
    std::string code_revision;
    std::string config_hash;
    std::int64_t seed = 0;
    std::uint64_t session = 0;
    std::vector<InputDigest> inputs;
    /// 这次跑出来的关键数字。进包是为了"打开包就知道当时是什么水平"。
    std::map<std::string, double> metrics;
    std::string note;

    [[nodiscard]] const InputDigest* input(const std::string& role) const;
};

/// 文件内容指纹。流式读取，不把整个数据集读进内存。
[[nodiscard]] std::string fileDigest(const std::string& path, std::uint64_t* size_out = nullptr);

void writeBundle(const std::string& path, const ExperimentBundle& bundle);
[[nodiscard]] ExperimentBundle readBundle(const std::string& path);

/// 比对两个回放包：哪些条件变了。返回人可读的差异描述，空表示条件一致。
[[nodiscard]] std::vector<std::string> diffBundles(const ExperimentBundle& before,
                                                   const ExperimentBundle& after);

}  // namespace simpleslam::record
