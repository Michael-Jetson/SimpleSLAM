// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file digest.hpp
/// 内容摘要：配置哈希、标定哈希、有效身份、轨迹文件绑定共用这一份实现。
///
/// FNV-1a 64，十六进制输出。它是**身份**摘要，不是安全摘要：用来回答"两次说的是不是
/// 同一份内容"，不用来抵御有意构造的碰撞。只有一份实现，Python 侧的同名算法由
/// 对照测试逐字节比对。

#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>

namespace simpleslam::fw {

[[nodiscard]] inline std::string digestHex(std::string_view text) {
    std::uint64_t hash = 0xcbf29ce484222325ULL;  // FNV-1a 64 偏移基
    for (const unsigned char c : text) {
        hash ^= c;
        hash *= 0x100000001b3ULL;  // FNV-1a 64 质数
    }
    char buffer[24];
    std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(hash));
    return buffer;
}

}  // namespace simpleslam::fw
