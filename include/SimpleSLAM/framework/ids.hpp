// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file ids.hpp
/// 强类型标识。混用不同种类的 id 必须编译不过，而不是靠命名约定。

#include <compare>
#include <cstdint>
#include <functional>

namespace simpleslam::fw {

#define SIMPLESLAM_FW_DEFINE_ID(Name)                                                \
    struct Name {                                                                    \
        std::uint64_t value = 0;                                                     \
        friend constexpr bool operator==(Name, Name) noexcept = default;             \
        friend constexpr auto operator<=>(Name, Name) noexcept = default;            \
        [[nodiscard]] constexpr bool valid() const noexcept { return value != 0; }   \
    }

/// 关键帧：真源的最小单位，从 1 开始，0 表示无效。
SIMPLESLAM_FW_DEFINE_ID(KeyframeId);
/// 子图：粗粒度锚点。
SIMPLESLAM_FW_DEFINE_ID(SubmapId);
/// 建图会话：换一次会话，旧结果一律身份失效。
SIMPLESLAM_FW_DEFINE_ID(SessionId);
/// 请求代号：防止旧任务的结果覆盖新任务。
SIMPLESLAM_FW_DEFINE_ID(RequestGeneration);
/// 约束代号：一次性约束的稳定身份，用于去重、重试与提交确认。
/// 通知可以丢、可以重发，但"这条约束是否已经进入真源"必须由它判定。
SIMPLESLAM_FW_DEFINE_ID(ConstraintId);

#undef SIMPLESLAM_FW_DEFINE_ID

}  // namespace simpleslam::fw

#define SIMPLESLAM_FW_HASH_ID(Name)                                             \
    template <>                                                                 \
    struct std::hash<simpleslam::fw::Name> {                                    \
        std::size_t operator()(simpleslam::fw::Name id) const noexcept {        \
            return std::hash<std::uint64_t>{}(id.value);                        \
        }                                                                       \
    }

SIMPLESLAM_FW_HASH_ID(KeyframeId);
SIMPLESLAM_FW_HASH_ID(SubmapId);
SIMPLESLAM_FW_HASH_ID(SessionId);
SIMPLESLAM_FW_HASH_ID(RequestGeneration);
SIMPLESLAM_FW_HASH_ID(ConstraintId);

#undef SIMPLESLAM_FW_HASH_ID
