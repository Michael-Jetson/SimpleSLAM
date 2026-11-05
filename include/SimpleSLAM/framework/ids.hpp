// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file ids.hpp
/// 强类型标识。混用不同种类的 id 必须编译不过，而不是靠命名约定。
///
/// 新增一种 id 只写一行：`using XxxId = Id<struct XxxTag>;`。标签只用来区分种类，
/// 不需要定义。

#include <compare>
#include <cstdint>
#include <functional>

namespace simpleslam::fw {

/// 0 是否表示"没有"。计数型 id 从 1 开始，0 表示无效；0 本身有含义的种类在这里登记为 false，
/// 对它调用 `valid()` 编译不过。
template <class Tag>
inline constexpr bool kZeroMeansNone = true;

template <class Tag, class Rep = std::uint64_t>
struct Id {
    Rep value = 0;

    friend constexpr bool operator==(Id, Id) noexcept = default;
    friend constexpr auto operator<=>(Id, Id) noexcept = default;

    [[nodiscard]] constexpr bool valid() const noexcept {
        static_assert(kZeroMeansNone<Tag>, "这种 id 的 0 有含义，不表示无效");
        return value != 0;
    }
};

/// 关键帧：真源的最小单位，从 1 开始，0 表示无效。
using KeyframeId = Id<struct KeyframeTag>;
/// 子图：粗粒度锚点。
using SubmapId = Id<struct SubmapTag>;
/// 建图会话：换一次会话，旧结果一律身份失效。
using SessionId = Id<struct SessionTag>;
/// 请求代号：防止旧请求的结果覆盖新请求。
using RequestGeneration = Id<struct RequestGenerationTag>;
/// 约束代号：一次性约束的稳定身份，用于去重、重试与提交确认。
/// "这条约束是否已经进入真源"由它判定。
using ConstraintId = Id<struct ConstraintTag>;

/// 时钟域。0 是参考域（系统对齐后的运行时间），不表示无效。
using ClockDomainId = Id<struct ClockDomainTag, std::uint32_t>;
template <>
inline constexpr bool kZeroMeansNone<ClockDomainTag> = false;

}  // namespace simpleslam::fw

template <class Tag, class Rep>
struct std::hash<simpleslam::fw::Id<Tag, Rep>> {
    std::size_t operator()(simpleslam::fw::Id<Tag, Rep> id) const noexcept {
        return std::hash<Rep>{}(id.value);
    }
};
