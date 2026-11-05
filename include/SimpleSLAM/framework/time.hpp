// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file time.hpp
/// 时间体系：纳秒整型 + 显式时钟域。
///
/// chrono 的纳秒类型解决表示与单位，解决不了"两个设备是否同一时钟"。
/// 因此每个样本带 ClockDomainId，域到参考时间的映射带版本与不确定度；
/// 硬件同步时映射是恒等式且成本为零，但"已同步"从此是可检查的字段。
///
/// 命名空间 fw 是过渡：迁移完成后与旧的 simpleslam 类型合并（见 docs/design/legacy.md）。

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>

namespace simpleslam::fw {

/// 某个时钟域内的时刻，纳秒整型。
using Timestamp = std::chrono::nanoseconds;
/// 时间间隔，纳秒整型。
using Duration = std::chrono::nanoseconds;

/// 时钟域标识。0 保留给参考域（系统对齐后的运行时间）。
struct ClockDomainId {
    std::uint32_t value = 0;

    friend constexpr bool operator==(ClockDomainId, ClockDomainId) noexcept = default;
    friend constexpr auto operator<=>(ClockDomainId, ClockDomainId) noexcept = default;
};

inline constexpr ClockDomainId kReferenceClock{0};

/// 域到参考时间的仿射映射，围绕一个整数锚点：
///
///   t_ref = t_device + offset + round((scale − 1) × (t_device − device_anchor))
///
/// 与 `scale × t_device + offset` 在数学上相同（锚点为零时），但计算方式不同：长时间戳
/// （Unix 纳秒约 1.3e18）整体转成 double 再乘，末几位纳秒就丢了，连恒等映射都做不到恒等。
/// 这里整数部分走整数，只有漂移项用浮点——它小，而且锚点取在数据附近时更小。
/// scale 恰为 1 时整条走整数路径，精确。
struct ClockMapping {
    ClockDomainId domain{};
    double scale = 1.0;
    Duration offset{0};
    /// 漂移项的锚点（设备时间）。取在数据附近，漂移项就小。
    Timestamp device_anchor{0};
    std::uint32_t version = 0;
    /// 时间不确定度。硬件同步也要记录一个非零估计，不默认误差为零。
    Duration uncertainty{0};
    bool synchronized = false;
    std::string source;  ///< 映射来源：硬件触发 / PTP / 软件估计 / 标定

    [[nodiscard]] Timestamp apply(Timestamp device_time) const {
        Timestamp mapped = device_time + offset;
        if (scale != 1.0) {
            const auto since_anchor = static_cast<double>((device_time - device_anchor).count());
            mapped += Duration{static_cast<std::int64_t>(std::llround((scale - 1.0) * since_anchor))};
        }
        return mapped;
    }
};

/// 一次换算的结果：时刻，以及它建立在哪一版映射、多大不确定度、是否硬件同步之上。
/// 只拿时刻的话，"软件估计、误差几毫秒"与"硬件触发、同一时钟"长得一模一样。
struct ClockConversion {
    Timestamp time{0};
    std::uint32_t version = 0;
    Duration uncertainty{0};
    bool synchronized = false;
};

/// 半开区间 [begin, end)。
struct TimeRange {
    Timestamp begin{0};
    Timestamp end{0};

    [[nodiscard]] bool empty() const noexcept { return end <= begin; }
    [[nodiscard]] Duration span() const noexcept { return end - begin; }
    [[nodiscard]] bool contains(Timestamp t) const noexcept { return t >= begin && t < end; }
    /// 本区间是否完整覆盖 other。
    [[nodiscard]] bool covers(const TimeRange& other) const noexcept {
        return begin <= other.begin && end >= other.end;
    }
};

/// 时钟域登记表。缺映射时转换显式失败，不悄悄当作同一时钟。
class ClockRegistry {
public:
    void setMapping(ClockMapping mapping) { mappings_[mapping.domain.value] = std::move(mapping); }

    [[nodiscard]] std::optional<ClockMapping> mapping(ClockDomainId domain) const {
        auto it = mappings_.find(domain.value);
        if (it == mappings_.end()) return std::nullopt;
        return it->second;
    }

    /// 转到参考时间。没有登记映射就返回空——调用方必须处理，不得外推。
    /// 只给时刻；要知道映射版本、不确定度与是否同步，用 convert()。
    [[nodiscard]] std::optional<Timestamp> toReference(ClockDomainId domain,
                                                       Timestamp device_time) const {
        const auto converted = convert(domain, device_time);
        if (!converted) return std::nullopt;
        return converted->time;
    }

    /// 转到参考时间，连同映射的版本、不确定度与是否同步。参考域自身是精确的恒等。
    [[nodiscard]] std::optional<ClockConversion> convert(ClockDomainId domain,
                                                         Timestamp device_time) const {
        if (domain == kReferenceClock) return ClockConversion{device_time, 0, Duration{0}, true};
        const auto m = mapping(domain);
        if (!m) return std::nullopt;
        return ClockConversion{m->apply(device_time), m->version, m->uncertainty, m->synchronized};
    }

    /// 该域是否声明为已同步。未登记视为未同步。
    [[nodiscard]] bool synchronized(ClockDomainId domain) const {
        if (domain == kReferenceClock) return true;
        const auto m = mapping(domain);
        return m && m->synchronized;
    }

    [[nodiscard]] std::size_t size() const noexcept { return mappings_.size(); }

private:
    std::unordered_map<std::uint32_t, ClockMapping> mappings_;
};

}  // namespace simpleslam::fw

template <>
struct std::hash<simpleslam::fw::ClockDomainId> {
    std::size_t operator()(simpleslam::fw::ClockDomainId id) const noexcept {
        return std::hash<std::uint32_t>{}(id.value);
    }
};
