// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file bindings.hpp
/// 配置 → 运行期类型的转换。集中在一处，因此"配置里的秒"与"代码里的纳秒"
/// 只在这里换算一次，不会散落在各模块里各换各的。

#include <SimpleSLAM/comm/runtime.hpp>
#include <SimpleSLAM/config/config.hpp>
#include <SimpleSLAM/framework/sensor_source.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace simpleslam::config {

namespace fw = simpleslam::fw;

/// 秒 → 纳秒。四舍五入（截断会让 0.3 s 变成 299999999 ns），并拒绝非有限值与纳秒整型
/// 装不下的量——溢出不会报错，只会悄悄变成一个负的极大值。
[[nodiscard]] inline fw::Duration secondsToDuration(double seconds) {
    // int64 纳秒约可表示 ±9.2e9 秒；留出余量，拒绝在舍入时可能越界的值
    constexpr double kRepresentableSeconds = 9.0e9;
    if (!std::isfinite(seconds) || std::abs(seconds) > kRepresentableSeconds) {
        throw std::out_of_range("时长超出可表示的范围: " + std::to_string(seconds) + " 秒");
    }
    return fw::Duration{static_cast<std::int64_t>(std::llround(seconds * 1e9))};
}

[[nodiscard]] inline fw::PacketizerPolicy toPacketizerPolicy(const PacketizerSection& section) {
    fw::PacketizerPolicy policy;
    policy.trigger_on_lidar = section.trigger_on_lidar;
    policy.imu = section.imu_required ? fw::ImuRequirement::kRequired
                                      : fw::ImuRequirement::kNotRequired;
    policy.post_support = secondsToDuration(section.post_support_s);
    policy.pre_support = secondsToDuration(section.pre_support_s);
    policy.max_imu_gap = secondsToDuration(section.max_imu_gap_s);
    policy.max_imu_backlog = static_cast<std::size_t>(section.max_imu_backlog);
    policy.attachment_tolerance = secondsToDuration(section.attachment_tolerance_s);
    return policy;
}

[[nodiscard]] inline comm::RuntimeOptions toRuntimeOptions(const RuntimeSection& section) {
    comm::RuntimeOptions options;
    options.offline = section.offline;  // 离线用虚拟时钟，不等墙钟
    return options;
}

}  // namespace simpleslam::config
