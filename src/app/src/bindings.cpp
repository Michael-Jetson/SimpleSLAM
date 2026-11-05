// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/app/bindings.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <stdexcept>

namespace simpleslam::app {

fw::Duration secondsToDuration(double seconds) {
    // int64 纳秒约可表示 ±9.2e9 秒；留出余量，拒绝在舍入时可能越界的值
    constexpr double kRepresentableSeconds = 9.0e9;
    if (!std::isfinite(seconds) || std::abs(seconds) > kRepresentableSeconds) {
        throw std::out_of_range(std::format("时长超出可表示的范围: {} 秒", seconds));
    }
    return fw::Duration{static_cast<std::int64_t>(std::llround(seconds * 1e9))};
}

fw::PacketizerPolicy toPacketizerPolicy(const config::PacketizerSection& section) {
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

comm::RuntimeOptions toRuntimeOptions(const config::RuntimeSection& section) {
    comm::RuntimeOptions options;
    options.offline = section.offline;
    return options;
}

modules::SlamSystemOptions toSystemOptions(const config::SlamConfig& config) {
    modules::SlamSystemOptions options;
    options.offline = config.runtime.offline;
    options.spin_budget = static_cast<std::size_t>(config.runtime.spin_budget);
    return options;
}

modules::OdometryEdgePolicy toEdgePolicy(const config::BackendSection& section) {
    modules::OdometryEdgePolicy policy;
    policy.translation_weight = section.odometry_translation_weight;
    policy.rotation_weight = section.odometry_rotation_weight;
    return policy;
}

}  // namespace simpleslam::app
