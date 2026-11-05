// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file config_bindings.hpp
/// 配置 → 装配选项。放在模块层是因为 SlamSystemOptions 属于这一层，
/// 而配置层不应该反过来依赖模块层。

#include <SimpleSLAM/config/bindings.hpp>
#include <SimpleSLAM/modules/slam_system.hpp>

namespace simpleslam::modules {

[[nodiscard]] inline SlamSystemOptions toSystemOptions(const config::SlamConfig& config) {
    SlamSystemOptions options;
    options.offline = config.runtime.offline;
    options.spin_budget = static_cast<std::size_t>(config.runtime.spin_budget);
    return options;
}

[[nodiscard]] inline OdometryEdgePolicy toEdgePolicy(const config::BackendSection& section) {
    OdometryEdgePolicy policy;
    policy.translation_weight = section.odometry_translation_weight;
    policy.rotation_weight = section.odometry_rotation_weight;
    return policy;
}

}  // namespace simpleslam::modules
