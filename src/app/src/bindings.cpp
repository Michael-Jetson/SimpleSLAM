// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/app/bindings.hpp>
#include <SimpleSLAM/config/error.hpp>
#include <SimpleSLAM/config/loader.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

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

modules::SlamSystemOptions toSystemOptions(const config::TaskConfig& config) {
    modules::SlamSystemOptions options;
    options.offline = config.task.mode == "offline";
    options.spin_budget = static_cast<std::size_t>(config.system.runtime.spin_budget);
    options.online_max_in_flight =
        static_cast<std::size_t>(config.system.runtime.online_max_in_flight);
    // 等级在配置校验时已经核对过：这里只换算
    const auto level = [](std::string_view text) {
        return logging::parseLevel(text).value_or(logging::Level::kInfo);
    };
    options.log.console = level(config.outputs.log.console);
    options.log.file = level(config.outputs.log.file);
    for (const auto& rule : config.outputs.log.levels) {
        const auto equals = rule.find('=');
        options.log.rules.push_back(
            {.key = rule.substr(0, equals), .level = level(rule.substr(equals + 1))});
    }
    return options;
}

}  // namespace simpleslam::app
