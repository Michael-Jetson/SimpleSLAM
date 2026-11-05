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

comm::RuntimeOptions toRuntimeOptions(const config::TaskConfig& config) {
    comm::RuntimeOptions options;
    options.offline = config.task.mode == "offline";
    return options;
}

modules::SlamSystemOptions toSystemOptions(const config::TaskConfig& config) {
    modules::SlamSystemOptions options;
    options.offline = config.task.mode == "offline";
    options.spin_budget = static_cast<std::size_t>(config.system.runtime.spin_budget);
    options.online_max_in_flight =
        static_cast<std::size_t>(config.system.runtime.online_max_in_flight);
    return options;
}

modules::OdometryEdgePolicy toEdgePolicy(const config::GraphOptimizerModuleSection& section) {
    modules::OdometryEdgePolicy policy;
    policy.translation_weight = section.odometry_translation_weight;
    policy.rotation_weight = section.odometry_rotation_weight;
    return policy;
}

std::string configParamName(const std::string& path) {
    constexpr std::string_view kModules = "system.modules.";
    std::string name;
    if (path.rfind(kModules, 0) == 0) {
        name = "/slam/";
        for (const char c : path.substr(kModules.size())) name += (c == '.' ? '/' : c);
        return name;
    }
    name = "/config/";
    for (const char c : path) name += (c == '.' ? '/' : c);
    return name;
}

void readFromParameters(config::TaskConfig& config, const comm::ParamSnapshot& snapshot) {
    config::visitConfig(config, [&](const char* path, auto& value, const char*,
                                    config::Tunable tunable) {
        if (tunable != config::Tunable::kYes) return;
        using ValueType = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<ValueType, bool> || std::is_same_v<ValueType, std::int64_t>
                      || std::is_same_v<ValueType, double>
                      || std::is_same_v<ValueType, std::string>) {
            if (auto stored = snapshot.get<ValueType>(configParamName(path))) value = *stored;
        }
    });
}

void applyToParameters(const config::TaskConfig& config, comm::ParameterStore& store) {
    config::visitConfig(config, [&](const char* path, const auto& value, const char*,
                                    config::Tunable tunable) {
        // 不可调的字段不进运行期参数：改了也不会生效
        if (tunable != config::Tunable::kYes) return;
        using ValueType = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<ValueType, bool> || std::is_same_v<ValueType, std::int64_t>
                      || std::is_same_v<ValueType, double>
                      || std::is_same_v<ValueType, std::string>) {
            store.preset(configParamName(path), comm::ParamValue{value});
        }
    });
    store.setValidator([base = config](const comm::ParamSnapshot& candidate, std::string& detail) {
        config::TaskConfig probe = base;
        readFromParameters(probe, candidate);
        try {
            config::validateConfig(probe);
        } catch (const config::ConfigError& e) {
            detail = e.what();
            return false;
        }
        return true;
    });
}

}  // namespace simpleslam::app
