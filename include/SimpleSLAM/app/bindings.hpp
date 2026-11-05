// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file bindings.hpp
/// 配置 → 运行期类型、装配选项与运行期参数。
///
/// 放在入口层，因为这里是唯一同时认识配置层与模块层的地方：配置层只描述"写了什么"，
/// 模块层只认运行期类型，两者互不依赖。"配置里的秒"与"代码里的纳秒"只在这里换算一次，
/// 不会散落在各模块里各换各的。

#include <SimpleSLAM/comm/parameters.hpp>
#include <SimpleSLAM/comm/runtime.hpp>
#include <SimpleSLAM/config/config.hpp>
#include <SimpleSLAM/framework/sensor_source.hpp>
#include <SimpleSLAM/framework/time.hpp>
#include <SimpleSLAM/modules/slam_system.hpp>

#include <string>

namespace simpleslam::app {

/// 秒 → 纳秒。四舍五入（截断会让 0.3 s 变成 299999999 ns），并拒绝非有限值与纳秒整型
/// 装不下的量——溢出不会报错，只会悄悄变成一个负的极大值。
[[nodiscard]] fw::Duration secondsToDuration(double seconds);

[[nodiscard]] fw::PacketizerPolicy toPacketizerPolicy(const config::PacketizerSection& section);

/// 离线用虚拟时钟，不等墙钟。
[[nodiscard]] comm::RuntimeOptions toRuntimeOptions(const config::TaskConfig& config);

[[nodiscard]] modules::SlamSystemOptions toSystemOptions(const config::TaskConfig& config);

[[nodiscard]] modules::OdometryEdgePolicy toEdgePolicy(
    const config::GraphOptimizerModuleSection& section);

/// 可调字段对应的运行期参数名。`system.modules.<模块>.<字段>` 是那个模块的私有参数
/// `/slam/<模块>/<字段>`，模块用 `~<字段>` 读，不必知道配置的布局；其余是 `/config/<路径>`。
[[nodiscard]] std::string configParamName(const std::string& path);

/// 把可调字段推进参数存储，并装上校验：运行期提交的参数要过和加载时同一套校验。
void applyToParameters(const config::TaskConfig& config, comm::ParameterStore& store);

/// 把参数快照里的可调字段读回配置（校验与回放用）。
void readFromParameters(config::TaskConfig& config, const comm::ParamSnapshot& snapshot);

}  // namespace simpleslam::app
