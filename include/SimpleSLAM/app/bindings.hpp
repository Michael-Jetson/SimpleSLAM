// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file bindings.hpp
/// 配置 → 运行期类型与装配选项。
///
/// 放在入口层，因为这里是唯一同时认识配置层与模块层的地方：配置层只描述"写了什么"，
/// 模块层只认运行期类型，两者互不依赖。"配置里的秒"与"代码里的纳秒"只在这里换算一次，
/// 不会散落在各模块里各换各的。

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

}  // namespace simpleslam::app
