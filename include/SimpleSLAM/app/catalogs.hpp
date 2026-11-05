// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file catalogs.hpp
/// 内置实现的目录，以及任务里每个插件字段要求的接口。
///
/// 内置的有：算法库的滤波与地图、三种数据集读取器、位置识别与图优化的 `none`。
/// 里程计方案没有内置实现：算法本体由用户写，在自己的入口里追加到目录。

#include <SimpleSLAM/config/plugin.hpp>

#include <map>
#include <string>

namespace simpleslam::app {

/// 登记好内置实现的目录。用户在它上面追加自己的实现，再交给 `app::run`。
[[nodiscard]] config::Catalogs builtinCatalogs();

/// 任务里的插件字段到它要求的接口名。schema 导出用。
[[nodiscard]] std::map<std::string, std::string> slotInterfaces();

}  // namespace simpleslam::app
