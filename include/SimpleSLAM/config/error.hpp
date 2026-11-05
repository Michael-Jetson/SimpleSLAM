// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file error.hpp
/// 配置错误。加载、校验、按目录还原插件时报的错都是它。
///
/// 单独成一个只依赖标准库的头文件：插件目录（`config/plugin.hpp`）也报这种错，
/// 而算法库包含目录时不应该连带依赖通信层。

#include <stdexcept>
#include <string>

namespace simpleslam::config {

class ConfigError : public std::runtime_error {
public:
    explicit ConfigError(const std::string& what) : std::runtime_error(what) {}
};

}  // namespace simpleslam::config
