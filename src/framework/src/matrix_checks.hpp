// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file matrix_checks.hpp
/// 6×6 矩阵的对称半正定判据。图边的信息矩阵与外参的协方差共用这一套容差，
/// 免得两处各判各的。实现里用到特征值求解，只在这一个翻译单元实例化。

#include <SimpleSLAM/core/types/geometry.hpp>

#include <cstdint>

namespace simpleslam::fw::detail {

enum class Definiteness : std::uint8_t { kSymmetricPsd, kAsymmetric, kIndefinite };

/// 输入必须是有限值。容差按矩阵自身的量级取：1e-14 量级的矩阵也要能被判对称、半正定；
/// 全零矩阵算对称半正定（没有信息，但不违法）。
[[nodiscard]] Definiteness definiteness(const Mat6d& m);

}  // namespace simpleslam::fw::detail
