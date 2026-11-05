// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

// 算法库内部共用的小工具：各接口在接缝处做的同一组检查。不是公共头文件。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/sensor_data.hpp>

#include <cmath>
#include <cstddef>

namespace simpleslam::algo::detail {

/// 各列长度一致、坐标都是有限数：算法库各接口对点云输入输出的共同前提。
[[nodiscard]] inline bool wellFormed(const fw::PointCloud& cloud) noexcept {
    if (!cloud.consistent()) return false;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        if (!std::isfinite(cloud.x[i]) || !std::isfinite(cloud.y[i]) ||
            !std::isfinite(cloud.z[i])) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] inline bool positiveFinite(double v) noexcept { return std::isfinite(v) && v > 0.0; }

[[nodiscard]] inline Vec3d pointAt(const fw::PointCloud& cloud, std::size_t i) noexcept {
    return {static_cast<double>(cloud.x[i]), static_cast<double>(cloud.y[i]),
            static_cast<double>(cloud.z[i])};
}

}  // namespace simpleslam::algo::detail
