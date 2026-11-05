// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file voxel_key.hpp
/// 体素坐标：一个点落在哪个体素。体素降采样与体素地图共用，用户写自己的体素类实现也直接用它。
///
/// 坐标除以体素边长后向下取整，结果钳在 int32 范围内，两端各留一格，所以邻域 ±1 不会溢出。
/// 超出范围的点落进边界体素：浮点转整数不会越界成未定义行为，查询结果由距离判据把关。
/// 体素边长 1 毫米时，可表示的范围是离原点 ±2147 公里。

#include <SimpleSLAM/core/types/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace simpleslam::algo {

struct VoxelKey {
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;

    /// `inv_size` 是体素边长的倒数，调用方预先算好，免得逐点做除法。
    [[nodiscard]] static VoxelKey of(const Vec3d& p, double inv_size) noexcept {
        return {.x = cell(p.x(), inv_size), .y = cell(p.y(), inv_size), .z = cell(p.z(), inv_size)};
    }

    friend bool operator==(const VoxelKey&, const VoxelKey&) = default;

private:
    [[nodiscard]] static std::int32_t cell(double v, double inv_size) noexcept {
        constexpr double kLo = static_cast<double>(std::numeric_limits<std::int32_t>::min()) + 1.0;
        constexpr double kHi = static_cast<double>(std::numeric_limits<std::int32_t>::max()) - 1.0;
        return static_cast<std::int32_t>(std::clamp(std::floor(v * inv_size), kLo, kHi));
    }
};

/// Teschner 等的空间哈希：各分量乘一个大素数后异或。乘法在无符号域里做，没有有符号溢出。
struct VoxelKeyHash {
    [[nodiscard]] std::size_t operator()(const VoxelKey& k) const noexcept {
        const auto u = [](std::int32_t v) {
            return static_cast<std::uint64_t>(static_cast<std::uint32_t>(v));
        };
        return static_cast<std::size_t>((u(k.x) * 73856093ULL) ^ (u(k.y) * 19349669ULL) ^
                                        (u(k.z) * 83492791ULL));
    }
};

}  // namespace simpleslam::algo
