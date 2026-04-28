#pragma once

/// @file registration_target.hpp
/// RegistrationTarget C++20 concept —— LiDAR 配准的统一抽象
///
/// 把"用于状态估计的地图结构"和"作用其上的配准算法"绑定为不可拆分的 concept。
/// 不同地图结构存储不同几何信息，残差计算方式与存储方式强耦合。
///
/// 用 concept 而非虚函数：热路径零开销，编译器可内联整个 match 循环。
/// MatchResult 由调用者持有并复用：避免每帧堆分配。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/core/types/sensor_data.hpp>

#include <concepts>
#include <cstddef>
#include <vector>

namespace simpleslam {

/// 配准结果缓冲区——调用者在循环外分配，每帧 clear + 复用
struct MatchResult {
    std::vector<double> residuals;   ///< 每个有效匹配的残差
    std::vector<double> jacobians;   ///< row-major, 每行 6 列（与 Eigen::Map 兼容）
    int num_valid = 0;               ///< 有效匹配点数

    void reserve(size_t max_points) {
        residuals.reserve(max_points);
        jacobians.reserve(max_points * 6);
    }

    void clear() {
        residuals.clear();
        jacobians.clear();
        num_valid = 0;
    }
};

/// LiDAR 配准专用 concept
/// VIO 视觉重投影配准是不同的数学结构，v1.5 时单独设计接口
template <typename T>
concept RegistrationTarget = requires(
    T& target,
    const LidarScan& scan,
    const SE3d& pose,
    MatchResult& result) {
    { target.match(scan, pose, result) } -> std::same_as<void>;
    { target.update(scan, pose) } -> std::same_as<void>;
    { target.empty() } -> std::convertible_to<bool>;
    { target.size() } -> std::convertible_to<std::size_t>;
};

}  // namespace simpleslam
