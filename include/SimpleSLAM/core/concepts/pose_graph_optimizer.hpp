#pragma once

/// @file pose_graph_optimizer.hpp
/// PoseGraphOptimizer C++20 concept —— 位姿图优化的统一抽象
///
/// 具体实现（GTSAM iSAM2、mini-PGO 等）满足此 concept 即可。

#include <SimpleSLAM/core/types/geometry.hpp>

#include <concepts>
#include <cstdint>
#include <unordered_map>

namespace simpleslam {

/// 优化结果
struct OptimizationResult {
    std::unordered_map<uint64_t, SE3d> optimized_poses;  ///< 节点 ID → 优化后位姿
    double final_error{0.0};
    int iterations{0};
    bool converged{false};
};

/// 位姿图优化器 concept
template <typename T>
concept PoseGraphOptimizer = requires(
    T& optimizer,
    uint64_t from_id, uint64_t to_id,
    const SE3d& relative_pose,
    const Mat6d& information) {
    { optimizer.addOdometryEdge(from_id, to_id, relative_pose, information) };
    { optimizer.addLoopEdge(from_id, to_id, relative_pose, information) };
    { optimizer.optimize() } -> std::convertible_to<OptimizationResult>;
    { optimizer.reset() } -> std::same_as<void>;
};

}  // namespace simpleslam
