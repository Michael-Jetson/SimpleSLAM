#pragma once

/// @file odometry_result.hpp
/// Odometry 前端输出结构——一帧位姿估计的完整结果

#include <SimpleSLAM/core/types/common.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>

#include <cstdint>
#include <optional>

namespace simpleslam {

/// 前端跟踪状态
enum class TrackingStatus : uint8_t {
    Initializing,  ///< 初始化中（积累数据 / 等待足够视差）
    Tracking,      ///< 正常跟踪
    Degraded,      ///< 退化环境（隧道、窄走廊等，部分自由度不可观）
    Lost,          ///< 跟踪丢失
};

/// 一帧 Odometry 输出
struct OdometryResult {
    Timestamp timestamp{0.0};
    SE3d pose{};                ///< T_world_body（含已应用的后端校正）
    Mat6d covariance{};         ///< 6x6 协方差（切空间 [tx ty tz rx ry rz] 顺序）
    TrackingStatus status{TrackingStatus::Initializing};
    bool is_keyframe{false};

    /// 各自由度的信息量——值越小越退化（可选，由退化检测模块填充）
    std::optional<Eigen::Matrix<double, 6, 1>> degeneracy;

    [[nodiscard]] bool isTracking() const {
        return status == TrackingStatus::Tracking;
    }
};

}  // namespace simpleslam
