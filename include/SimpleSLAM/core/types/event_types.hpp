#pragma once

/// @file event_types.hpp
/// Topic 通信事件类型——通过 shared_ptr<const T> 不可变共享
///
/// 所有事件结构均为纯值类型，发布后不可修改。
/// 命名约定：T_A_B 表示"从 B 到 A 的变换"，与 convention.hpp 一致。

#include <SimpleSLAM/core/types/common.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/core/types/sensor_data.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

namespace simpleslam {

/// Odometry 产出关键帧时发布
struct KeyframeEvent {
    uint64_t keyframe_id;
    Timestamp timestamp;
    SE3d pose;                                      ///< T_world_body
    std::shared_ptr<const LidarScan> scan;          ///< 零拷贝共享原始点云
    std::shared_ptr<const ImageFrame> image;         ///< 可选
};

/// LoopDetector 检测到回环时发布
struct LoopDetectedEvent {
    uint64_t query_keyframe_id;
    uint64_t match_keyframe_id;
    SE3d T_match_query;                             ///< 从 query 到 match 的相对变换
    double confidence;                               ///< 置信度 [0, 1]
};

/// PGOptimizer 优化完成后发布——携带所有被校正的位姿
struct CorrectionEvent {
    std::unordered_map<uint64_t, SE3d> corrected_poses;  ///< kf_id → 校正后的 T_world_body
};

/// Odometry 跟踪丢失时发布
struct TrackingLostEvent {
    Timestamp timestamp;
    SE3d last_good_pose;                            ///< 最后一个可靠的 T_world_body
};

/// 系统关闭事件
struct ShutdownEvent {
    std::string reason;
};

}  // namespace simpleslam
