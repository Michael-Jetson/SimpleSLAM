#pragma once

/// @file convention.hpp
/// 框架全局公约——所有开发者必须遵守
///
/// 世界坐标系：ENU（东-北-天），右手系
/// Body frame = IMU frame
/// 位姿语义：T_world_body，即 p_world = T_world_body * p_body
/// 外参命名：T_body_sensor（如 T_body_lidar、T_body_camera）
/// 时间戳：double，单位秒
///   - 传感器数据时间戳使用 UTC 纪元秒
///   - 系统内部计时使用 steady_clock 相对秒数

#include <string_view>

namespace simpleslam::convention {

// ── 坐标系 ──
inline constexpr std::string_view kCoordinateSystem = "ENU";
inline constexpr std::string_view kBodyFrame = "IMU";

// ── 位姿语义 ──
inline constexpr std::string_view kPoseSemantics = "T_world_body";

// ── 精度 ──
inline constexpr std::string_view kPointPrecision = "float";
inline constexpr std::string_view kStatePrecision = "double";

// ── 时间戳 ──
inline constexpr std::string_view kTimestampUnit = "seconds";

// ── 并发约束 ──
// shared_mutex 写锁持有时间必须 < 1ms

}  // namespace simpleslam::convention
