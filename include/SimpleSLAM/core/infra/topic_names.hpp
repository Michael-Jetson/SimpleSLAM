#pragma once

/// @file topic_names.hpp
/// 标准话题名常量——防止魔术字符串，编译期检查拼写

#include <string_view>

namespace simpleslam::topic_names {

inline constexpr std::string_view kSensorLidar    = "sensor/lidar";
inline constexpr std::string_view kSensorImu      = "sensor/imu";
inline constexpr std::string_view kSensorImage    = "sensor/image";
inline constexpr std::string_view kSlamOdometry   = "slam/odometry";
inline constexpr std::string_view kSlamKeyframe   = "slam/keyframe";
inline constexpr std::string_view kSlamLoop       = "slam/loop";
inline constexpr std::string_view kSlamCorrection = "slam/correction";
inline constexpr std::string_view kSystemShutdown = "system/shutdown";

}  // namespace simpleslam::topic_names
