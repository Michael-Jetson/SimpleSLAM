#pragma once

/// @file odometry.hpp
/// 里程计模块伞形头——一行引入全部里程计基础设施

#include <SimpleSLAM/odometry/odometry_base.hpp>
#include <SimpleSLAM/odometry/keyframe_selector.hpp>
#include <SimpleSLAM/odometry/imu_buffer.hpp>
#include <SimpleSLAM/odometry/voxel_hash_target.hpp>
#include <SimpleSLAM/odometry/lo_icp_odometry.hpp>
