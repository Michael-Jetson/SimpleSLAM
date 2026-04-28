#pragma once

/// @file gtsam_bridge.hpp
/// manif::SE3d ↔ gtsam::Pose3 类型转换 + 信息矩阵顺序转换
///
/// 仅在 SIMPLESLAM_HAS_GTSAM 定义时可用（CMake 选项 SIMPLESLAM_USE_GTSAM）。
///
/// 关键注意点：切空间顺序不同
///   manif:  [tx ty tz rx ry rz]
///   GTSAM:  [rx ry rz tx ty tz]
/// 信息矩阵传递时必须做顺序转换，否则协方差语义错误。

#ifdef SIMPLESLAM_HAS_GTSAM

#include <SimpleSLAM/core/types/geometry.hpp>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Core>

namespace simpleslam::adapters {

/// manif SE3d → GTSAM Pose3
inline gtsam::Pose3 toGtsam(const SE3d& pose) {
    const Eigen::Quaterniond q(pose.rotation());
    const auto& t = pose.translation();
    return gtsam::Pose3(
        gtsam::Rot3(q.w(), q.x(), q.y(), q.z()),
        gtsam::Point3(t.x(), t.y(), t.z()));
}

/// GTSAM Pose3 → manif SE3d
inline SE3d fromGtsam(const gtsam::Pose3& pose) {
    const auto q = pose.rotation().toQuaternion();
    const auto& t = pose.translation();
    return SE3d(
        Vec3d(t.x(), t.y(), t.z()),
        Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));
}

/// 信息矩阵顺序转换：manif [t,r] → GTSAM [r,t]
inline Mat6d toGtsamInfoOrder(const Mat6d& simpleslam_info) {
    // 交换前 3 行/列（平移）和后 3 行/列（旋转）
    Mat6d gtsam_info;
    gtsam_info.topLeftCorner<3, 3>() = simpleslam_info.bottomRightCorner<3, 3>();
    gtsam_info.topRightCorner<3, 3>() = simpleslam_info.bottomLeftCorner<3, 3>();
    gtsam_info.bottomLeftCorner<3, 3>() = simpleslam_info.topRightCorner<3, 3>();
    gtsam_info.bottomRightCorner<3, 3>() = simpleslam_info.topLeftCorner<3, 3>();
    return gtsam_info;
}

/// 信息矩阵顺序转换：GTSAM [r,t] → manif [t,r]
inline Mat6d fromGtsamInfoOrder(const Mat6d& gtsam_info) {
    return toGtsamInfoOrder(gtsam_info);  // 置换是对合的（自逆）
}

}  // namespace simpleslam::adapters

#endif  // SIMPLESLAM_HAS_GTSAM
