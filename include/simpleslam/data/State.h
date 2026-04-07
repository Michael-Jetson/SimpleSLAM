#pragma once
#include <Eigen/Core>
#include <simpleslam/core/Types.h>

namespace SimpleSLAM {
// 18-dimensional error state (FAST-LIO2 style)
// 18 = 3(rotation) + 3(position) + 3(velocity) + 3(bias_accel) + 3(bias_gyro) + 3(gravity)
struct State {
    double timestamp = 0.0;
    SE3 pose;                                    // T_world_body (ENU, Hamilton wxyz)
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81); // FAST-LIO2: online estimated
    Eigen::Matrix<double, 18, 18> covariance = Eigen::Matrix<double, 18, 18>::Identity();
};
} // namespace SimpleSLAM
