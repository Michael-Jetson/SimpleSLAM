#pragma once

/// @file lie_utils.hpp
/// 李群/李代数工具——hat/vee、位姿扰动、点-位姿雅可比
///
/// 全部 inline，热路径零开销。
/// 切空间约定与 manif 一致：[tx ty tz rx ry rz]

#include <SimpleSLAM/core/types/geometry.hpp>

#include <Eigen/Core>

namespace simpleslam {

/// so(3) hat map: 3-vector -> 3x3 反对称矩阵
inline Mat3d hat(const Vec3d& v) {
    Mat3d m;
    m <<    0, -v.z(),  v.y(),
         v.z(),    0,  -v.x(),
        -v.y(),  v.x(),    0;
    return m;
}

/// so(3) vee map: 3x3 反对称矩阵 -> 3-vector
inline Vec3d vee(const Mat3d& m) {
    return {m(2, 1), m(0, 2), m(1, 0)};
}

/// 左扰动: T' = Exp(delta) * T
/// delta 是 se(3) 切向量 [tx ty tz rx ry rz]
inline SE3d perturbPose(const SE3d& T, const Eigen::Matrix<double, 6, 1>& delta) {
    return SE3d::Tangent(delta).exp() * T;
}

/// d(T.act(p)) / d(delta)  在 delta=0 处的雅可比
/// 其中 T' = Exp(delta) * T，p 是 body 系下的点
/// 返回 3x6 矩阵，列顺序 [d/dtx d/dty d/dtz d/drx d/dry d/drz]
inline Eigen::Matrix<double, 3, 6> pointToPoseJacobian(
    const SE3d& T, const Vec3d& p_body) {
    // p_world = R * p_body + t
    // d(p_world)/d(delta) = [ I | -hat(R * p_body) ]  （左扰动模型）
    const Vec3d p_world = T.act(p_body);
    Eigen::Matrix<double, 3, 6> J;
    J.leftCols<3>() = Mat3d::Identity();
    J.rightCols<3>() = -hat(p_world);
    return J;
}

}  // namespace simpleslam
