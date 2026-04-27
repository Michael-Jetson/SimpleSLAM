#pragma once

/// @file geometry.hpp
/// 几何类型别名——全框架统一使用 manif 李群表示
///
/// 公约（详见 convention.hpp）：
///   - 位姿语义：T_world_body（从 body 到 world 的变换）
///   - 四元数：Hamilton 约定（w,x,y,z），注意 Eigen coeffs() 返回 [x,y,z,w]
///   - 状态向量用 double，点坐标用 float

#include <Eigen/Core>
#include <manif/SE3.h>
#include <manif/SO3.h>

namespace simpleslam {

// ── 李群类型（manif 别名）──
using SE3d = manif::SE3d;  // 刚体变换（旋转 + 平移）
using SO3d = manif::SO3d;  // 纯旋转

// ── Eigen 矩阵/向量别名 ──
using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
using Mat6d = Eigen::Matrix<double, 6, 6>;

}  // namespace simpleslam
