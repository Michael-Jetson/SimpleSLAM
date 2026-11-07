// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file scenes.hpp
/// 契约测试用的合成几何：互相垂直的平面块、共面与极稀疏的点云，以及位姿的小工具。
/// 只依赖框架的数据类型、Eigen 与 manif，任何实现的测试都能用。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/sensor_data.hpp>

#include <Eigen/Geometry>
#include <manif/SE3.h>

#include <cstddef>

namespace simpleslam::testing {

using Vec6d = Eigen::Matrix<double, 6, 1>;

/// 切向量：平移在前、旋转在后，与 manif 一致。
inline Vec6d twist(double tx, double ty, double tz, double rx, double ry, double rz) {
    return (Vec6d() << tx, ty, tz, rx, ry, rz).finished();
}

/// 左乘更新：`Exp(δ)·T`，与配准地图的扰动约定一致。
inline SE3d expLeft(const Vec6d& delta, const SE3d& T) {
    return manif::SE3Tangentd(delta).exp() * T;
}

/// 三块互相垂直、互不相交的平面：地面 z = 0、墙 x = 0、墙 y = 0，各占 [1, 4) 米。
///
/// 三个法向约束全部 6 个自由度。平面互不相交，每个 1 米的体素里只有一块平面；间距
/// 0.25 米时两点间距平方是 0.0625。于是在真值处每个查询点都有零残差的对应，高斯牛顿
/// 的不动点就是真值，恢复可以按严格的容差检查。
inline fw::PointCloud planePatches(double spacing = 0.25) {
    fw::PointCloud c;
    const int n = static_cast<int>(3.0 / spacing);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const auto a = static_cast<float>(1.0 + i * spacing);
            const auto b = static_cast<float>(1.0 + j * spacing);
            c.push(a, b, 0.0F);  // 地面
            c.push(0.0F, a, b);  // 墙 x = 0
            c.push(a, 0.0F, b);  // 墙 y = 0
        }
    }
    return c;
}

/// 共面：只有地面 z = 0 一块平面，所有法向平行（走廊、隧道的极端情形），
/// 只约束三个自由度。
inline fw::PointCloud coplanar(double spacing = 0.25) {
    fw::PointCloud c;
    const int n = static_cast<int>(3.0 / spacing);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            c.push(static_cast<float>(1.0 + i * spacing), static_cast<float>(1.0 + j * spacing),
                   0.0F);
        }
    }
    return c;
}

/// 极稀疏：几个散开的点，少于 10 个。
inline fw::PointCloud sparse(std::size_t n = 5) {
    fw::PointCloud c;
    for (std::size_t i = 0; i < n; ++i) {
        const auto k = static_cast<float>(i);
        c.push(1.0F + k, 2.0F - 0.5F * k, 0.25F * k);
    }
    return c;
}

/// 把点云整体变到另一个坐标系：`p' = T · p`。
inline fw::PointCloud transformed(const fw::PointCloud& in, const SE3d& T) {
    fw::PointCloud out;
    out.reserve(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        const Vec3d p = T.act(Vec3d(in.x[i], in.y[i], in.z[i]));
        out.push(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    }
    return out;
}

inline double translationError(const SE3d& a, const SE3d& b) {
    return (a.translation() - b.translation()).norm();
}

inline double rotationErrorRad(const SE3d& a, const SE3d& b) {
    return Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle();
}

}  // namespace simpleslam::testing
