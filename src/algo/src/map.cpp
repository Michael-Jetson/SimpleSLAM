// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/algo/map/map.hpp>

#include <cmath>
#include <format>
#include <stdexcept>
#include <string_view>

#include "cloud_checks.hpp"

namespace simpleslam::algo {

namespace {

void checkInput(const fw::PointCloud& points, const SE3d& pose, std::string_view who) {
    if (!detail::wellFormed(points)) {
        throw std::invalid_argument(
            std::format("{}：输入点云各列长度不一致，或有非有限的坐标", who));
    }
    if (!pose.coeffs().allFinite()) {
        throw std::invalid_argument(std::format("{}：位姿不是有限数", who));
    }
}

}  // namespace

double RobustKernel::weight(double e2) const noexcept {
    switch (kind) {
        case Kind::kNone:
            return 1.0;
        case Kind::kHuber: {
            const double e = std::sqrt(e2);
            return e <= scale ? 1.0 : scale / e;
        }
        case Kind::kGemanMcClure: {
            const double k = scale * scale;
            const double d = k + e2;
            return (k * k) / (d * d);
        }
    }
    return 1.0;
}

double RobustKernel::rho(double e2) const noexcept {
    switch (kind) {
        case Kind::kNone:
            return e2;
        case Kind::kHuber: {
            const double e = std::sqrt(e2);
            return e <= scale ? e2 : 2.0 * scale * e - scale * scale;
        }
        case Kind::kGemanMcClure: {
            const double k = scale * scale;
            return k * e2 / (k + e2);
        }
    }
    return e2;
}

LinearizedSystem MapQuery::linearize(const fw::PointCloud& points, const SE3d& T_map_sensor,
                                     const MatchOptions& options) const {
    checkInput(points, T_map_sensor, typeName());
    if (!detail::positiveFinite(options.max_correspondence_m)) {
        throw std::invalid_argument(
            std::format("{}：max_correspondence_m 必须是正的有限数，实际是 {}", typeName(),
                        options.max_correspondence_m));
    }
    const bool weighted = options.kernel.kind != RobustKernel::Kind::kNone;
    if (weighted && !detail::positiveFinite(options.kernel.scale)) {
        throw std::invalid_argument(std::format("{}：鲁棒核的 scale 必须是正的有限数，实际是 {}",
                                                typeName(), options.kernel.scale));
    }
    LinearizedSystem out = doLinearize(points, T_map_sensor, options);
    if (!out.H.allFinite() || !out.g.allFinite() || !std::isfinite(out.cost)) {
        throw std::logic_error(std::format("{}：线性化结果不是有限数", typeName()));
    }
    const bool all_zero = out.H.isZero(0.0) && out.g.isZero(0.0) && out.cost == 0.0;
    if (out.rows == 0 && !all_zero) {
        throw std::logic_error(
            std::format("{}：没有约束（0 行）时 H、g、cost 必须为零", typeName()));
    }
    return out;
}

void LinearizableMap::insert(const fw::PointCloud& points, const SE3d& T_map_sensor) {
    checkInput(points, T_map_sensor, typeName());
    doInsert(points, T_map_sensor);
}

void LinearizableMap::maintain(const Vec3d& center) {
    if (!center.allFinite()) {
        throw std::invalid_argument(std::format("{}：维护中心不是有限数", typeName()));
    }
    doMaintain(center);
}

}  // namespace simpleslam::algo
