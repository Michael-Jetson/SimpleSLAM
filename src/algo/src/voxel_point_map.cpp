// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/algo/map/voxel_point_map.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <stdexcept>
#include <string_view>

#include "cloud_checks.hpp"

namespace simpleslam::algo {

namespace {

void requirePositiveFinite(std::string_view name, double value) {
    if (!detail::positiveFinite(value)) {
        throw std::invalid_argument(std::format("{} 必须是正的有限数，实际是 {}", name, value));
    }
}

}  // namespace

VoxelPointMap::VoxelPointMap(const Config& config) : config_(config) {
    requirePositiveFinite("voxel_size_m", config_.voxel_size_m);
    requirePositiveFinite("max_range_m", config_.max_range_m);
    requirePositiveFinite("max_distance_m", config_.max_distance_m);
    const std::int64_t cap = config_.max_points_per_voxel;
    if (cap < 1 || cap > std::numeric_limits<std::int32_t>::max()) {
        throw std::invalid_argument(
            std::format("max_points_per_voxel 要在 1 到 2^31−1 之间，实际是 {}", cap));
    }
    inv_voxel_size_ = 1.0 / config_.voxel_size_m;
    min_spacing2_ = config_.voxel_size_m * config_.voxel_size_m / static_cast<double>(cap);
}

const Vec3d* VoxelPointMap::nearest(const Vec3d& p) const {
    const VoxelKey c = VoxelKey::of(p, inv_voxel_size_);
    const Vec3d* best = nullptr;
    double best_d2 = std::numeric_limits<double>::infinity();
    // 固定的遍历顺序加严格小于：距离相同时取先遇到的，结果只取决于地图内容
    for (std::int32_t dx = -1; dx <= 1; ++dx) {
        for (std::int32_t dy = -1; dy <= 1; ++dy) {
            for (std::int32_t dz = -1; dz <= 1; ++dz) {
                const auto it = voxels_.find(VoxelKey{.x = c.x + dx, .y = c.y + dy, .z = c.z + dz});
                if (it == voxels_.end()) continue;
                for (const Vec3d& q : it->second) {
                    const double d2 = (p - q).squaredNorm();
                    if (d2 < best_d2) {
                        best_d2 = d2;
                        best = &q;
                    }
                }
            }
        }
    }
    return best;
}

LinearizedSystem VoxelPointMap::doLinearize(const fw::PointCloud& points, const SE3d& T_map_sensor,
                                            const MatchOptions& options) const {
    LinearizedSystem out;
    if (voxels_.empty() || points.empty()) return out;

    const Mat3d R = T_map_sensor.rotation();
    const Vec3d t = T_map_sensor.translation();
    const double max_corr2 = options.max_correspondence_m * options.max_correspondence_m;

    Eigen::Matrix<double, 3, 6> J;
    J.leftCols<3>().setIdentity();
    for (std::size_t i = 0; i < points.size(); ++i) {
        const Vec3d p = R * detail::pointAt(points, i) + t;
        const Vec3d* q = nearest(p);
        if (q == nullptr) continue;
        const Vec3d r = p - *q;
        const double e2 = r.squaredNorm();
        if (e2 > max_corr2) continue;

        const double w = options.kernel.weight(e2);
        // 左扰动、平移在前：∂(Exp(δ)·p)/∂δ = [I | −[p]×]
        J.rightCols<3>() << 0.0, p.z(), -p.y(),  //
            -p.z(), 0.0, p.x(),                  //
            p.y(), -p.x(), 0.0;
        out.H.noalias() += w * J.transpose() * J;
        out.g.noalias() += w * J.transpose() * r;
        out.cost += 0.5 * options.kernel.rho(e2);
        out.rows += 3;
    }
    return out;
}

void VoxelPointMap::doInsert(const fw::PointCloud& points, const SE3d& T_map_sensor) {
    const Mat3d R = T_map_sensor.rotation();
    const Vec3d t = T_map_sensor.translation();
    const double max_range2 = config_.max_range_m * config_.max_range_m;
    const auto cap = static_cast<std::size_t>(config_.max_points_per_voxel);

    for (std::size_t i = 0; i < points.size(); ++i) {
        const Vec3d ps = detail::pointAt(points, i);
        if (ps.squaredNorm() > max_range2) continue;
        const Vec3d p = R * ps + t;
        auto& bucket = voxels_[VoxelKey::of(p, inv_voxel_size_)];
        if (bucket.size() >= cap) continue;
        const bool too_close = std::any_of(bucket.begin(), bucket.end(), [&](const Vec3d& q) {
            return (q - p).squaredNorm() < min_spacing2_;
        });
        if (too_close) continue;
        bucket.push_back(p);
        ++points_;
    }
}

void VoxelPointMap::doMaintain(const Vec3d& center) {
    const double max2 = config_.max_distance_m * config_.max_distance_m;
    for (auto it = voxels_.begin(); it != voxels_.end();) {
        if (it->second.empty() || (it->second.front() - center).squaredNorm() > max2) {
            points_ -= it->second.size();
            it = voxels_.erase(it);
        } else {
            ++it;
        }
    }
}

void VoxelPointMap::clear() {
    voxels_.clear();
    points_ = 0;
}

}  // namespace simpleslam::algo
