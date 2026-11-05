// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/algo/filter/scan_filter.hpp>
#include <SimpleSLAM/algo/voxel_key.hpp>

#include <cstddef>
#include <format>
#include <stdexcept>
#include <unordered_set>

#include "cloud_checks.hpp"

namespace simpleslam::algo {

namespace {

/// 把第 i 个点（含可选列）追加到 out。
void copyPoint(const fw::PointCloud& in, std::size_t i, fw::PointCloud& out) {
    out.push(in.x[i], in.y[i], in.z[i]);
    if (!in.intensity.empty()) out.intensity.push_back(in.intensity[i]);
    if (!in.time_offset_ns.empty()) out.time_offset_ns.push_back(in.time_offset_ns[i]);
    if (!in.ring.empty()) out.ring.push_back(in.ring[i]);
}

}  // namespace

fw::PointCloud ScanFilter::apply(const fw::PointCloud& in) const {
    if (!detail::wellFormed(in)) {
        throw std::invalid_argument(
            std::format("{}：输入点云各列长度不一致，或有非有限的坐标", typeName()));
    }
    fw::PointCloud out = doApply(in);
    if (!detail::wellFormed(out)) {
        throw std::logic_error(
            std::format("{}：输出点云不合法（列长度不一致或坐标非有限）", typeName()));
    }
    return out;
}

VoxelDownsample::VoxelDownsample(const Config& config) : config_(config) {
    if (!detail::positiveFinite(config_.voxel_size_m)) {
        throw std::invalid_argument(
            std::format("voxel_size_m 必须是正的有限数，实际是 {}", config_.voxel_size_m));
    }
}

fw::PointCloud VoxelDownsample::doApply(const fw::PointCloud& in) const {
    fw::PointCloud out;
    out.reserve(in.size());
    std::unordered_set<VoxelKey, VoxelKeyHash> seen;
    seen.reserve(in.size());
    const double inv = 1.0 / config_.voxel_size_m;
    for (std::size_t i = 0; i < in.size(); ++i) {
        if (seen.insert(VoxelKey::of(detail::pointAt(in, i), inv)).second) {
            copyPoint(in, i, out);
        }
    }
    return out;
}

}  // namespace simpleslam::algo
