// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file voxel_point_map.hpp
/// 体素点地图：体素哈希存原始点，点到点残差。对标 KISS-ICP 的 VoxelHashMap。
///
/// - **查询**：每个点在所在体素及 26 个相邻体素里找最近的地图点，残差 r = p − q，
///   雅可比 J = [I | −[p]×]（左扰动、平移在前）。每个点最多比较 27 × 每体素上限次，
///   与地图总点数无关。
/// - **插入**：离传感器超过 `max_range_m` 的点不插入；体素满了、或与体素里已有的点
///   太近（间距平方小于 体素边长² / 每体素上限），就跳过，体素里的点因此近似均匀。
/// - **维护**：删掉离中心超过 `max_distance_m` 的体素（按体素里第一个点算）。

#include <SimpleSLAM/algo/map/map.hpp>
#include <SimpleSLAM/algo/voxel_key.hpp>

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace simpleslam::algo {

class VoxelPointMap final : public LinearizableMap {
public:
    static constexpr std::string_view kTypeName = "voxel_point_map";

    struct Config {
        double voxel_size_m = 1.0;
        std::int64_t max_points_per_voxel = 20;
        double max_range_m = 100.0;
        double max_distance_m = 100.0;

        template <class V>
        void visit(V& v) {
            v("voxel_size_m", voxel_size_m, "体素边长（米）");
            v("max_points_per_voxel", max_points_per_voxel,
              "每个体素最多存几个点，决定查询代价的上界");
            v("max_range_m", max_range_m, "离传感器超过这个距离（米）的点不插入");
            v("max_distance_m", max_distance_m, "维护时删掉离中心超过这个距离（米）的体素");
        }
    };

    /// 体素边长、距离必须是正的有限数，每体素上限至少为 1，否则抛 `std::invalid_argument`。
    explicit VoxelPointMap(const Config& config);

    [[nodiscard]] bool empty() const noexcept override { return points_ == 0; }
    [[nodiscard]] std::size_t size() const noexcept override { return points_; }
    [[nodiscard]] std::string_view typeName() const noexcept override { return kTypeName; }
    void clear() override;

private:
    [[nodiscard]] LinearizedSystem doLinearize(const fw::PointCloud& points,
                                               const SE3d& T_map_sensor,
                                               const MatchOptions& options) const override;
    void doInsert(const fw::PointCloud& points, const SE3d& T_map_sensor) override;
    void doMaintain(const Vec3d& center) override;

    /// 27 邻域里离 p 最近的地图点；邻域里没有点就返回空。
    [[nodiscard]] const Vec3d* nearest(const Vec3d& p) const;

    Config config_;
    double inv_voxel_size_ = 0.0;
    double min_spacing2_ = 0.0;
    std::unordered_map<VoxelKey, std::vector<Vec3d>, VoxelKeyHash> voxels_;
    std::size_t points_ = 0;
};

}  // namespace simpleslam::algo
