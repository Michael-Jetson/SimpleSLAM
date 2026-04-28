#pragma once

/// @file submap.hpp
/// 子图被动数据容器——锚点位姿 + 冻结标志 + 关键帧关联
///
/// 这是纯数据结构，不持有 RegistrationTarget 实例。
/// 实际的地图结构由 SubMapManager（用户实现）管理。
///
/// FUTURE_ONLY 校正策略：PGO 只更新 T_world_submap，不修改内部点坐标。
/// 配准查询时自动做坐标变换：p_world = T_world_submap * p_local。

#include <SimpleSLAM/core/types/common.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace simpleslam {

struct Submap {
    uint64_t id{0};
    SE3d T_world_submap{};       ///< 锚点位姿（PGO 优化只修改此变换）

    bool frozen{false};           ///< 冻结后只读，不再接受新点

    std::vector<uint64_t> keyframe_ids;  ///< 属于此子图的关键帧 ID

    size_t estimated_memory_bytes{0};    ///< 内存占用估计（PageBudgetManager 用）
    Timestamp created_at{0.0};
    Timestamp frozen_at{0.0};            ///< 冻结时间，0 表示未冻结

    [[nodiscard]] size_t keyframeCount() const { return keyframe_ids.size(); }
    [[nodiscard]] bool isEmpty() const { return keyframe_ids.empty(); }
    [[nodiscard]] bool isFrozen() const { return frozen; }
};

}  // namespace simpleslam
