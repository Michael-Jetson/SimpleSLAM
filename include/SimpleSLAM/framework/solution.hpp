// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file solution.hpp
/// 解快照：身份与数据绑在同一个不可变对象上。
///
/// 起因：校正事件只带 SolutionIdentity，消费者校验完身份
/// 再去读"当前最新图"。并发运行时，事件属于解 S1，读到的图可能已经是 S2，
/// 最终把 S1 的身份写进当前状态。给通知加版本字段解决不了这个问题——
/// 只要"身份"和"数据"是分两次取的，它们之间就有一个可以插入新解的窗口。
///
/// 所以校正事件携带解本身：一个不可变的位姿图快照，它自己带着身份。
/// 消费者读到的身份与数据必然同源，不需要再做一致性判断。

#include <SimpleSLAM/framework/keyframe.hpp>
#include <SimpleSLAM/framework/pose_graph.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <memory>
#include <optional>
#include <utility>

namespace simpleslam::fw {

/// 不可变的解快照。内容永不变，可以跨模块随意传递。
using PoseGraphSnapshot = std::shared_ptr<const PoseGraphData>;

/// 位姿图产生了新解。事件自带那份解，不是"去读最新图"的指令。
struct CorrectionEvent {
    Timestamp time{0};
    PoseGraphSnapshot solution;

    /// 解的身份。取自快照自身，因此与数据必然同源。
    [[nodiscard]] SolutionIdentity identity() const {
        return solution ? solution->identity : SolutionIdentity{};
    }
    [[nodiscard]] bool valid() const noexcept { return static_cast<bool>(solution); }
};

/// 取指定里程计段最新的已校正关键帧，返回 (T_world_body, T_odom_body)。
///
/// 只认同时出现在位姿图与关键帧库里的那一帧：图里有而库里没有（或反之）说明
/// 两者还没对齐到同一个时点，此时推不出可信的 T_world_odom。
/// 同一张图可以包含多个段；旧段锚点不能给新段定义 T_world_odom。
[[nodiscard]] inline std::optional<std::pair<SE3d, SE3d>> newestCorrectedAnchor(
    const PoseGraphData& graph, const KeyframeStoreData& store, std::uint64_t odometry_segment) {
    for (std::size_t i = graph.nodes.size(); i > 0; --i) {
        const auto& node = graph.nodes.at(i - 1);
        if (auto frame = store.find(node.id);
            frame && frame->odometry_segment == odometry_segment) {
            return std::make_pair(node.T_world_body, frame->T_odom_body);
        }
    }
    return std::nullopt;
}

/// 由锚点得到里程计系到世界系的变换：T_world_odom = T_world_body ∘ T_odom_body⁻¹。
[[nodiscard]] inline std::optional<SE3d> worldFromOdom(const PoseGraphData& graph,
                                                       const KeyframeStoreData& store,
                                                       std::uint64_t odometry_segment) {
    const auto anchor = newestCorrectedAnchor(graph, store, odometry_segment);
    if (!anchor) return std::nullopt;
    return anchor->first.compose(anchor->second.inverse());
}

}  // namespace simpleslam::fw
