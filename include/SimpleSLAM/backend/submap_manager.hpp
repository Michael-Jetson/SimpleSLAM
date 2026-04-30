#pragma once

/// @file submap_manager.hpp
/// 子图生命周期管理——创建、追加关键帧、冻结、应用 PGO 校正
///
/// 被动数据管理器，不订阅 Topic。与 KeyframeStore/PoseGraph 同类：
/// 由 Runner 或后端服务在适当时机调用。
///
/// 冻结策略：活跃子图关键帧数达到上限时应冻结并创建新子图。
/// PGO 校正只更新锚点位姿 T_world_submap（FUTURE_ONLY 策略）。

#include <SimpleSLAM/backend/submap/submap.hpp>

#include <cassert>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace simpleslam {

struct SubMapManagerConfig {
    size_t max_keyframes_per_submap = 50;
};

class SubMapManager final {
public:
    explicit SubMapManager(const SubMapManagerConfig& config = {})
        : config_(config) {}

    uint64_t createSubmap(const SE3d& anchor_pose) {
        Submap submap;
        submap.id = next_id_++;
        submap.T_world_submap = anchor_pose;
        submaps_.push_back(std::move(submap));
        active_index_ = submaps_.size() - 1;
        return submaps_.back().id;
    }

    void addKeyframe(uint64_t keyframe_id) {
        assert(active_index_.has_value());
        submaps_[*active_index_].keyframe_ids.push_back(keyframe_id);
    }

    [[nodiscard]] bool shouldFreezeActive() const {
        if (!active_index_) return false;
        return submaps_[*active_index_].keyframeCount() >= config_.max_keyframes_per_submap;
    }

    uint64_t freezeAndCreateNew(const SE3d& new_anchor) {
        assert(active_index_.has_value());
        submaps_[*active_index_].frozen = true;
        return createSubmap(new_anchor);
    }

    [[nodiscard]] std::optional<uint64_t> activeSubmapId() const {
        if (!active_index_) return std::nullopt;
        return submaps_[*active_index_].id;
    }

    [[nodiscard]] const Submap* activeSubmap() const {
        if (!active_index_) return nullptr;
        return &submaps_[*active_index_];
    }

    [[nodiscard]] const Submap* getSubmap(uint64_t id) const {
        for (const auto& s : submaps_) {
            if (s.id == id) return &s;
        }
        return nullptr;
    }

    [[nodiscard]] const std::vector<Submap>& submaps() const { return submaps_; }

    void applyCorrections(const std::unordered_map<uint64_t, SE3d>& corrections) {
        for (auto& submap : submaps_) {
            auto it = corrections.find(submap.id);
            if (it != corrections.end()) {
                submap.T_world_submap = it->second;
            }
        }
    }

    [[nodiscard]] size_t submapCount() const { return submaps_.size(); }

private:
    SubMapManagerConfig config_;
    std::vector<Submap> submaps_;
    uint64_t next_id_{0};
    std::optional<size_t> active_index_;
};

}  // namespace simpleslam
