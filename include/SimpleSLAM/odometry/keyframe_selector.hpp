#pragma once

/// @file keyframe_selector.hpp
/// 关键帧选择策略——基于距离/角度/时间阈值的判定器
///
/// 纯策略组件，不含算法逻辑。shouldSelect 是纯查询（不修改状态），
/// update 才提交当前帧为关键帧。这允许调用者在配准成功后再确认。

#include <SimpleSLAM/core/types/common.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>

#include <cmath>
#include <optional>

namespace simpleslam {

/// 关键帧判定阈值配置
struct KeyframeCriteria {
    double min_distance = 1.0;             ///< 最小平移距离（米）
    double min_angle_deg = 10.0;           ///< 最小旋转角度（度）
    double max_interval = 5.0;             ///< 最大时间间隔（秒）
    bool first_frame_is_keyframe = true;   ///< 首帧是否自动选为关键帧
};

/// 关键帧选择器——shouldSelect 查询 + update 提交模式
class KeyframeSelector final {
public:
    explicit KeyframeSelector(const KeyframeCriteria& criteria = {})
        : criteria_(criteria) {}

    /// 判断当前帧是否应选为关键帧（不修改内部状态）
    [[nodiscard]] bool shouldSelect(const SE3d& current_pose,
                                     Timestamp current_time) const {
        if (!last_pose_.has_value()) {
            return criteria_.first_frame_is_keyframe;
        }

        if (current_time - *last_time_ >= criteria_.max_interval) {
            return true;
        }

        SE3d delta = last_pose_->inverse() * current_pose;

        double translation_distance = delta.translation().norm();
        if (translation_distance >= criteria_.min_distance) {
            return true;
        }

        Eigen::AngleAxisd aa(delta.rotation());
        double rotation_angle_deg = aa.angle() * 180.0 / M_PI;
        if (rotation_angle_deg >= criteria_.min_angle_deg) {
            return true;
        }

        return false;
    }

    /// 确认当前帧为关键帧，更新内部基准
    void update(const SE3d& pose, Timestamp time) {
        last_pose_ = pose;
        last_time_ = time;
    }

    /// 重置为初始状态（下一帧视为首帧）
    void reset() {
        last_pose_.reset();
        last_time_.reset();
    }

    [[nodiscard]] const KeyframeCriteria& criteria() const { return criteria_; }

private:
    KeyframeCriteria criteria_;
    std::optional<SE3d> last_pose_;
    std::optional<Timestamp> last_time_;
};

}  // namespace simpleslam
