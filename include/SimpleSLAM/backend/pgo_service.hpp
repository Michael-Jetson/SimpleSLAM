#pragma once

/// @file pgo_service.hpp
/// 位姿图优化后端服务——跟踪里程计序列、处理回环、发布校正
///
/// 纯框架骨架，不含优化算法。通过 AnyPoseGraphOptimizer 类型擦除
/// 持有任意满足 PoseGraphOptimizer concept 的实现。
///
/// 数据流：kSlamKeyframe → addOdometryEdge
///         kSlamLoop → addLoopEdge → optimize → kSlamCorrection

#include <SimpleSLAM/backend/service_base.hpp>
#include <SimpleSLAM/core/concepts/any_pose_graph_optimizer.hpp>
#include <SimpleSLAM/core/infra/topic.hpp>
#include <SimpleSLAM/core/infra/topic_hub.hpp>
#include <SimpleSLAM/core/infra/topic_names.hpp>
#include <SimpleSLAM/core/types/event_types.hpp>

#include <optional>
#include <utility>

namespace simpleslam {

class PgoService final : public ServiceBase {
public:
    explicit PgoService(AnyPoseGraphOptimizer optimizer)
        : ServiceBase("PgoService"), optimizer_(std::move(optimizer)) {
        optimizer_.setOwner(name());
    }

    void initialize(TopicHub& hub) override {
        correction_pub_ = hub.createPublisherImpl<CorrectionEvent>(
            topic_names::kSlamCorrection);
        keyframe_sub_ = hub.subscribeImpl(
            topic_names::kSlamKeyframe, &PgoService::onKeyframe, this);
        loop_sub_ = hub.subscribeImpl(
            topic_names::kSlamLoop, &PgoService::onLoop, this);
    }

    void shutdown() override {
        keyframe_sub_.reset();
        loop_sub_.reset();
    }

private:
    void onKeyframe(const KeyframeEvent& event) {
        if (last_keyframe_id_) {
            SE3d relative = last_keyframe_pose_->inverse() * event.pose;
            optimizer_.addOdometryEdge(*last_keyframe_id_, event.keyframe_id,
                                       relative, Mat6d::Identity());
        }
        last_keyframe_id_ = event.keyframe_id;
        last_keyframe_pose_ = event.pose;
    }

    void onLoop(const LoopDetectedEvent& event) {
        optimizer_.addLoopEdge(event.query_keyframe_id, event.match_keyframe_id,
                               event.T_match_query, Mat6d::Identity());
        auto result = optimizer_.optimize();
        if (result.converged && !result.optimized_poses.empty()) {
            correction_pub_.publish(CorrectionEvent{result.optimized_poses});
        }
    }

    AnyPoseGraphOptimizer optimizer_;
    Publisher<CorrectionEvent> correction_pub_;
    SubscriptionHandle keyframe_sub_;
    SubscriptionHandle loop_sub_;

    std::optional<uint64_t> last_keyframe_id_;
    std::optional<SE3d> last_keyframe_pose_;
};

}  // namespace simpleslam
