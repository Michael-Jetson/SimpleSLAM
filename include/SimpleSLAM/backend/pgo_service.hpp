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
#include <SimpleSLAM/core/infra/comm/comm_config.hpp>
#include <SimpleSLAM/core/infra/comm/topic.hpp>
#include <SimpleSLAM/core/infra/comm/topic_names.hpp>
#include <SimpleSLAM/core/types/event_types.hpp>

#include <optional>
#include <utility>

namespace simpleslam {

class PgoService final : public ServiceBase {
public:
    explicit PgoService(AnyPoseGraphOptimizer optimizer, const Config& cfg = {})
        : ServiceBase("PgoService"), optimizer_(std::move(optimizer)) {
        optimizer_.setOwner(name());
        if (cfg.has("correction")) correction_spec_ = loadTopicSpec(cfg, "correction");
        if (cfg.has("keyframe")) keyframe_spec_ = loadTopicSpec(cfg, "keyframe");
        if (cfg.has("loop")) loop_spec_ = loadTopicSpec(cfg, "loop");
    }

    void initialize(TopicHub& hub) override {
        correction_pub_ = hub.createPublisher<CorrectionEvent>(
            correction_spec_.name, correction_spec_.qos);
        keyframe_sub_ = hub.subscribe(
            keyframe_spec_.name, &PgoService::onKeyframe, this, keyframe_spec_.options);
        loop_sub_ = hub.subscribe(
            loop_spec_.name, &PgoService::onLoop, this, loop_spec_.options);
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
            // epoch 暂留默认 0：PgoService 是脚手架，无 canonical PoseGraph 版本源
            // （真实 epoch 由 R3 后端底物设置）。消费端 SubMapManager 已按 epoch 拒过期。
            correction_pub_.publish(CorrectionEvent{
                .level = CorrectionEvent::Level::OffsetAndRebuild,
                .corrected_poses = result.optimized_poses});
        }
    }

    AnyPoseGraphOptimizer optimizer_;
    Publisher<CorrectionEvent> correction_pub_;
    SubscriptionHandle keyframe_sub_;
    SubscriptionHandle loop_sub_;

    TopicSpec correction_spec_{std::string(topic_names::kSlamCorrection), QoS::Event, {}};
    TopicSpec keyframe_spec_{std::string(topic_names::kSlamKeyframe), QoS::Event, {}};
    TopicSpec loop_spec_{std::string(topic_names::kSlamLoop), QoS::Event, {}};

    std::optional<uint64_t> last_keyframe_id_;
    std::optional<SE3d> last_keyframe_pose_;
};

}  // namespace simpleslam
