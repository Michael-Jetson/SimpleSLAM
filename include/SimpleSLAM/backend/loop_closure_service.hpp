#pragma once

/// @file loop_closure_service.hpp
/// 回环检测后端服务——订阅关键帧事件，调用检测器，发布回环结果
///
/// 纯框架骨架，不含检测算法。通过 AnyLoopDetector 类型擦除持有
/// 任意满足 LoopDetector concept 的实现（ScanContext、STD 等）。
///
/// 数据流：kSlamKeyframe → detect → kSlamLoop

#include <SimpleSLAM/backend/service_base.hpp>
#include <SimpleSLAM/core/concepts/any_loop_detector.hpp>
#include <SimpleSLAM/core/infra/comm/comm_config.hpp>
#include <SimpleSLAM/core/infra/comm/topic.hpp>
#include <SimpleSLAM/core/infra/comm/topic_names.hpp>
#include <SimpleSLAM/core/types/event_types.hpp>

#include <utility>

namespace simpleslam {

class LoopClosureService final : public ServiceBase {
public:
    explicit LoopClosureService(AnyLoopDetector detector, const Config& cfg = {})
        : ServiceBase("LoopClosureService"), detector_(std::move(detector)) {
        detector_.setOwner(name());
        if (cfg.has("loop")) loop_spec_ = loadTopicSpec(cfg, "loop");
        if (cfg.has("keyframe")) keyframe_spec_ = loadTopicSpec(cfg, "keyframe");
    }

    void initialize(TopicHub& hub) override {
        loop_pub_ = hub.createPublisherImpl<LoopDetectedEvent>(
            loop_spec_.name, loop_spec_.qos);
        keyframe_sub_ = hub.subscribeImpl(
            keyframe_spec_.name, &LoopClosureService::onKeyframe, this,
            keyframe_spec_.options);
    }

    void shutdown() override {
        keyframe_sub_.reset();
    }

private:
    void onKeyframe(const KeyframeEvent& event) {
        auto kf = event.toKeyframeData();
        detector_.addKeyframe(kf);

        for (const auto& candidate : detector_.detect(kf)) {
            loop_pub_.publish(LoopDetectedEvent{
                kf.id, candidate.match_keyframe_id,
                candidate.T_match_query, candidate.score});
        }
    }

    AnyLoopDetector detector_;
    Publisher<LoopDetectedEvent> loop_pub_;
    SubscriptionHandle keyframe_sub_;

    TopicSpec loop_spec_{std::string(topic_names::kSlamLoop), QoS::Event, {}};
    TopicSpec keyframe_spec_{std::string(topic_names::kSlamKeyframe), QoS::Event, {}};
};

}  // namespace simpleslam
