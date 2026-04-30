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
#include <SimpleSLAM/core/infra/topic.hpp>
#include <SimpleSLAM/core/infra/topic_hub.hpp>
#include <SimpleSLAM/core/infra/topic_names.hpp>
#include <SimpleSLAM/core/types/event_types.hpp>

#include <utility>

namespace simpleslam {

class LoopClosureService final : public ServiceBase {
public:
    explicit LoopClosureService(AnyLoopDetector detector)
        : ServiceBase("LoopClosureService"), detector_(std::move(detector)) {
        detector_.setOwner(name());
    }

    void initialize(TopicHub& hub) override {
        loop_pub_ = hub.createPublisherImpl<LoopDetectedEvent>(topic_names::kSlamLoop);
        keyframe_sub_ = hub.subscribeImpl(
            topic_names::kSlamKeyframe, &LoopClosureService::onKeyframe, this);
    }

    void shutdown() override {
        keyframe_sub_.reset();
    }

private:
    void onKeyframe(const KeyframeEvent& event) {
        auto kf = event.toKeyframeData();
        detector_.addKeyframe(kf);

        auto candidate = detector_.detect(kf);
        if (candidate) {
            loop_pub_.publish(LoopDetectedEvent{
                kf.id, candidate->match_keyframe_id,
                candidate->T_match_query, candidate->score});
        }
    }

    AnyLoopDetector detector_;
    Publisher<LoopDetectedEvent> loop_pub_;
    SubscriptionHandle keyframe_sub_;
};

}  // namespace simpleslam
