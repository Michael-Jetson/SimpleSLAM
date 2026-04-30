#include <SimpleSLAM/backend/loop_closure_service.hpp>
#include <SimpleSLAM/core/infra/topic_hub.hpp>
#include <SimpleSLAM/core/infra/topic_names.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace simpleslam;

namespace {

struct AlwaysDetectMock {
    int add_count = 0;

    void addKeyframe(const KeyframeData&) { ++add_count; }

    std::optional<LoopCandidate> detect(const KeyframeData&) {
        LoopCandidate c;
        c.match_keyframe_id = 99;
        c.T_match_query = SE3d{};
        c.score = 0.8;
        return c;
    }
};

struct NeverDetectMock {
    void addKeyframe(const KeyframeData&) {}
    std::optional<LoopCandidate> detect(const KeyframeData&) { return std::nullopt; }
};

KeyframeEvent makeKeyframeEvent(uint64_t id, Timestamp ts) {
    KeyframeEvent evt;
    evt.keyframe_id = id;
    evt.timestamp = ts;
    evt.pose = SE3d{};
    evt.scan = std::make_shared<const LidarScan>();
    return evt;
}

}  // namespace

TEST_CASE("LoopClosureService initialize 创建 Topic", "[loop_closure_service]") {
    TopicHub hub(true);
    LoopClosureService svc(AnyLoopDetector(AlwaysDetectMock{}));
    svc.initialize(hub);

    REQUIRE(hub.hasTopic(topic_names::kSlamLoop));
}

TEST_CASE("LoopClosureService AlwaysDetect 发布 LoopDetectedEvent", "[loop_closure_service]") {
    TopicHub hub(true);
    LoopClosureService svc(AnyLoopDetector(AlwaysDetectMock{}));
    svc.initialize(hub);

    int loop_count = 0;
    uint64_t match_id = 0;
    auto sub = hub.subscribeImpl<LoopDetectedEvent>(
        topic_names::kSlamLoop,
        [&](MsgPtr<LoopDetectedEvent> msg) {
            ++loop_count;
            match_id = msg->match_keyframe_id;
        });

    auto kf_pub = hub.createPublisherImpl<KeyframeEvent>(
        topic_names::kSlamKeyframe);
    kf_pub.publish(makeKeyframeEvent(1, 0.0));
    hub.drainAll();

    REQUIRE(loop_count == 1);
    REQUIRE(match_id == 99);
}

TEST_CASE("LoopClosureService NeverDetect 无 LoopDetectedEvent", "[loop_closure_service]") {
    TopicHub hub(true);
    LoopClosureService svc(AnyLoopDetector(NeverDetectMock{}));
    svc.initialize(hub);

    int loop_count = 0;
    auto sub = hub.subscribeImpl<LoopDetectedEvent>(
        topic_names::kSlamLoop,
        [&](MsgPtr<LoopDetectedEvent>) { ++loop_count; });

    auto kf_pub = hub.createPublisherImpl<KeyframeEvent>(
        topic_names::kSlamKeyframe);
    kf_pub.publish(makeKeyframeEvent(1, 0.0));
    hub.drainAll();

    REQUIRE(loop_count == 0);
}

TEST_CASE("LoopClosureService 多关键帧连续检测", "[loop_closure_service]") {
    TopicHub hub(true);
    LoopClosureService svc(AnyLoopDetector(AlwaysDetectMock{}));
    svc.initialize(hub);

    int loop_count = 0;
    auto sub = hub.subscribeImpl<LoopDetectedEvent>(
        topic_names::kSlamLoop,
        [&](MsgPtr<LoopDetectedEvent>) { ++loop_count; });

    auto kf_pub = hub.createPublisherImpl<KeyframeEvent>(
        topic_names::kSlamKeyframe);
    kf_pub.publish(makeKeyframeEvent(1, 0.0));
    kf_pub.publish(makeKeyframeEvent(2, 1.0));
    kf_pub.publish(makeKeyframeEvent(3, 2.0));
    hub.drainAll();

    REQUIRE(loop_count == 3);
}

TEST_CASE("LoopClosureService shutdown 退订", "[loop_closure_service]") {
    TopicHub hub(true);
    LoopClosureService svc(AnyLoopDetector(AlwaysDetectMock{}));
    svc.initialize(hub);
    svc.shutdown();

    REQUIRE(svc.name() == "LoopClosureService");
}
