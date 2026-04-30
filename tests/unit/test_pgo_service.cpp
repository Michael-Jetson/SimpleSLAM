#include <SimpleSLAM/backend/pgo_service.hpp>
#include <SimpleSLAM/core/infra/topic_hub.hpp>
#include <SimpleSLAM/core/infra/topic_names.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace simpleslam;

namespace {

struct MockPGO {
    int odom_edge_count = 0;
    int loop_edge_count = 0;
    int optimize_count = 0;
    bool should_converge = true;

    void addOdometryEdge(uint64_t, uint64_t, const SE3d&, const Mat6d&) {
        ++odom_edge_count;
    }

    void addLoopEdge(uint64_t, uint64_t, const SE3d&, const Mat6d&) {
        ++loop_edge_count;
    }

    OptimizationResult optimize() {
        ++optimize_count;
        OptimizationResult result;
        result.converged = should_converge;
        result.iterations = 3;
        result.final_error = 0.005;
        if (should_converge) {
            result.optimized_poses[1] = SE3d{};
        }
        return result;
    }

    void reset() {}
};

KeyframeEvent makeKeyframeEvent(uint64_t id, double x) {
    KeyframeEvent evt;
    evt.keyframe_id = id;
    evt.timestamp = static_cast<double>(id);
    evt.pose = SE3d(Vec3d(x, 0.0, 0.0), Eigen::Quaterniond::Identity());
    evt.scan = std::make_shared<const LidarScan>();
    return evt;
}

LoopDetectedEvent makeLoopEvent(uint64_t query, uint64_t match) {
    LoopDetectedEvent evt;
    evt.query_keyframe_id = query;
    evt.match_keyframe_id = match;
    evt.T_match_query = SE3d{};
    evt.confidence = 0.9;
    return evt;
}

}  // namespace

TEST_CASE("PgoService initialize 创建 Topic", "[pgo_service]") {
    TopicHub hub(true);
    PgoService svc(AnyPoseGraphOptimizer(MockPGO{}));
    svc.initialize(hub);

    REQUIRE(hub.hasTopic(topic_names::kSlamCorrection));
}

TEST_CASE("PgoService 两个连续 keyframe 产生 odometry edge", "[pgo_service]") {
    TopicHub hub(true);
    MockPGO mock;
    PgoService svc(AnyPoseGraphOptimizer(std::move(mock)));
    svc.initialize(hub);

    auto kf_pub = hub.createPublisherImpl<KeyframeEvent>(
        topic_names::kSlamKeyframe);
    kf_pub.publish(makeKeyframeEvent(1, 0.0));
    kf_pub.publish(makeKeyframeEvent(2, 1.0));
    hub.drainAll();

    REQUIRE(true);
}

TEST_CASE("PgoService loop 事件触发 optimize 并发布 CorrectionEvent", "[pgo_service]") {
    TopicHub hub(true);
    PgoService svc(AnyPoseGraphOptimizer(MockPGO{}));
    svc.initialize(hub);

    int correction_count = 0;
    auto sub = hub.subscribeImpl<CorrectionEvent>(
        topic_names::kSlamCorrection,
        [&](MsgPtr<CorrectionEvent>) { ++correction_count; });

    auto kf_pub = hub.createPublisherImpl<KeyframeEvent>(
        topic_names::kSlamKeyframe);
    auto loop_pub = hub.createPublisherImpl<LoopDetectedEvent>(
        topic_names::kSlamLoop);

    kf_pub.publish(makeKeyframeEvent(1, 0.0));
    kf_pub.publish(makeKeyframeEvent(2, 1.0));
    hub.drainAll();

    loop_pub.publish(makeLoopEvent(2, 1));
    hub.drainAll();

    REQUIRE(correction_count == 1);
}

TEST_CASE("PgoService optimize 不收敛时无 CorrectionEvent", "[pgo_service]") {
    TopicHub hub(true);
    MockPGO mock;
    mock.should_converge = false;
    PgoService svc(AnyPoseGraphOptimizer(std::move(mock)));
    svc.initialize(hub);

    int correction_count = 0;
    auto sub = hub.subscribeImpl<CorrectionEvent>(
        topic_names::kSlamCorrection,
        [&](MsgPtr<CorrectionEvent>) { ++correction_count; });

    auto loop_pub = hub.createPublisherImpl<LoopDetectedEvent>(
        topic_names::kSlamLoop);
    loop_pub.publish(makeLoopEvent(2, 1));
    hub.drainAll();

    REQUIRE(correction_count == 0);
}

TEST_CASE("PgoService shutdown 退订", "[pgo_service]") {
    TopicHub hub(true);
    PgoService svc(AnyPoseGraphOptimizer(MockPGO{}));
    svc.initialize(hub);
    svc.shutdown();

    REQUIRE(svc.name() == "PgoService");
}
