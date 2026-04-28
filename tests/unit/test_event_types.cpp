#include <SimpleSLAM/core/types/event_types.hpp>
#include <SimpleSLAM/core/types/keyframe.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace simpleslam;

TEST_CASE("KeyframeEvent 构造", "[event_types]") {
    KeyframeEvent event;
    event.keyframe_id = 42;
    event.timestamp = 1.0;
    event.pose = SE3d{};
    REQUIRE(event.keyframe_id == 42);
}

TEST_CASE("LoopDetectedEvent 构造", "[event_types]") {
    LoopDetectedEvent event;
    event.query_keyframe_id = 10;
    event.match_keyframe_id = 5;
    event.confidence = 0.95;
    REQUIRE(event.confidence == 0.95);
}

TEST_CASE("CorrectionEvent 携带多个位姿", "[event_types]") {
    CorrectionEvent event;
    event.corrected_poses[1] = SE3d{};
    event.corrected_poses[2] = SE3d{};
    REQUIRE(event.corrected_poses.size() == 2);
}

TEST_CASE("ShutdownEvent 带原因", "[event_types]") {
    ShutdownEvent event{"user request"};
    REQUIRE(event.reason == "user request");
}

TEST_CASE("KeyframeData extras 扩展", "[keyframe]") {
    KeyframeData kf;
    kf.id = 1;
    kf.extras["descriptor"] = std::vector<uint8_t>{1, 2, 3};
    REQUIRE(kf.hasExtra("descriptor"));
    auto* desc = kf.tryGetExtra<std::vector<uint8_t>>("descriptor");
    REQUIRE(desc != nullptr);
    REQUIRE(desc->size() == 3);
}
