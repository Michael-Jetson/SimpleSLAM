#include <SimpleSLAM/odometry/keyframe_selector.hpp>

#include <catch2/catch_test_macros.hpp>
#include <cmath>

using namespace simpleslam;

namespace {

SE3d makePose(double x, double y, double z) {
    return SE3d(Eigen::Vector3d(x, y, z), Eigen::Quaterniond::Identity());
}

SE3d makeRotatedPose(double angle_deg) {
    double angle_rad = angle_deg * M_PI / 180.0;
    Eigen::Quaterniond q(Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ()));
    return SE3d(Eigen::Vector3d::Zero(), q);
}

}  // namespace

TEST_CASE("KeyframeSelector 首帧是关键帧", "[keyframe_selector]") {
    KeyframeSelector selector;
    SE3d pose = makePose(0, 0, 0);
    REQUIRE(selector.shouldSelect(pose, 0.0));
}

TEST_CASE("KeyframeSelector first_frame_is_keyframe=false", "[keyframe_selector]") {
    KeyframeCriteria criteria;
    criteria.first_frame_is_keyframe = false;
    KeyframeSelector selector(criteria);
    REQUIRE_FALSE(selector.shouldSelect(makePose(0, 0, 0), 0.0));
}

TEST_CASE("KeyframeSelector 距离阈值", "[keyframe_selector]") {
    KeyframeCriteria criteria;
    criteria.min_distance = 2.0;
    criteria.min_angle_deg = 90.0;
    criteria.max_interval = 100.0;
    KeyframeSelector selector(criteria);

    SE3d origin = makePose(0, 0, 0);
    selector.update(origin, 0.0);

    REQUIRE_FALSE(selector.shouldSelect(makePose(1.0, 0, 0), 1.0));
    REQUIRE(selector.shouldSelect(makePose(2.0, 0, 0), 1.0));
    REQUIRE(selector.shouldSelect(makePose(3.0, 0, 0), 1.0));
}

TEST_CASE("KeyframeSelector 角度阈值", "[keyframe_selector]") {
    KeyframeCriteria criteria;
    criteria.min_distance = 100.0;
    criteria.min_angle_deg = 15.0;
    criteria.max_interval = 100.0;
    KeyframeSelector selector(criteria);

    selector.update(makePose(0, 0, 0), 0.0);

    REQUIRE_FALSE(selector.shouldSelect(makeRotatedPose(10.0), 1.0));
    REQUIRE(selector.shouldSelect(makeRotatedPose(20.0), 1.0));
}

TEST_CASE("KeyframeSelector 时间阈值", "[keyframe_selector]") {
    KeyframeCriteria criteria;
    criteria.min_distance = 100.0;
    criteria.min_angle_deg = 90.0;
    criteria.max_interval = 3.0;
    KeyframeSelector selector(criteria);

    SE3d pose = makePose(0, 0, 0);
    selector.update(pose, 0.0);

    REQUIRE_FALSE(selector.shouldSelect(pose, 2.0));
    REQUIRE(selector.shouldSelect(pose, 3.0));
    REQUIRE(selector.shouldSelect(pose, 5.0));
}

TEST_CASE("KeyframeSelector 在阈值内返回 false", "[keyframe_selector]") {
    KeyframeCriteria criteria;
    criteria.min_distance = 5.0;
    criteria.min_angle_deg = 30.0;
    criteria.max_interval = 10.0;
    KeyframeSelector selector(criteria);

    selector.update(makePose(0, 0, 0), 0.0);

    REQUIRE_FALSE(selector.shouldSelect(makePose(0.1, 0, 0), 1.0));
}

TEST_CASE("KeyframeSelector shouldSelect 不修改状态", "[keyframe_selector]") {
    KeyframeSelector selector;

    SE3d pose = makePose(0, 0, 0);
    selector.update(pose, 0.0);

    SE3d far_pose = makePose(10, 0, 0);
    REQUIRE(selector.shouldSelect(far_pose, 1.0));
    REQUIRE(selector.shouldSelect(far_pose, 1.0));
}

TEST_CASE("KeyframeSelector reset 恢复初始状态", "[keyframe_selector]") {
    KeyframeSelector selector;

    selector.update(makePose(0, 0, 0), 0.0);
    REQUIRE_FALSE(selector.shouldSelect(makePose(0.1, 0, 0), 0.5));

    selector.reset();
    REQUIRE(selector.shouldSelect(makePose(0.1, 0, 0), 0.5));
}
