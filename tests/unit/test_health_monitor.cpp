#include <SimpleSLAM/core/infra/health_monitor.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace simpleslam;

namespace {

HealthMetrics goodFrame() {
    return {TrackingStatus::Tracking, 100, 1.0, 10.0};
}

HealthMetrics badFrame() {
    return {TrackingStatus::Degraded, 5, 50.0, 100.0};
}

HealthMetrics lostFrame() {
    return {TrackingStatus::Lost, 0, 200.0, 150.0};
}

HealthMetrics highCovarianceFrame() {
    return {TrackingStatus::Tracking, 100, 200.0, 10.0};
}

}  // namespace

TEST_CASE("HealthMonitor 初始状态 OK", "[health_monitor]") {
    HealthMonitor monitor;
    REQUIRE(monitor.state() == SystemHealth::OK);
}

TEST_CASE("HealthMonitor 连续坏帧达到退化阈值", "[health_monitor]") {
    HealthThresholds thresholds;
    thresholds.degraded_frames_threshold = 3;
    HealthMonitor monitor(thresholds);

    monitor.update(badFrame());
    monitor.update(badFrame());
    REQUIRE(monitor.state() == SystemHealth::OK);

    monitor.update(badFrame());
    REQUIRE(monitor.state() == SystemHealth::Degraded);
}

TEST_CASE("HealthMonitor 退化后恢复到 OK", "[health_monitor]") {
    HealthThresholds thresholds;
    thresholds.degraded_frames_threshold = 2;
    thresholds.recovery_frames = 2;
    HealthMonitor monitor(thresholds);

    monitor.update(badFrame());
    monitor.update(badFrame());
    REQUIRE(monitor.state() == SystemHealth::Degraded);

    monitor.update(goodFrame());
    monitor.update(goodFrame());
    REQUIRE(monitor.state() == SystemHealth::OK);
}

TEST_CASE("HealthMonitor 退化继续坏帧到 Lost", "[health_monitor]") {
    HealthThresholds thresholds;
    thresholds.degraded_frames_threshold = 2;
    thresholds.lost_frames_threshold = 5;
    HealthMonitor monitor(thresholds);

    for (int i = 0; i < 2; ++i) monitor.update(badFrame());
    REQUIRE(monitor.state() == SystemHealth::Degraded);

    for (int i = 0; i < 3; ++i) monitor.update(badFrame());
    REQUIRE(monitor.state() == SystemHealth::Lost);
}

TEST_CASE("HealthMonitor Lost 恢复到 Degraded（阶梯式）", "[health_monitor]") {
    HealthThresholds thresholds;
    thresholds.degraded_frames_threshold = 2;
    thresholds.lost_frames_threshold = 4;
    thresholds.recovery_frames = 2;
    HealthMonitor monitor(thresholds);

    for (int i = 0; i < 2; ++i) monitor.update(badFrame());
    REQUIRE(monitor.state() == SystemHealth::Degraded);

    for (int i = 0; i < 2; ++i) monitor.update(badFrame());
    REQUIRE(monitor.state() == SystemHealth::Lost);

    monitor.update(goodFrame());
    monitor.update(goodFrame());
    REQUIRE(monitor.state() == SystemHealth::Degraded);

    monitor.update(goodFrame());
    monitor.update(goodFrame());
    REQUIRE(monitor.state() == SystemHealth::OK);
}

TEST_CASE("HealthMonitor Failed 是终态", "[health_monitor]") {
    HealthThresholds thresholds;
    thresholds.degraded_frames_threshold = 1;
    thresholds.lost_frames_threshold = 2;
    HealthMonitor monitor(thresholds);

    monitor.update(badFrame());
    REQUIRE(monitor.state() == SystemHealth::Degraded);

    monitor.update(badFrame());
    REQUIRE(monitor.state() == SystemHealth::Lost);

    for (int i = 0; i < 100; ++i) monitor.update(lostFrame());
    REQUIRE(monitor.state() == SystemHealth::Failed);

    monitor.update(goodFrame());
    REQUIRE(monitor.state() == SystemHealth::Failed);
}

TEST_CASE("HealthMonitor reset 恢复初始", "[health_monitor]") {
    HealthThresholds thresholds;
    thresholds.degraded_frames_threshold = 1;
    thresholds.lost_frames_threshold = 2;
    HealthMonitor monitor(thresholds);

    monitor.update(badFrame());
    monitor.update(badFrame());
    for (int i = 0; i < 100; ++i) monitor.update(lostFrame());
    REQUIRE(monitor.state() == SystemHealth::Failed);

    monitor.reset();
    REQUIRE(monitor.state() == SystemHealth::OK);
    REQUIRE(monitor.consecutiveBadFrames() == 0);
    REQUIRE(monitor.consecutiveGoodFrames() == 0);
}

TEST_CASE("HealthMonitor covariance_trace 超阈值算坏帧", "[health_monitor]") {
    HealthThresholds thresholds;
    thresholds.degraded_frames_threshold = 2;
    thresholds.max_covariance_trace = 50.0;
    HealthMonitor monitor(thresholds);

    monitor.update(highCovarianceFrame());
    monitor.update(highCovarianceFrame());
    REQUIRE(monitor.state() == SystemHealth::Degraded);
}
