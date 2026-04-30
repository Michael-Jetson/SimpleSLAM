#include <SimpleSLAM/odometry/lo_icp_odometry.hpp>
#include <SimpleSLAM/core/infra/topic_hub.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace simpleslam;

namespace {

LidarScan makeCubeScan(double offset_x = 0.0, double ts = 0.0) {
    LidarScan scan;
    scan.timestamp = ts;
    for (int i = -5; i <= 5; ++i) {
        for (int j = -5; j <= 5; ++j) {
            scan.points.push_back(Eigen::Vector3f(
                static_cast<float>(i + offset_x),
                static_cast<float>(j),
                0.0f));
        }
    }
    return scan;
}

}  // namespace

TEST_CASE("LoIcpOdometry: first frame returns Initializing", "[lo_icp]") {
    LoIcpOdometry odom;
    auto result = odom.processLidar(makeCubeScan(0.0, 0.0));

    REQUIRE(result.status == TrackingStatus::Initializing);
    REQUIRE(result.timestamp == 0.0);
}

TEST_CASE("LoIcpOdometry: second frame returns Tracking", "[lo_icp]") {
    LoIcpConfig cfg;
    cfg.target.voxel_size = 2.0f;
    cfg.target.max_correspondence_dist = 5.0;
    cfg.downsample_voxel_size = 1.0f;
    LoIcpOdometry odom(cfg);

    odom.processLidar(makeCubeScan(0.0, 0.0));
    auto result = odom.processLidar(makeCubeScan(0.0, 0.1));

    REQUIRE(result.status == TrackingStatus::Tracking);
}

TEST_CASE("LoIcpOdometry: identical frames give near-zero motion", "[lo_icp]") {
    LoIcpConfig cfg;
    cfg.target.voxel_size = 2.0f;
    cfg.target.max_correspondence_dist = 5.0;
    cfg.downsample_voxel_size = 1.0f;
    LoIcpOdometry odom(cfg);

    odom.processLidar(makeCubeScan(0.0, 0.0));
    auto result = odom.processLidar(makeCubeScan(0.0, 0.1));

    double translation = result.pose.translation().norm();
    REQUIRE(translation < 0.1);
}

TEST_CASE("LoIcpOdometry: reset clears state", "[lo_icp]") {
    LoIcpOdometry odom;
    odom.processLidar(makeCubeScan(0.0, 0.0));
    odom.reset();

    auto result = odom.processLidar(makeCubeScan(0.0, 1.0));
    REQUIRE(result.status == TrackingStatus::Initializing);
}

TEST_CASE("LoIcpOdometry: name returns LoIcpOdometry", "[lo_icp]") {
    LoIcpOdometry odom;
    REQUIRE(odom.name() == "LoIcpOdometry");
}

TEST_CASE("LoIcpOdometry: multiple frames without crash", "[lo_icp]") {
    LoIcpConfig cfg;
    cfg.target.voxel_size = 2.0f;
    cfg.target.max_correspondence_dist = 5.0;
    cfg.downsample_voxel_size = 1.0f;
    cfg.solver.max_iterations = 10;
    LoIcpOdometry odom(cfg);

    for (int i = 0; i < 20; ++i) {
        double offset = i * 0.1;
        auto result = odom.processLidar(makeCubeScan(offset, i * 0.1));
        REQUIRE(result.pose.translation().allFinite());
    }
}
