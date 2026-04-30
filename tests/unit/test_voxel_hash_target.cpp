#include <SimpleSLAM/odometry/voxel_hash_target.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace simpleslam;

namespace {

LidarScan makeTestScan(const std::vector<Eigen::Vector3f>& pts, double ts = 0.0) {
    LidarScan scan;
    scan.timestamp = ts;
    scan.points = pts;
    return scan;
}

}  // namespace

TEST_CASE("VoxelHashTarget: empty after construction", "[voxel_hash_target]") {
    VoxelHashTarget target;
    REQUIRE(target.empty());
    REQUIRE(target.size() == 0);
}

TEST_CASE("VoxelHashTarget: update inserts points", "[voxel_hash_target]") {
    VoxelHashTargetConfig cfg{.voxel_size = 1.0f, .max_points_per_voxel = 20};
    VoxelHashTarget target(cfg);

    auto scan = makeTestScan({
        {1.0f, 2.0f, 3.0f},
        {1.5f, 2.5f, 3.5f},
        {10.0f, 0.0f, 0.0f},
    });
    target.update(scan, SE3d{});

    REQUIRE_FALSE(target.empty());
    REQUIRE(target.size() == 3);
}

TEST_CASE("VoxelHashTarget: max_range filters distant points", "[voxel_hash_target]") {
    VoxelHashTargetConfig cfg{.voxel_size = 1.0f, .max_range = 5.0};
    VoxelHashTarget target(cfg);

    auto scan = makeTestScan({
        {1.0f, 0.0f, 0.0f},
        {10.0f, 0.0f, 0.0f},
    });
    target.update(scan, SE3d{});
    REQUIRE(target.size() == 1);
}

TEST_CASE("VoxelHashTarget: max_points_per_voxel limits bucket", "[voxel_hash_target]") {
    VoxelHashTargetConfig cfg{.voxel_size = 100.0f, .max_points_per_voxel = 3};
    VoxelHashTarget target(cfg);

    auto scan = makeTestScan({
        {1.0f, 0.0f, 0.0f},
        {2.0f, 0.0f, 0.0f},
        {3.0f, 0.0f, 0.0f},
        {4.0f, 0.0f, 0.0f},
        {5.0f, 0.0f, 0.0f},
    });
    target.update(scan, SE3d{});
    REQUIRE(target.size() == 3);
}

TEST_CASE("VoxelHashTarget: match produces residuals and jacobians", "[voxel_hash_target]") {
    VoxelHashTargetConfig cfg{.voxel_size = 1.0f, .max_correspondence_dist = 5.0};
    VoxelHashTarget target(cfg);

    auto map_scan = makeTestScan({
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    });
    target.update(map_scan, SE3d{});

    auto query_scan = makeTestScan({
        {1.05f, 0.01f, -0.02f},
        {0.03f, 1.02f, 0.01f},
    });

    MatchResult result;
    target.match(query_scan, SE3d{}, result);

    REQUIRE(result.num_valid > 0);
    REQUIRE(result.num_valid % 3 == 0);
    REQUIRE(result.residuals.size() == static_cast<size_t>(result.num_valid));
    REQUIRE(result.jacobians.size() == static_cast<size_t>(result.num_valid) * 6);
}

TEST_CASE("VoxelHashTarget: identity match gives zero residuals", "[voxel_hash_target]") {
    VoxelHashTargetConfig cfg{.voxel_size = 2.0f, .max_correspondence_dist = 1.0};
    VoxelHashTarget target(cfg);

    std::vector<Eigen::Vector3f> pts;
    for (int i = 0; i < 10; ++i)
        pts.push_back(Eigen::Vector3f(static_cast<float>(i), 0.0f, 0.0f));
    target.update(makeTestScan(pts), SE3d{});

    MatchResult result;
    target.match(makeTestScan(pts), SE3d{}, result);

    REQUIRE(result.num_valid > 0);
    for (int i = 0; i < result.num_valid; ++i)
        REQUIRE_THAT(result.residuals[i], Catch::Matchers::WithinAbs(0.0, 1e-10));
}

TEST_CASE("VoxelHashTarget: removeIfFar removes distant voxels", "[voxel_hash_target]") {
    VoxelHashTarget target(VoxelHashTargetConfig{.voxel_size = 1.0f});

    auto scan = makeTestScan({{1.0f, 0.0f, 0.0f}, {50.0f, 0.0f, 0.0f}});
    target.update(scan, SE3d{});
    REQUIRE(target.size() == 2);

    target.removeIfFar(Eigen::Vector3d::Zero(), 10.0);
    REQUIRE(target.size() == 1);
}

TEST_CASE("VoxelHashTarget: clear empties the map", "[voxel_hash_target]") {
    VoxelHashTarget target;
    target.update(makeTestScan({{1.0f, 2.0f, 3.0f}}), SE3d{});
    REQUIRE_FALSE(target.empty());

    target.clear();
    REQUIRE(target.empty());
}

TEST_CASE("VoxelHashTarget: static_assert RegistrationTarget", "[voxel_hash_target]") {
    static_assert(RegistrationTarget<VoxelHashTarget>);
}
