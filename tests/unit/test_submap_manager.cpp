#include <SimpleSLAM/backend/submap_manager.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

using namespace simpleslam;

TEST_CASE("SubMapManager 初始无活跃子图", "[submap_manager]") {
    SubMapManager mgr;
    REQUIRE(mgr.submapCount() == 0);
    REQUIRE_FALSE(mgr.activeSubmapId().has_value());
    REQUIRE(mgr.activeSubmap() == nullptr);
}

TEST_CASE("SubMapManager createSubmap 正确锚点", "[submap_manager]") {
    SubMapManager mgr;
    SE3d anchor;
    uint64_t id = mgr.createSubmap(anchor);

    REQUIRE(mgr.submapCount() == 1);
    REQUIRE(mgr.activeSubmapId().value() == id);
    REQUIRE(mgr.activeSubmap() != nullptr);
    REQUIRE_FALSE(mgr.activeSubmap()->isFrozen());
}

TEST_CASE("SubMapManager addKeyframe 添加到活跃子图", "[submap_manager]") {
    SubMapManager mgr;
    mgr.createSubmap(SE3d{});

    mgr.addKeyframe(10);
    mgr.addKeyframe(20);
    mgr.addKeyframe(30);

    REQUIRE(mgr.activeSubmap()->keyframeCount() == 3);
    REQUIRE(mgr.activeSubmap()->keyframe_ids[0] == 10);
    REQUIRE(mgr.activeSubmap()->keyframe_ids[2] == 30);
}

TEST_CASE("SubMapManager shouldFreezeActive 阈值判定", "[submap_manager]") {
    SubMapManagerConfig config;
    config.max_keyframes_per_submap = 3;
    SubMapManager mgr(config);
    mgr.createSubmap(SE3d{});

    mgr.addKeyframe(1);
    mgr.addKeyframe(2);
    REQUIRE_FALSE(mgr.shouldFreezeActive());

    mgr.addKeyframe(3);
    REQUIRE(mgr.shouldFreezeActive());
}

TEST_CASE("SubMapManager freezeAndCreateNew 冻结并创建新子图", "[submap_manager]") {
    SubMapManagerConfig config;
    config.max_keyframes_per_submap = 2;
    SubMapManager mgr(config);

    uint64_t id0 = mgr.createSubmap(SE3d{});
    mgr.addKeyframe(1);
    mgr.addKeyframe(2);

    uint64_t id1 = mgr.freezeAndCreateNew(SE3d{});

    REQUIRE(mgr.submapCount() == 2);
    REQUIRE(mgr.activeSubmapId().value() == id1);
    REQUIRE(mgr.getSubmap(id0)->isFrozen());
    REQUIRE_FALSE(mgr.getSubmap(id1)->isFrozen());
    REQUIRE(mgr.getSubmap(id0)->keyframeCount() == 2);
    REQUIRE(mgr.getSubmap(id1)->keyframeCount() == 0);
}

TEST_CASE("SubMapManager applyCorrections 更新锚点位姿", "[submap_manager]") {
    SubMapManager mgr;
    uint64_t id0 = mgr.createSubmap(SE3d{});
    uint64_t id1 = mgr.freezeAndCreateNew(SE3d{});

    SE3d corrected_pose(Vec3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity());
    std::unordered_map<uint64_t, SE3d> corrections;
    corrections[id0] = corrected_pose;

    mgr.applyCorrections(corrections);

    auto t = mgr.getSubmap(id0)->T_world_submap.translation();
    REQUIRE(t.x() == Catch::Approx(1.0));
    REQUIRE(t.y() == Catch::Approx(2.0));
    REQUIRE(t.z() == Catch::Approx(3.0));
}

TEST_CASE("SubMapManager applyCorrections 拒绝过期 epoch（不变量4）", "[submap_manager]") {
    SubMapManager mgr;
    uint64_t id0 = mgr.createSubmap(SE3d::Identity());

    SE3d pose_v5(Vec3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity());
    std::unordered_map<uint64_t, SE3d> c5{{id0, pose_v5}};
    REQUIRE(mgr.applyCorrections(c5, GraphEpoch{5}));        // 新版本：应用
    REQUIRE(epochValue(mgr.appliedEpoch()) == 5);
    REQUIRE(mgr.getSubmap(id0)->T_world_submap.translation().x() == Catch::Approx(1.0));

    // 过期校正（epoch 3 < 已应用 5）：拒绝，位姿与 epoch 均不变
    SE3d pose_v3(Vec3d(9.0, 9.0, 9.0), Eigen::Quaterniond::Identity());
    std::unordered_map<uint64_t, SE3d> c3{{id0, pose_v3}};
    REQUIRE_FALSE(mgr.applyCorrections(c3, GraphEpoch{3}));  // 过期：拒
    REQUIRE(mgr.getSubmap(id0)->T_world_submap.translation().x() == Catch::Approx(1.0));
    REQUIRE(epochValue(mgr.appliedEpoch()) == 5);

    // 更新版本（6 >= 5）：应用
    SE3d pose_v6(Vec3d(4.0, 5.0, 6.0), Eigen::Quaterniond::Identity());
    std::unordered_map<uint64_t, SE3d> c6{{id0, pose_v6}};
    REQUIRE(mgr.applyCorrections(c6, GraphEpoch{6}));
    REQUIRE(epochValue(mgr.appliedEpoch()) == 6);
    REQUIRE(mgr.getSubmap(id0)->T_world_submap.translation().x() == Catch::Approx(4.0));
}
