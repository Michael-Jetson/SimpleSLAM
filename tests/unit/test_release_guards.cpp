/// @file test_release_guards.cpp
/// 回归：把承重 assert 换成运行期守卫——release（NDEBUG）下不再 UB/崩溃。
///
/// 背景（深度审计 B 类）：以下三处仅用 assert 守卫，NDEBUG 下断言剥除后
/// 解引用空/越界 → UB。改为运行期检查（安全返回 / 抛 logic_error）。

#include <catch2/catch_test_macros.hpp>

#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include <SimpleSLAM/backend/submap_manager.hpp>
#include <SimpleSLAM/core/math/kdtree.hpp>
#include <SimpleSLAM/odometry/imu_buffer.hpp>

using namespace simpleslam;

TEST_CASE("KDTree radiusSearch 空树返回空且不崩", "[guards]") {
    std::vector<Eigen::Vector3f> empty;
    KDTree3f tree(empty);  // 点数为 0 → 内部 index_ 为空
    REQUIRE(tree.empty());

    std::vector<size_t> idx;
    std::vector<float> dsq;
    tree.radiusSearch({0.0f, 0.0f, 0.0f}, 1.0f, idx, dsq);  // 修前：空指针解引用
    CHECK(idx.empty());
    CHECK(dsq.empty());
}

TEST_CASE("SubMapManager 未建子图就 addKeyframe 抛错而非 UB", "[guards]") {
    SubMapManager mgr;
    CHECK_THROWS_AS(mgr.addKeyframe(1), std::logic_error);            // 修前：*nullopt UB
    CHECK_THROWS_AS(mgr.freezeAndCreateNew(SE3d::Identity()), std::logic_error);

    // 正常路径：建子图后可加关键帧
    mgr.createSubmap(SE3d::Identity());
    CHECK_NOTHROW(mgr.addKeyframe(1));
}

TEST_CASE("ImuBuffer query 反向区间返回空而非越界", "[guards]") {
    ImuBuffer buf;
    ImuSample s;
    s.timestamp = 1.0;
    buf.addSample(s);
    s.timestamp = 2.0;
    buf.addSample(s);

    auto result = buf.query(2.0, 1.0);  // t_start > t_end，修前：迭代器越过 end() UB
    CHECK(result.empty());
}
