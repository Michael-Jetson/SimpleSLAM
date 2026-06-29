/// @file test_icp_robustness.cpp
/// 回归：ICP 数值守卫（深度审计 C 类，CRIT）。
///
/// 危险链（修前）：退化几何 → 奇异/近奇异法方程 → 非有限 δξ →
/// exp(NaN)·pose = NaN → target_.update(NaN) 永久污染地图，且 status 仍报 Tracking。
/// 修后：solver 阻尼+finite 守卫绝不返回非有限；lo_icp 对应不足报 Degraded、
/// 位姿非有限报 Lost 且不更新地图。

#include <catch2/catch_test_macros.hpp>

#include <vector>

#include <SimpleSLAM/core/math/icp_solver.hpp>
#include <SimpleSLAM/odometry/lo_icp_odometry.hpp>

using namespace simpleslam;

namespace {
LidarScan makeCubeScan(double offset_x, double ts) {
    LidarScan scan;
    scan.timestamp = ts;
    for (int i = -5; i <= 5; ++i)
        for (int j = -5; j <= 5; ++j)
            scan.points.push_back(Eigen::Vector3f(
                static_cast<float>(i + offset_x), static_cast<float>(j), 0.0f));
    return scan;
}
}  // namespace

TEST_CASE("IcpSolver 秩亏输入产出有限增量（不 NaN/不爆炸）", "[icp][guards]") {
    // 单点对应（3 行）：旋转自由度 rx 列全零 → JtJ 秩亏（6 DoF 不可全约束）
    IcpSolver solver;
    MatchResult r;
    r.num_rows = 3;
    r.residuals = {0.5, 0.5, 0.5};
    r.jacobians = {
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 1,
        0, 0, 1, 0, -1, 0,
    };
    auto dx = solver.solveOneStep(r);
    CHECK(dx.allFinite());
    CHECK(dx.norm() < 1e3);  // 阻尼应抑制零空间方向的爆炸
}

TEST_CASE("LoIcpOdometry 无对应（远离地图）报 Degraded 而非硬编码 Tracking", "[icp][lo_icp]") {
    LoIcpConfig cfg;
    cfg.target.voxel_size = 2.0f;
    cfg.target.max_correspondence_dist = 5.0;
    cfg.downsample_voxel_size = 1.0f;
    LoIcpOdometry odom(cfg);

    odom.processLidar(makeCubeScan(0.0, 0.0));               // 建图（原点附近）
    auto result = odom.processLidar(makeCubeScan(1000.0, 0.1));  // 远离 → 无对应

    CHECK(result.status != TrackingStatus::Tracking);  // 修前：硬编码 Tracking
    CHECK(result.status == TrackingStatus::Degraded);
    CHECK(result.pose.translation().allFinite());
}
