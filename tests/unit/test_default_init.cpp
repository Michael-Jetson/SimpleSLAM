/// @file test_default_init.cpp
/// 回归：默认构造的位姿/协方差必须是合法 identity/zero，而非零四元数或垃圾。
///
/// 背景（深度审计 A 类）：`SE3d pose{}` 值初始化得到零四元数 (0,0,0,0)——
/// 既非 identity（w=1）也非归一，对其做位姿运算会产生 NaN；`Mat6d cov{}` 在
/// 默认初始化（非值初始化）路径下是未初始化垃圾（项目未定义
/// EIGEN_INITIALIZE_MATRICES_BY_ZERO）。修复为显式 SE3d::Identity()/Mat6d::Zero()。

#include <catch2/catch_test_macros.hpp>

#include <SimpleSLAM/core/types/event_types.hpp>
#include <SimpleSLAM/core/types/keyframe.hpp>
#include <SimpleSLAM/core/types/odometry_result.hpp>
#include <SimpleSLAM/resources/current_state.hpp>

using namespace simpleslam;

// ── 危险位姿：零四元数 ≠ identity（确定性，无需依赖未初始化内存）──

TEST_CASE("默认构造的 pose 是合法 identity 而非零四元数", "[default_init]") {
    CHECK(OdometryResult{}.pose.isApprox(SE3d::Identity()));
    CHECK(KeyframeData{}.pose.isApprox(SE3d::Identity()));
    CHECK(CurrentState::OdomSnapshot{}.pose.isApprox(SE3d::Identity()));
    CHECK(KeyframeEvent{}.pose.isApprox(SE3d::Identity()));
    CHECK(LoopDetectedEvent{}.T_match_query.isApprox(SE3d::Identity()));
    CHECK(TrackingLostEvent{}.last_good_pose.isApprox(SE3d::Identity()));
}

TEST_CASE("CurrentState 无校正时主位姿输出是 identity", "[default_init]") {
    CurrentState s;
    CHECK(s.readCorrection().isApprox(SE3d::Identity()));
    CHECK(s.correctedPose().isApprox(SE3d::Identity()));
}

// ── 协方差/标量硬化：锁定 zero/0 保证（值初始化已安全，NSDMI 额外覆盖 default-init 路径）──

TEST_CASE("默认构造的 covariance 为零矩阵", "[default_init]") {
    CHECK(OdometryResult{}.covariance.isZero());
    CHECK(CurrentState::OdomSnapshot{}.covariance.isZero());
}

TEST_CASE("默认构造的事件标量字段归零", "[default_init]") {
    CHECK(KeyframeEvent{}.keyframe_id == 0);
    CHECK(KeyframeEvent{}.timestamp == 0.0);
    CHECK(LoopDetectedEvent{}.query_keyframe_id == 0);
    CHECK(LoopDetectedEvent{}.match_keyframe_id == 0);
    CHECK(LoopDetectedEvent{}.confidence == 0.0);
    CHECK(TrackingLostEvent{}.timestamp == 0.0);
}
