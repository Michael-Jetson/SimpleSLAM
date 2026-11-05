// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// ATE / RPE：对齐方式是结果的一部分、平移与旋转分开统计、
/// 关联覆盖率如实给出、尺度不可观时 Sim3 能把它解出来。

#include <SimpleSLAM/eval/metrics.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace simpleslam;
using namespace simpleslam::eval;
using Catch::Approx;

namespace {

Trajectory line(int count, double step_m, double period_s = 0.1,
                const SE3d& transform = SE3d::Identity(), double scale = 1.0) {
    Trajectory trajectory;
    for (int i = 0; i < count; ++i) {
        const SE3d pose(Vec3d(i * step_m * scale, 0.0, 0.0), Eigen::Quaterniond::Identity());
        trajectory.push(fw::Timestamp{static_cast<std::int64_t>(i * period_s * 1e9)},
                        transform.compose(pose));
    }
    return trajectory;
}

}  // namespace

TEST_CASE("完全一致的轨迹误差为零", "[eval][metrics]") {
    const auto truth = line(20, 0.5);
    AteOptions options;
    options.alignment = AlignmentKind::kNone;
    const auto ate = absoluteTrajectoryError(truth, truth, options);

    CHECK(ate.pairs == 20);
    CHECK(ate.coverage() == Approx(1.0));
    CHECK(ate.translation.rmse == Approx(0.0).margin(1e-12));
    CHECK(ate.rotation_deg.rmse == Approx(0.0).margin(1e-9));
}

TEST_CASE("恒定偏移下 ATE 等于偏移量", "[eval][metrics]") {
    const auto truth = line(10, 1.0);
    Trajectory shifted;
    for (const auto& sample : truth) {
        shifted.push(sample.time,
                     SE3d(Vec3d(sample.pose.translation() + Vec3d(0.0, 0.3, 0.0)),
                          sample.pose.quat()));
    }

    AteOptions options;
    options.alignment = AlignmentKind::kNone;
    const auto raw = absoluteTrajectoryError(shifted, truth, options);
    CHECK(raw.translation.rmse == Approx(0.3).margin(1e-9));

    // SE3 对齐能把这个刚体偏移吸收掉——所以"用了哪种对齐"必须写进报告
    options.alignment = AlignmentKind::kSe3;
    const auto aligned = absoluteTrajectoryError(shifted, truth, options);
    CHECK(aligned.alignment.kind == AlignmentKind::kSe3);
    CHECK(aligned.translation.rmse == Approx(0.0).margin(1e-9));
}

TEST_CASE("Sim3 对齐解出尺度", "[eval][metrics]") {
    const auto truth = line(12, 1.0);
    const auto scaled = line(12, 1.0, 0.1, SE3d::Identity(), 2.0);

    AteOptions options;
    options.alignment = AlignmentKind::kSim3;
    const auto ate = absoluteTrajectoryError(scaled, truth, options);
    CHECK(ate.alignment.scale == Approx(0.5).epsilon(1e-6));
    CHECK(ate.translation.rmse == Approx(0.0).margin(1e-6));
}

TEST_CASE("时间对不上时覆盖率如实下降", "[eval][metrics]") {
    const auto truth = line(10, 1.0, 0.1);
    Trajectory shifted_in_time;
    for (const auto& sample : truth) {
        shifted_in_time.push(sample.time + std::chrono::milliseconds(60), sample.pose);
    }

    AteOptions options;
    options.tolerance = std::chrono::milliseconds(10);
    options.alignment = AlignmentKind::kNone;
    const auto ate = absoluteTrajectoryError(shifted_in_time, truth, options);
    CHECK(ate.pairs == 0);
    CHECK(ate.coverage() == Approx(0.0));
}

TEST_CASE("RPE 只看局部，不受全局偏移影响", "[eval][metrics]") {
    const auto truth = line(50, 0.2, 0.1);
    const SE3d big_offset(Vec3d(100.0, -50.0, 7.0),
                          Eigen::Quaterniond(Eigen::AngleAxisd(1.1, Vec3d::UnitZ())));
    const auto drifted = line(50, 0.2, 0.1, big_offset);

    RpeOptions options;
    options.delta_time = std::chrono::milliseconds(500);
    const auto rpe = relativePoseError(drifted, truth, options);
    CHECK(rpe.pairs > 0);
    CHECK(rpe.translation.rmse == Approx(0.0).margin(1e-9));
    CHECK(rpe.rotation_deg.rmse == Approx(0.0).margin(1e-7));
}

TEST_CASE("RPE 按距离间隔并归一到每米", "[eval][metrics]") {
    const auto truth = line(101, 0.1, 0.1);  // 共 10 米
    Trajectory drifting;                     // 每帧多走 1%
    for (int i = 0; i < 101; ++i) {
        drifting.push(fw::Timestamp{static_cast<std::int64_t>(i * 0.1 * 1e9)},
                      SE3d(Vec3d(i * 0.101, 0.0, 0.0), Eigen::Quaterniond::Identity()));
    }

    RpeOptions options;
    options.delta_kind = RpeDelta::kDistance;
    options.delta_distance_m = 2.0;
    options.normalize = true;
    const auto rpe = relativePoseError(drifting, truth, options);
    CHECK(rpe.normalized);
    CHECK(rpe.pairs >= 4);
    CHECK(rpe.translation.mean == Approx(0.01).epsilon(0.05));  // 每米漂 1%
}

TEST_CASE("统计量：空集合不装作有结果", "[eval][metrics]") {
    const auto empty = statisticsOf({});
    CHECK(empty.count == 0);
    CHECK(empty.rmse == 0.0);

    const auto stats = statisticsOf({1.0, 2.0, 3.0, 4.0});
    CHECK(stats.count == 4);
    CHECK(stats.mean == Approx(2.5));
    CHECK(stats.median == Approx(2.5));
    CHECK(stats.min == Approx(1.0));
    CHECK(stats.max == Approx(4.0));
    CHECK(stats.rmse == Approx(std::sqrt(30.0 / 4.0)));
}

TEST_CASE("参考侧覆盖率不会超过 100%", "[eval][metrics]") {
    // 关联是最近邻、允许多对一，所以分子必须是**不同的**参考样本数。
    // 用 pairs 作分子的话，11 个估计挤在 2 个参考样本上会得到 550%。
    Trajectory estimate;
    for (int i = 0; i < 11; ++i) {
        estimate.push(fw::Timestamp{i * 1'000'000}, SE3d::Identity());  // 每毫秒一个
    }
    Trajectory reference;
    reference.push(fw::Timestamp{0}, SE3d::Identity());
    reference.push(fw::Timestamp{10'000'000}, SE3d::Identity());

    AteOptions options;
    options.tolerance = std::chrono::milliseconds(20);
    options.alignment = AlignmentKind::kNone;
    const auto ate = absoluteTrajectoryError(estimate, reference, options);

    CHECK(ate.pairs == 11);
    CHECK(ate.matched_reference_samples == 2);
    CHECK(ate.referenceCoverage() <= 1.0);
    CHECK(ate.referenceCoverage() == Approx(1.0));
}

TEST_CASE("跨度 100% 不代表中间没有整段缺失", "[eval][metrics]") {
    // spanRatio 只衡量首尾：只输出首尾两帧也能得到 100%。
    // 中间掉没掉线要看最大连续缺口。
    Trajectory estimate;
    estimate.push(fw::Timestamp{0}, SE3d::Identity());
    estimate.push(fw::Timestamp{10'000'000'000LL}, SE3d::Identity());
    Trajectory reference;
    for (int i = 0; i <= 10; ++i) {
        reference.push(fw::Timestamp{static_cast<std::int64_t>(i) * 1'000'000'000},
                       SE3d::Identity());
    }

    AteOptions options;
    options.tolerance = std::chrono::milliseconds(20);
    options.alignment = AlignmentKind::kNone;
    const auto ate = absoluteTrajectoryError(estimate, reference, options);

    CHECK(ate.spanRatio() == Approx(1.0));         // 首尾都在，跨度满分
    CHECK(ate.referenceCoverage() < 0.3);          // 但只对上了 2/11
    CHECK(ate.maxReferenceGap() == Approx(10.0));  // 中间整整缺了 10 秒
}

TEST_CASE("不对齐就比的话，参考系不同要直接拒绝", "[eval][metrics]") {
    // 对齐本身就是那次显式换算，所以只有 kNone 才要求系别相同
    Trajectory estimate;
    estimate.push(fw::Timestamp{0}, SE3d::Identity());
    estimate.setFrame(TrajectoryFrame::kOdom);
    Trajectory reference;
    reference.push(fw::Timestamp{0}, SE3d::Identity());
    reference.setFrame(TrajectoryFrame::kWorld);

    AteOptions none;
    none.alignment = AlignmentKind::kNone;
    CHECK_THROWS_AS(absoluteTrajectoryError(estimate, reference, none), TrajectoryError);

    AteOptions aligned;
    aligned.alignment = AlignmentKind::kSe3;
    CHECK_NOTHROW(absoluteTrajectoryError(estimate, reference, aligned));
}

TEST_CASE("时钟域不同一律拒绝比较", "[eval][metrics]") {
    // 时钟域不同就是在比两把不同的尺子，对齐也救不回来
    Trajectory estimate;
    estimate.push(fw::Timestamp{0}, SE3d::Identity());
    estimate.setClock(fw::ClockDomainId{1});
    Trajectory reference;
    reference.push(fw::Timestamp{0}, SE3d::Identity());
    reference.setClock(fw::ClockDomainId{2});

    CHECK_THROWS_AS(absoluteTrajectoryError(estimate, reference, AteOptions{}), TrajectoryError);
}
