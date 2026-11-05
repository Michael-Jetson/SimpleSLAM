// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 轨迹容器与 TUM/KITTI 读写：时间严格递增、李群插值、格式往返不丢精度。

#include <SimpleSLAM/eval/association.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>

using namespace simpleslam;
using namespace simpleslam::eval;
using Catch::Approx;

namespace {

SE3d poseAt(double x, double yaw_rad) {
    return SE3d(Vec3d(x, 0.0, 0.0),
                Eigen::Quaterniond(Eigen::AngleAxisd(yaw_rad, Vec3d::UnitZ())));
}

Trajectory straightLine(int count, double step_m, double period_s = 0.1) {
    Trajectory trajectory;
    for (int i = 0; i < count; ++i) {
        trajectory.push(fw::Timestamp{static_cast<std::int64_t>(i * period_s * 1e9)},
                        poseAt(i * step_m, 0.0));
    }
    return trajectory;
}

std::filesystem::path tempFile(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}

}  // namespace

TEST_CASE("轨迹样本必须时间严格递增", "[eval][trajectory]") {
    Trajectory trajectory;
    trajectory.push(fw::Timestamp{10}, SE3d::Identity());
    CHECK_THROWS_AS(trajectory.push(fw::Timestamp{10}, SE3d::Identity()), TrajectoryError);
    CHECK_THROWS_AS(trajectory.push(fw::Timestamp{5}, SE3d::Identity()), TrajectoryError);
}

TEST_CASE("插值在李群上做，区间外返回空", "[eval][trajectory]") {
    Trajectory trajectory;
    trajectory.push(fw::Timestamp{0}, poseAt(0.0, 0.0));
    trajectory.push(fw::Timestamp{1000}, poseAt(2.0, M_PI / 2.0));

    const auto mid = trajectory.interpolate(fw::Timestamp{500});
    REQUIRE(mid.has_value());
    CHECK(mid->translation().x() == Approx(1.0).margin(1e-9));
    // 旋转按李群插值：一半的角度，而不是四元数分量各取一半再归一
    const double yaw = 2.0 * std::atan2(mid->quat().z(), mid->quat().w());
    CHECK(yaw == Approx(M_PI / 4.0).margin(1e-9));

    CHECK_FALSE(trajectory.interpolate(fw::Timestamp{-1}).has_value());
    CHECK_FALSE(trajectory.interpolate(fw::Timestamp{1001}).has_value());
}

TEST_CASE("最近邻带容差，超出就没有关联", "[eval][trajectory]") {
    auto trajectory = straightLine(5, 1.0);
    const auto hit = trajectory.nearest(fw::Timestamp{101'000'000}, std::chrono::milliseconds(5));
    REQUIRE(hit.has_value());
    CHECK(*hit == 1);
    CHECK_FALSE(trajectory.nearest(fw::Timestamp{150'000'000}, std::chrono::milliseconds(5))
                    .has_value());
}

TEST_CASE("路径长度与时间跨度", "[eval][trajectory]") {
    const auto trajectory = straightLine(11, 0.5);
    CHECK(trajectory.pathLength() == Approx(5.0));
    CHECK(trajectory.span().begin == fw::Timestamp{0});
    CHECK(trajectory.span().end.count() == 1'000'000'000);
}

TEST_CASE("TUM 往返不丢精度", "[eval][trajectory][io]") {
    Trajectory original;
    original.push(fw::Timestamp{0}, poseAt(0.0, 0.0));
    original.push(fw::Timestamp{100'000'000}, poseAt(1.25, 0.3));
    original.push(fw::Timestamp{250'000'000}, poseAt(-2.5, -1.2));

    const auto path = tempFile("simpleslam_traj_roundtrip.tum");
    writeTrajectory(path.string(), original, TrajectoryFormat::kTum);
    const auto loaded = readTrajectory(path.string(), TrajectoryFormat::kTum);

    REQUIRE(loaded.size() == original.size());
    for (std::size_t i = 0; i < loaded.size(); ++i) {
        CHECK(loaded.at(i).time == original.at(i).time);
        CHECK((loaded.at(i).pose.translation() - original.at(i).pose.translation()).norm()
              == Approx(0.0).margin(1e-8));
        const SE3d delta = original.at(i).pose.inverse().compose(loaded.at(i).pose);
        CHECK(std::abs(delta.quat().w()) == Approx(1.0).margin(1e-8));
    }
    std::filesystem::remove(path);
}

TEST_CASE("KITTI 没有时间列，按周期或 times 文件补", "[eval][trajectory][io]") {
    const auto path = tempFile("simpleslam_traj.kitti");
    {
        std::ofstream file(path);
        file << "1 0 0 0  0 1 0 0  0 0 1 0\n";
        file << "1 0 0 1  0 1 0 0  0 0 1 0\n";
        file << "1 0 0 2  0 1 0 0  0 0 1 0\n";
    }

    TrajectoryIoOptions options;
    options.kitti_period_s = 0.5;
    const auto by_period = readTrajectory(path.string(), TrajectoryFormat::kKitti, options);
    REQUIRE(by_period.size() == 3);
    CHECK(by_period.at(1).time.count() == 500'000'000);
    CHECK(by_period.at(2).pose.translation().x() == Approx(2.0));

    const auto times_path = tempFile("simpleslam_traj_times.txt");
    {
        std::ofstream file(times_path);
        file << "0.0\n0.2\n0.9\n";
    }
    options.kitti_times_path = times_path.string();
    const auto by_times = readTrajectory(path.string(), TrajectoryFormat::kKitti, options);
    CHECK(by_times.at(2).time.count() == 900'000'000);

    std::filesystem::remove(path);
    std::filesystem::remove(times_path);
}

TEST_CASE("格式可以按内容判定", "[eval][trajectory][io]") {
    const auto tum_path = tempFile("simpleslam_detect.tum");
    writeTrajectory(tum_path.string(), straightLine(3, 1.0), TrajectoryFormat::kTum);
    CHECK(detectTrajectoryFormat(tum_path.string()) == TrajectoryFormat::kTum);

    const auto kitti_path = tempFile("simpleslam_detect.kitti");
    writeTrajectory(kitti_path.string(), straightLine(3, 1.0), TrajectoryFormat::kKitti);
    CHECK(detectTrajectoryFormat(kitti_path.string()) == TrajectoryFormat::kKitti);

    std::filesystem::remove(tum_path);
    std::filesystem::remove(kitti_path);
}

TEST_CASE("列数不对是错误，不是猜", "[eval][trajectory][io]") {
    const auto path = tempFile("simpleslam_bad.tum");
    {
        std::ofstream file(path);
        file << "0.0 1.0 2.0\n";
    }
    CHECK_THROWS_AS(detectTrajectoryFormat(path.string()), TrajectoryIoError);
    CHECK_THROWS_AS(readTrajectory(path.string(), TrajectoryFormat::kTum), TrajectoryIoError);
    std::filesystem::remove(path);
}
