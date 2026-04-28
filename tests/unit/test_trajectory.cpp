#include <SimpleSLAM/resources/trajectory.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

using namespace simpleslam;
using Catch::Matchers::WithinAbs;

TEST_CASE("Trajectory append 和 size", "[trajectory]") {
    Trajectory traj;
    REQUIRE(traj.empty());
    traj.append(0.0, SE3d{});
    traj.append(1.0, SE3d{});
    REQUIRE(traj.size() == 2);
}

TEST_CASE("Trajectory poseAt 线性插值", "[trajectory]") {
    Trajectory traj;

    // 三个点：t=0 原点, t=1 平移(2,0,0), t=2 平移(4,0,0)
    Eigen::Matrix<double, 6, 1> d1, d2;
    d1 << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    d2 << 4.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    traj.append(0.0, SE3d{});
    traj.append(1.0, SE3d::Tangent(d1).exp());
    traj.append(2.0, SE3d::Tangent(d2).exp());

    // t=0.5 应在第一段中间：x ≈ 1.0
    auto mid = traj.poseAt(0.5);
    REQUIRE(mid.has_value());
    REQUIRE_THAT(mid->translation().x(), WithinAbs(1.0, 0.1));

    // t=1.5 应在第二段中间：x ≈ 3.0
    auto mid2 = traj.poseAt(1.5);
    REQUIRE(mid2.has_value());
    REQUIRE_THAT(mid2->translation().x(), WithinAbs(3.0, 0.1));
}

TEST_CASE("Trajectory poseAt 边界外", "[trajectory]") {
    Trajectory traj;
    traj.append(1.0, SE3d{});
    traj.append(2.0, SE3d{});

    auto before = traj.poseAt(0.5);
    REQUIRE(before.has_value());

    auto after = traj.poseAt(3.0);
    REQUIRE(after.has_value());
}

TEST_CASE("Trajectory exportTUM 格式", "[trajectory]") {
    Trajectory traj;
    traj.append(1.0, SE3d{});

    const std::string path = "/tmp/test_traj.tum";
    traj.exportTUM(path);

    std::ifstream file(path);
    REQUIRE(file.is_open());
    std::string line;
    std::getline(file, line);
    REQUIRE_FALSE(line.empty());
    // TUM 格式：timestamp tx ty tz qx qy qz qw
    // 单位变换应有 tx=ty=tz=0, qx=qy=qz=0, qw=1
    REQUIRE(line.find("1.0") != std::string::npos);
    std::filesystem::remove(path);
}

TEST_CASE("Trajectory exportKITTI 格式", "[trajectory]") {
    Trajectory traj;
    traj.append(0.0, SE3d{});

    const std::string path = "/tmp/test_traj.kitti";
    traj.exportKITTI(path);

    std::ifstream file(path);
    REQUIRE(file.is_open());
    std::string line;
    std::getline(file, line);
    // 单位矩阵前 3 行：1 0 0 0 | 0 1 0 0 | 0 0 1 0
    REQUIRE(line.find("1.0") != std::string::npos);
    std::filesystem::remove(path);
}

TEST_CASE("Trajectory clear", "[trajectory]") {
    Trajectory traj;
    traj.append(0.0, SE3d{});
    traj.clear();
    REQUIRE(traj.empty());
}
