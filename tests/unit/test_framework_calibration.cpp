// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 标定与去畸变：外参链、"没标定"与"标定过且是单位阵"可分、
/// 版本由提交入口生成、来源未定的外参取不出来、YAML 三种旋转写法、
/// 去畸变把外参进出各做一次并给出明确状态。

#include <SimpleSLAM/config/calibration_io.hpp>
#include <SimpleSLAM/framework/deskew.hpp>
#include <SimpleSLAM/modules/topics.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>

using namespace simpleslam;
using Catch::Approx;
namespace fw = simpleslam::fw;
namespace cfg = simpleslam::config;
namespace modules = simpleslam::modules;

namespace {

SE3d translation(double x, double y, double z) {
    return SE3d(Vec3d(x, y, z), Eigen::Quaterniond::Identity());
}

SE3d yaw(double radians) {
    return SE3d(Vec3d::Zero(), Eigen::Quaterniond(Eigen::AngleAxisd(radians, Vec3d::UnitZ())));
}

}  // namespace

TEST_CASE("没标定与标定过是两回事", "[framework][calibration]") {
    fw::CalibrationData calibration;
    CHECK(calibration.extrinsic("velodyne") == nullptr);
    CHECK_FALSE(calibration.T_body_sensor("velodyne").has_value());  // 不默认成单位阵

    fw::Extrinsic assumed;
    assumed.source = fw::CalibrationSource::kAssumed;  // 是单位阵，但我们知道它是假设的
    calibration.extrinsics["velodyne"] = assumed;
    REQUIRE(calibration.T_body_sensor("velodyne").has_value());
    CHECK(calibration.extrinsic("velodyne")->known());
    CHECK(calibration.extrinsic("velodyne")->source == fw::CalibrationSource::kAssumed);
    // 没给协方差就是不确定度未知，不是"精确"：零矩阵不能冒充"已知且为零"
    CHECK_FALSE(calibration.extrinsic("velodyne")->covariance_known);
    CHECK_FALSE(calibration.extrinsic("velodyne")->exact());
}

TEST_CASE("外参链：两个传感器之间", "[framework][calibration]") {
    fw::CalibrationData calibration;
    calibration.extrinsics["lidar"].T_body_sensor = translation(1.0, 0.0, 0.0);
    calibration.extrinsics["lidar"].source = fw::CalibrationSource::kDataset;
    calibration.extrinsics["cam0"].T_body_sensor = translation(0.0, 2.0, 0.0);
    calibration.extrinsics["cam0"].source = fw::CalibrationSource::kDataset;

    const auto T_lidar_cam = calibration.T_sensor_sensor("lidar", "cam0");
    REQUIRE(T_lidar_cam.has_value());
    CHECK(T_lidar_cam->translation().x() == Approx(-1.0));
    CHECK(T_lidar_cam->translation().y() == Approx(2.0));

    CHECK_FALSE(calibration.T_sensor_sensor("lidar", "imu").has_value());
}

TEST_CASE("标定版本由提交入口生成", "[framework][calibration]") {
    // 版本与内容同出一源：两份不同的标定都可以在文件里声称自己是 1，
    // 就地递增又会把"草稿改了几次"混进版本号
    modules::SlamResources resources;
    CHECK(resources.calibrationRevision() == 0);

    fw::CalibrationData calibration;
    calibration.revision = 7;  // 文件里写什么都不算数
    fw::Extrinsic extrinsic;
    extrinsic.source = fw::CalibrationSource::kOnline;
    calibration.set("lidar", extrinsic);

    CHECK(resources.setCalibration(calibration) == 1);
    CHECK(resources.calibrationRevision() == 1);

    // 改内容（哪怕不是外参）也是新版本，基于旧标定的异步结果就此失效（不变量 15）
    resources.modifyCalibration([](fw::CalibrationData& data) { data.imu.gravity = 9.8; });
    CHECK(resources.calibrationRevision() == 2);
    CHECK(resources.calibration.version() == resources.calibrationRevision());
}

TEST_CASE("来源未定的外参取不出来", "[framework][calibration]") {
    // 默认构造的条目里躺着一个单位阵。把它当外参返回，
    // 就是把"忘了标定"变成了"标定过，恰好是单位阵"
    fw::CalibrationData calibration;
    calibration.extrinsics["lidar"] = fw::Extrinsic{};
    CHECK_FALSE(calibration.T_body_sensor("lidar").has_value());

    fw::Extrinsic assumed;
    assumed.source = fw::CalibrationSource::kAssumed;
    calibration.set("lidar", assumed);
    CHECK(calibration.T_body_sensor("lidar").has_value());   // 已知（是假设的）
    CHECK_FALSE(calibration.hasMeasuredExtrinsic("lidar"));  // 但不是量出来的
}

TEST_CASE("标定 YAML：三种旋转写法等价", "[config][calibration]") {
    const auto by_quaternion = cfg::loadCalibrationString(R"(
extrinsics:
  lidar:
    translation: [1.0, 2.0, 3.0]
    quaternion: [0.7071067811865476, 0, 0, 0.7071067811865476]
)");
    const auto by_rpy = cfg::loadCalibrationString(R"(
extrinsics:
  lidar:
    translation: [1.0, 2.0, 3.0]
    rpy_deg: [0, 0, 90]
)");
    const auto by_matrix = cfg::loadCalibrationString(R"(
extrinsics:
  lidar:
    matrix: [0, -1, 0, 1.0,  1, 0, 0, 2.0,  0, 0, 1, 3.0]
)");

    for (const auto* loaded : {&by_quaternion, &by_rpy, &by_matrix}) {
        const auto pose = loaded->T_body_sensor("lidar");
        REQUIRE(pose.has_value());
        CHECK(pose->translation().x() == Approx(1.0));
        CHECK(pose->translation().z() == Approx(3.0));
        const SE3d delta = yaw(M_PI / 2.0).inverse().compose(
            SE3d(Vec3d::Zero(), pose->quat()));
        CHECK(std::abs(delta.quat().w()) == Approx(1.0).margin(1e-9));
    }
}

TEST_CASE("标定 YAML：旋转只能给一种，未知键报错", "[config][calibration]") {
    CHECK_THROWS_AS(cfg::loadCalibrationString(
                        "extrinsics:\n  lidar:\n    rpy_deg: [0,0,0]\n"
                        "    quaternion: [1,0,0,0]\n"),
                    cfg::CalibrationError);
    CHECK_THROWS_AS(cfg::loadCalibrationString("extrinsic:\n  lidar:\n    translation: [0,0,0]\n"),
                    cfg::CalibrationError);
    CHECK_THROWS_AS(cfg::loadCalibrationString(
                        "extrinsics:\n  lidar:\n    translatino: [0,0,0]\n"),
                    cfg::CalibrationError);
}

TEST_CASE("标定文件往返：读回来的导出与原来的相同，改了就不同", "[config][calibration]") {
    fw::CalibrationData calibration;
    calibration.device = "demo";
    calibration.imu.rate_hz = 100.0;
    calibration.imu.gravity = 9.80665;
    fw::Extrinsic lidar;
    lidar.T_body_sensor = SE3d(Vec3d(0.1, -0.2, 0.3),
                               Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Vec3d::UnitY())));
    lidar.source = fw::CalibrationSource::kDataset;
    calibration.extrinsics["velodyne"] = lidar;
    calibration.lidars["velodyne"] = fw::LidarIntrinsics{};

    const auto path = std::filesystem::temp_directory_path() / "simpleslam_calib.yaml";
    cfg::writeCalibrationFile(path.string(), calibration);
    const auto loaded = cfg::loadCalibrationFile(path.string());

    CHECK(loaded.device == "demo");
    CHECK(loaded.imu.gravity == Approx(9.80665));
    REQUIRE(loaded.T_body_sensor("velodyne").has_value());
    CHECK((loaded.T_body_sensor("velodyne")->translation() - lidar.T_body_sensor.translation())
              .norm() == Approx(0.0).margin(1e-12));
    CHECK(cfg::dumpCalibration(loaded) == cfg::dumpCalibration(calibration));

    fw::CalibrationData changed = calibration;
    changed.imu.gravity = 9.81;
    CHECK(cfg::dumpCalibration(changed) != cfg::dumpCalibration(calibration));
    std::filesystem::remove(path);
}

TEST_CASE("去畸变把外参进出各做一次", "[framework][deskew]") {
    // 传感器装在 body 前方 1 米；帧内 body 绕 z 转。
    // 传感器原点上的那个点，在参考时刻应当落在 Rz(ωΔt)·(1,0,0) - (1,0,0)。
    const SE3d T_body_sensor = translation(1.0, 0.0, 0.0);

    fw::PointCloud points;
    points.push(0.0F, 0.0F, 0.0F);
    points.time_offset_ns.push_back(-100'000'000);  // 比参考时刻早 0.1 秒

    // T_ref_body(Δt) = exp(Δt·ξ)。取 ω = +π/2 / 0.1，于是 Δt = -0.1 秒时
    // 该时刻的 body 相对参考时刻的姿态是 Rz(-90°)。
    const double omega = M_PI / 2.0 / 0.1;
    SE3d::Tangent twist;
    twist.coeffs() << 0.0, 0.0, 0.0, 0.0, 0.0, omega;

    const auto [corrected, result] = fw::deskew(points, T_body_sensor, fw::ConstantTwist(twist));
    REQUIRE(result.status == fw::DeskewStatus::kCorrected);
    REQUIRE(result.points == 1);
    REQUIRE(corrected.size() == 1);
    CHECK(corrected.x[0] == Approx(-1.0).margin(1e-9));
    CHECK(corrected.y[0] == Approx(-1.0).margin(1e-9));

    // 忽略外参会得到 (0,0,0)——这正是"少做一次外参变换"的典型症状
    const auto [without_extrinsic, ignored] =
        fw::deskew(points, SE3d::Identity(), fw::ConstantTwist(twist));
    CHECK(without_extrinsic.x[0] == Approx(0.0).margin(1e-9));
}

TEST_CASE("参考时刻的点不被移动", "[framework][deskew]") {
    fw::PointCloud points;
    points.push(3.0F, 4.0F, 5.0F);
    points.time_offset_ns.push_back(0);  // 就在参考时刻

    SE3d::Tangent twist;
    twist.coeffs() << 10.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    const auto [corrected, result] =
        fw::deskew(points, translation(0.5, 0.0, 0.0), fw::ConstantTwist(twist));
    CHECK(result.status == fw::DeskewStatus::kCorrected);
    CHECK(corrected.x[0] == Approx(3.0).margin(1e-9));
    CHECK(corrected.y[0] == Approx(4.0).margin(1e-9));
}

TEST_CASE("没有逐点时间就不假装去过畸变", "[framework][deskew]") {
    fw::PointCloud points;
    points.push(1.0F, 2.0F, 3.0F);  // 没有 time_offset_ns
    SE3d::Tangent twist;
    twist.coeffs() << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    const auto result = fw::deskewInPlace(points, SE3d::Identity(), fw::ConstantTwist(twist));
    CHECK(result.status == fw::DeskewStatus::kNoPointTimes);
    CHECK_FALSE(result.corrected());
    CHECK(points.x[0] == Approx(1.0));
}

TEST_CASE("时间列长度不符要与缺时间区分开", "[framework][deskew]") {
    // 两者都会让点云原样返回，但一个是"这份数据没有时间"，
    // 一个是"这份数据坏了"——处置不同，所以状态必须不同
    fw::PointCloud points;
    points.push(1.0F, 2.0F, 3.0F);
    points.push(4.0F, 5.0F, 6.0F);
    points.time_offset_ns.push_back(-1000);  // 只有一条，点却有两个
    SE3d::Tangent twist;
    twist.coeffs() << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    const auto result = fw::deskewInPlace(points, SE3d::Identity(), fw::ConstantTwist(twist));
    CHECK(result.status == fw::DeskewStatus::kMalformedTimes);
    CHECK(points.x[0] == Approx(1.0));
}

TEST_CASE("标定往返不丢协方差与扰动约定", "[config][calibration]") {
    // 只导对角就等于把相关性丢了，而导出照样写得出来——
    // 于是"导出相同"不再意味着"内容相同"
    fw::CalibrationData calibration;
    calibration.device = "含 # 号的设备名";
    fw::Extrinsic lidar;
    lidar.source = fw::CalibrationSource::kFile;
    lidar.convention.side = fw::PerturbationSide::kLeftWorld;
    lidar.convention.order = fw::TangentOrder::kRotationTranslation;
    for (int i = 0; i < 6; ++i) lidar.covariance(i, i) = 0.1 * (i + 1);
    lidar.covariance(0, 3) = 0.2;
    lidar.covariance(3, 0) = 0.2;
    lidar.covariance_known = true;
    lidar.fixed = false;
    calibration.extrinsics["velodyne"] = lidar;

    const auto path = std::filesystem::temp_directory_path() / "simpleslam_calib_cov.yaml";
    cfg::writeCalibrationFile(path.string(), calibration);
    const auto loaded = cfg::loadCalibrationFile(path.string());

    CHECK(loaded.device == calibration.device);  // `#` 没有被当成注释吃掉
    const auto* entry = loaded.extrinsic("velodyne");
    REQUIRE(entry != nullptr);
    CHECK(entry->covariance(0, 3) == Approx(0.2));
    CHECK(entry->convention.side == fw::PerturbationSide::kLeftWorld);
    CHECK(entry->convention.order == fw::TangentOrder::kRotationTranslation);
    CHECK(entry->covariance_known);
    CHECK_FALSE(entry->fixed);
    CHECK(cfg::dumpCalibration(loaded) == cfg::dumpCalibration(calibration));

    // 改一个非对角项，导出必须跟着变
    fw::CalibrationData changed = calibration;
    changed.extrinsics["velodyne"].covariance(1, 4) = 0.3;
    changed.extrinsics["velodyne"].covariance(4, 1) = 0.3;
    CHECK(cfg::dumpCalibration(changed) != cfg::dumpCalibration(calibration));
    std::filesystem::remove(path);
}

TEST_CASE("写不成旋转的矩阵要被拒绝", "[config][calibration]") {
    // 四元数归一化会把一个坏矩阵变成"合法"旋转，不会让它变成对的
    CHECK_THROWS_AS(cfg::loadCalibrationString(R"(
extrinsics:
  lidar:
    quaternion: [0, 0, 0, 0]
)"), cfg::CalibrationError);

    CHECK_THROWS_AS(cfg::loadCalibrationString(R"(
extrinsics:
  lidar:
    matrix: [2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2, 0]
)"), cfg::CalibrationError);

    CHECK_THROWS_AS(cfg::loadCalibrationString(R"(
extrinsics:
  lidar:
    covariance:
      - [1, 9, 0, 0, 0, 0]
      - [0, 1, 0, 0, 0, 0]
      - [0, 0, 1, 0, 0, 0]
      - [0, 0, 0, 1, 0, 0]
      - [0, 0, 0, 0, 1, 0]
      - [0, 0, 0, 0, 0, 1]
)"), cfg::CalibrationError);
}
