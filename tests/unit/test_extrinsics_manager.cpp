#include <SimpleSLAM/resources/extrinsics_manager.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace simpleslam;
using Catch::Matchers::WithinAbs;

TEST_CASE("ExtrinsicsManager 注册和查询", "[extrinsics]") {
    ExtrinsicsManager mgr;
    REQUIRE_FALSE(mgr.hasSensor("lidar"));

    Eigen::Matrix<double, 6, 1> delta;
    delta << 0.0, 0.0, 0.1, 0.0, 0.0, 0.0;
    mgr.registerSensor("lidar", SE3d::Tangent(delta).exp());

    REQUIRE(mgr.hasSensor("lidar"));
    REQUIRE_THAT(mgr.T_body_sensor("lidar").translation().z(),
                 WithinAbs(0.1, 1e-6));
}

TEST_CASE("ExtrinsicsManager 不存在的传感器抛异常", "[extrinsics]") {
    ExtrinsicsManager mgr;
    REQUIRE_THROWS(mgr.T_body_sensor("nonexistent"));
}

TEST_CASE("ExtrinsicsManager sensorNames", "[extrinsics]") {
    ExtrinsicsManager mgr;
    mgr.registerSensor("lidar", SE3d{});
    mgr.registerSensor("camera", SE3d{});
    auto names = mgr.sensorNames();
    REQUIRE(names.size() == 2);
}

TEST_CASE("ExtrinsicsManager updateExtrinsic", "[extrinsics]") {
    ExtrinsicsManager mgr;
    mgr.registerSensor("lidar", SE3d{});

    Eigen::Matrix<double, 6, 1> delta;
    delta << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    mgr.updateExtrinsic("lidar", SE3d::Tangent(delta).exp());

    REQUIRE_THAT(mgr.T_body_sensor("lidar").translation().x(),
                 WithinAbs(1.0, 1e-6));
}
