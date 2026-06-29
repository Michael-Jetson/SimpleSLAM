#include <SimpleSLAM/resources/extrinsics_manager.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <atomic>
#include <thread>

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
    mgr.registerSensor("lidar", SE3d::Identity());

    Eigen::Matrix<double, 6, 1> delta;
    delta << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    mgr.updateExtrinsic("lidar", SE3d::Tangent(delta).exp());

    REQUIRE_THAT(mgr.T_body_sensor("lidar").translation().x(),
                 WithinAbs(1.0, 1e-6));
}

TEST_CASE("ExtrinsicsManager 并发 update/query 不崩（加锁 + 返回副本）", "[extrinsics][concurrency]") {
    ExtrinsicsManager mgr;
    mgr.registerSensor("s", SE3d::Identity());

    std::atomic<bool> stop{false};
    std::thread writer([&] {
        Eigen::Matrix<double, 6, 1> d;
        d << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        const SE3d v = SE3d::Tangent(d).exp();
        while (!stop.load()) mgr.updateExtrinsic("s", v);
    });

    for (int i = 0; i < 20000; ++i) {
        SE3d x = mgr.T_body_sensor("s");   // 按值，无逃逸 ref
        (void)x.translation().x();         // 修前：无锁 + 写者并发改 map → 竞争/崩
    }
    stop.store(true);
    writer.join();
    SUCCEED();
}
