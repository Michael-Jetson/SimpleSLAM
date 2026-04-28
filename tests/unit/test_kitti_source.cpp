#include <SimpleSLAM/sensor_io/kitti_source.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace simpleslam;
using Catch::Matchers::WithinAbs;

static std::string getTestDataDir() {
    return std::string(SIMPLESLAM_TEST_DATA_DIR) + "/tests/data/kitti_mini";
}

/// 在 tests/data/kitti_mini/velodyne/ 下生成 2 帧测试 .bin 文件
static void createTestData() {
    namespace fs = std::filesystem;
    const auto dir = fs::path(getTestDataDir()) / "velodyne";
    fs::create_directories(dir);

    // 每帧 5 个点 (x,y,z,intensity) = 20 floats
    auto writeFrame = [&](const std::string& name,
                          const std::vector<float>& data) {
        std::ofstream f(dir / name, std::ios::binary);
        f.write(reinterpret_cast<const char*>(data.data()),
                static_cast<std::streamsize>(data.size() * sizeof(float)));
    };

    writeFrame("000000.bin", {
        1.0f, 2.0f, 3.0f, 0.5f,
        4.0f, 5.0f, 6.0f, 0.8f,
        7.0f, 8.0f, 9.0f, 1.0f,
    });
    writeFrame("000001.bin", {
        10.0f, 20.0f, 30.0f, 0.1f,
        40.0f, 50.0f, 60.0f, 0.2f,
    });

    // times.txt
    std::ofstream ts(fs::path(getTestDataDir()) / "times.txt");
    ts << "0.000000\n";
    ts << "0.100000\n";
}

static void cleanupTestData() {
    namespace fs = std::filesystem;
    fs::remove_all(fs::path(getTestDataDir()));
}

TEST_CASE("KittiSource 读取测试数据", "[kitti_source]") {
    createTestData();

    KittiSource source(getTestDataDir());
    REQUIRE(source.totalFrames() == 2);
    REQUIRE(source.hasNext());

    // 第一帧
    auto scan0 = source.nextScan();
    REQUIRE(scan0.has_value());
    REQUIRE(scan0->size() == 3);
    REQUIRE(scan0->isConsistent());
    REQUIRE(scan0->hasIntensities());
    REQUIRE_THAT(scan0->timestamp, WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(scan0->points[0].x(), WithinAbs(1.0, 1e-5));
    REQUIRE_THAT((*scan0->intensities)[0], WithinAbs(0.5, 1e-5));

    // 第二帧
    REQUIRE(source.hasNext());
    auto scan1 = source.nextScan();
    REQUIRE(scan1.has_value());
    REQUIRE(scan1->size() == 2);
    REQUIRE_THAT(scan1->timestamp, WithinAbs(0.1, 1e-6));

    // 没有更多数据
    REQUIRE_FALSE(source.hasNext());
    auto empty = source.nextScan();
    REQUIRE_FALSE(empty.has_value());

    cleanupTestData();
}

TEST_CASE("KittiSource 无 times.txt 时使用递增时间戳", "[kitti_source]") {
    namespace fs = std::filesystem;
    createTestData();
    fs::remove(fs::path(getTestDataDir()) / "times.txt");

    KittiSource source(getTestDataDir());
    auto scan0 = source.nextScan();
    REQUIRE_THAT(scan0->timestamp, WithinAbs(0.0, 1e-6));
    auto scan1 = source.nextScan();
    REQUIRE_THAT(scan1->timestamp, WithinAbs(0.1, 1e-6));

    cleanupTestData();
}
