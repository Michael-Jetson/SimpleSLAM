#include <SimpleSLAM/core/math/pcd_io.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <filesystem>
#include <string>

using namespace simpleslam;
using Catch::Matchers::WithinAbs;

static LidarScan makeTestScan() {
    LidarScan scan;
    scan.points = {
        Eigen::Vector3f(1, 2, 3),
        Eigen::Vector3f(4, 5, 6),
        Eigen::Vector3f(7, 8, 9),
    };
    auto& inten = scan.intensities.emplace();
    inten = {0.1f, 0.5f, 0.9f};
    auto& normals = scan.normals.emplace();
    normals = {
        Eigen::Vector3f(0, 0, 1),
        Eigen::Vector3f(0, 1, 0),
        Eigen::Vector3f(1, 0, 0),
    };
    return scan;
}

TEST_CASE("PCD 写入-读取往返", "[pcd_io]") {
    const std::string path = "/tmp/test_scan.pcd";
    auto original = makeTestScan();

    pcd_io::writePCD(original, path);
    auto loaded = pcd_io::readPCD(path);

    REQUIRE(loaded.size() == original.size());
    REQUIRE(loaded.isConsistent());

    for (size_t i = 0; i < original.size(); ++i) {
        REQUIRE_THAT(loaded.points[i].x(), WithinAbs(original.points[i].x(), 1e-5));
        REQUIRE_THAT(loaded.points[i].y(), WithinAbs(original.points[i].y(), 1e-5));
        REQUIRE_THAT(loaded.points[i].z(), WithinAbs(original.points[i].z(), 1e-5));
    }

    REQUIRE(loaded.hasIntensities());
    REQUIRE_THAT((*loaded.intensities)[0], WithinAbs(0.1f, 1e-5));

    REQUIRE(loaded.hasNormals());
    REQUIRE_THAT((*loaded.normals)[2].x(), WithinAbs(1.0f, 1e-5));

    std::filesystem::remove(path);
}

TEST_CASE("PCD 纯坐标（无可选字段）", "[pcd_io]") {
    const std::string path = "/tmp/test_xyz_only.pcd";
    LidarScan scan;
    scan.points = {Eigen::Vector3f(1, 0, 0), Eigen::Vector3f(0, 1, 0)};

    pcd_io::writePCD(scan, path);
    auto loaded = pcd_io::readPCD(path);

    REQUIRE(loaded.size() == 2);
    REQUIRE_FALSE(loaded.hasIntensities());
    REQUIRE_FALSE(loaded.hasNormals());

    std::filesystem::remove(path);
}

TEST_CASE("PCD 不存在的文件抛异常", "[pcd_io]") {
    REQUIRE_THROWS(pcd_io::readPCD("/tmp/nonexistent_file_12345.pcd"));
}
