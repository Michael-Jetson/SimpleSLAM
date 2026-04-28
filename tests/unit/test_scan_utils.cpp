#include <SimpleSLAM/core/math/scan_utils.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <Eigen/Core>
#include <limits>
#include <vector>

using namespace simpleslam;

static LidarScan makeFullScan(size_t n) {
    LidarScan scan;
    scan.timestamp = 42.0;
    scan.points.resize(n);
    auto& intensities = scan.intensities.emplace();
    intensities.resize(n);
    auto& rings = scan.rings.emplace();
    rings.resize(n);
    auto& normals = scan.normals.emplace();
    normals.resize(n);

    for (size_t i = 0; i < n; ++i) {
        float fi = static_cast<float>(i);
        scan.points[i] = Eigen::Vector3f(fi, fi * 2.0f, fi * 3.0f);
        intensities[i] = fi * 0.1f;
        rings[i] = static_cast<uint16_t>(i % 4);
        normals[i] = Eigen::Vector3f::UnitZ();
    }
    return scan;
}

TEST_CASE("extractByIndices 正确提取子集", "[scan_utils]") {
    auto scan = makeFullScan(10);
    std::vector<size_t> indices = {2, 5, 7};
    auto result = extractByIndices(scan, indices);

    REQUIRE(result.size() == 3);
    REQUIRE(result.isConsistent());
    REQUIRE(result.points[0] == scan.points[2]);
    REQUIRE(result.points[1] == scan.points[5]);
    REQUIRE(result.points[2] == scan.points[7]);

    REQUIRE(result.hasIntensities());
    REQUIRE((*result.intensities)[0] == (*scan.intensities)[2]);

    REQUIRE(result.hasRings());
    REQUIRE((*result.rings)[1] == (*scan.rings)[5]);

    REQUIRE(result.hasNormals());
}

TEST_CASE("extractByIndices 空索引返回空扫描", "[scan_utils]") {
    auto scan = makeFullScan(10);
    std::vector<size_t> empty_indices;
    auto result = extractByIndices(scan, empty_indices);

    REQUIRE(result.empty());
    REQUIRE(result.isConsistent());
    REQUIRE(result.timestamp == scan.timestamp);
}

TEST_CASE("extractByIndices 结果为 Unorganized", "[scan_utils]") {
    LidarScan scan;
    scan.layout.type = ScanLayout::Type::Organized;
    scan.layout.height = 2;
    scan.layout.width = 3;
    scan.points.resize(6, Eigen::Vector3f::Zero());

    std::vector<size_t> indices = {0, 3, 5};
    auto result = extractByIndices(scan, indices);
    REQUIRE_FALSE(result.isOrganized());
}

TEST_CASE("extractByIndices 无可选字段时不创建", "[scan_utils]") {
    LidarScan scan;
    scan.points = {Eigen::Vector3f(1, 2, 3), Eigen::Vector3f(4, 5, 6)};

    std::vector<size_t> indices = {0};
    auto result = extractByIndices(scan, indices);
    REQUIRE_FALSE(result.hasIntensities());
    REQUIRE_FALSE(result.hasNormals());
    REQUIRE_FALSE(result.hasRings());
}

TEST_CASE("RangeImageView 2D 访问", "[scan_utils]") {
    LidarScan scan;
    scan.layout.type = ScanLayout::Type::Organized;
    scan.layout.height = 2;
    scan.layout.width = 3;
    scan.points.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        scan.points[i] = Eigen::Vector3f(
            static_cast<float>(i), 0.0f, 0.0f);
    }
    scan.points[4] = Eigen::Vector3f(
        std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);

    RangeImageView view(scan);
    REQUIRE(view.height() == 2);
    REQUIRE(view.width() == 3);
    REQUIRE(view.at(0, 0).x() == 0.0f);
    REQUIRE(view.at(1, 2).x() == 5.0f);
    REQUIRE(view.isValid(0, 0));
    REQUIRE_FALSE(view.isValid(1, 1));
}
