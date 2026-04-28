#include <SimpleSLAM/core/types/sensor_data.hpp>

#include <catch2/catch_test_macros.hpp>
#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace simpleslam;

static LidarScan makeTestScan(size_t n, bool with_optionals = false) {
    LidarScan scan;
    scan.timestamp = 1000.0;
    scan.points.resize(n);
    for (size_t i = 0; i < n; ++i) {
        float fi = static_cast<float>(i);
        scan.points[i] = Eigen::Vector3f(fi, fi + 1.0f, fi + 2.0f);
    }
    if (with_optionals) {
        auto& inten = scan.intensities.emplace();
        inten.resize(n, 0.5f);
        auto& rings = scan.rings.emplace();
        rings.resize(n, 0);
        auto& normals = scan.normals.emplace();
        normals.resize(n, Eigen::Vector3f::UnitZ());
    }
    return scan;
}

TEST_CASE("LidarScan 基本属性", "[sensor_data]") {
    auto scan = makeTestScan(100);
    REQUIRE(scan.size() == 100);
    REQUIRE_FALSE(scan.empty());
    REQUIRE_FALSE(scan.isOrganized());
    REQUIRE(scan.isConsistent());
}

TEST_CASE("LidarScan 可选字段查询", "[sensor_data]") {
    auto scan = makeTestScan(10, true);
    REQUIRE(scan.hasIntensities());
    REQUIRE(scan.hasRings());
    REQUIRE(scan.hasNormals());
    REQUIRE_FALSE(scan.hasPointOffsets());
    REQUIRE_FALSE(scan.hasColors());
}

TEST_CASE("LidarScan isConsistent 尺寸不匹配检测", "[sensor_data]") {
    auto scan = makeTestScan(10, true);
    REQUIRE(scan.isConsistent());

    scan.intensities->push_back(1.0f);
    REQUIRE_FALSE(scan.isConsistent());
}

TEST_CASE("LidarScan reserve 预分配", "[sensor_data]") {
    LidarScan scan;
    scan.intensities.emplace();
    scan.normals.emplace();
    scan.reserve(1000);
    REQUIRE(scan.points.capacity() >= 1000);
    REQUIRE(scan.intensities->capacity() >= 1000);
    REQUIRE(scan.normals->capacity() >= 1000);
}

TEST_CASE("LidarScan extras 泛型扩展", "[sensor_data]") {
    auto scan = makeTestScan(5);
    scan.extras["confidence"] = std::vector<float>{0.9f, 0.8f, 0.7f, 0.6f, 0.5f};

    REQUIRE(scan.hasExtra("confidence"));
    REQUIRE_FALSE(scan.hasExtra("nonexistent"));

    auto* vec = scan.tryGetExtra<std::vector<float>>("confidence");
    REQUIRE(vec != nullptr);
    REQUIRE(vec->size() == 5);
    REQUIRE((*vec)[0] == 0.9f);

    auto* wrong_type = scan.tryGetExtra<int>("confidence");
    REQUIRE(wrong_type == nullptr);
}

TEST_CASE("ScanLayout 2D 索引往返", "[sensor_data]") {
    ScanLayout layout;
    layout.type = ScanLayout::Type::Organized;
    layout.height = 4;
    layout.width = 8;

    REQUIRE(layout.index(0, 0) == 0);
    REQUIRE(layout.index(1, 0) == 8);
    REQUIRE(layout.index(3, 7) == 31);

    auto [row, col] = layout.rowCol(11);
    REQUIRE(row == 1);
    REQUIRE(col == 3);

    for (uint32_t r = 0; r < 4; ++r) {
        for (uint32_t c = 0; c < 8; ++c) {
            auto [rr, cc] = layout.rowCol(layout.index(r, c));
            REQUIRE(rr == r);
            REQUIRE(cc == c);
        }
    }
}

TEST_CASE("ScanLayout::isValid 有限/无穷判定", "[sensor_data]") {
    REQUIRE(ScanLayout::isValid(Eigen::Vector3f(1.0f, 2.0f, 3.0f)));
    REQUIRE_FALSE(ScanLayout::isValid(
        Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f)));
    REQUIRE_FALSE(ScanLayout::isValid(
        Eigen::Vector3f(0.0f, std::numeric_limits<float>::infinity(), 0.0f)));
}

TEST_CASE("LidarScan Organized 一致性", "[sensor_data]") {
    LidarScan scan;
    scan.layout.type = ScanLayout::Type::Organized;
    scan.layout.height = 2;
    scan.layout.width = 3;
    scan.points.resize(6, Eigen::Vector3f::Zero());
    REQUIRE(scan.isOrganized());
    REQUIRE(scan.isConsistent());

    scan.points.resize(5);
    REQUIRE_FALSE(scan.isConsistent());
}
