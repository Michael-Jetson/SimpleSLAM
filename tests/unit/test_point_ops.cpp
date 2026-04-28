#include <SimpleSLAM/core/math/point_ops.hpp>
#include <SimpleSLAM/core/math/normal_estimation.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <Eigen/Core>
#include <cmath>

using namespace simpleslam;
using Catch::Matchers::WithinAbs;

static LidarScan makeGridScan() {
    LidarScan scan;
    scan.timestamp = 1.0;
    auto& inten = scan.intensities.emplace();

    for (int x = 0; x < 3; ++x) {
        for (int y = 0; y < 3; ++y) {
            for (int z = 0; z < 3; ++z) {
                scan.points.push_back(Eigen::Vector3f(
                    static_cast<float>(x),
                    static_cast<float>(y),
                    static_cast<float>(z)));
                inten.push_back(static_cast<float>(x + y + z));
            }
        }
    }
    return scan;
}

TEST_CASE("voxelDownsample 体素降采样", "[point_ops]") {
    auto scan = makeGridScan();
    REQUIRE(scan.size() == 27);

    auto result = voxelDownsample(scan, 1.5f);
    // 3x3x3 点在 voxel_size=1.5 下应被分为 2x2x2=8 个体素
    REQUIRE(result.size() == 8);
    REQUIRE(result.isConsistent());
    // 降采样后无可选字段（质心合并无法保持一一对应）
    REQUIRE_FALSE(result.hasIntensities());
}

TEST_CASE("voxelDownsample 空扫描", "[point_ops]") {
    LidarScan empty;
    auto result = voxelDownsample(empty, 1.0f);
    REQUIRE(result.empty());
}

TEST_CASE("rangeFilter 距离过滤", "[point_ops]") {
    LidarScan scan;
    scan.points = {
        Eigen::Vector3f(0.5f, 0, 0),   // 距离 0.5
        Eigen::Vector3f(1.0f, 0, 0),   // 距离 1.0
        Eigen::Vector3f(3.0f, 0, 0),   // 距离 3.0
        Eigen::Vector3f(10.0f, 0, 0),  // 距离 10.0
    };
    auto& inten = scan.intensities.emplace();
    inten = {10.0f, 20.0f, 30.0f, 40.0f};

    auto result = rangeFilter(scan, 1.0f, 5.0f);
    REQUIRE(result.size() == 2);
    REQUIRE(result.isConsistent());
    REQUIRE(result.points[0].x() == 1.0f);
    REQUIRE(result.points[1].x() == 3.0f);
    REQUIRE(result.hasIntensities());
    REQUIRE((*result.intensities)[0] == 20.0f);
    REQUIRE((*result.intensities)[1] == 30.0f);
}

TEST_CASE("cropBox 包围盒裁剪", "[point_ops]") {
    LidarScan scan;
    scan.points = {
        Eigen::Vector3f(0, 0, 0),       // 在盒内
        Eigen::Vector3f(5, 5, 5),       // 在盒内
        Eigen::Vector3f(10, 10, 10),    // 在盒外
        Eigen::Vector3f(-1, 0, 0),      // 在盒外
    };

    auto result = cropBox(scan,
                          Eigen::Vector3f(-0.5f, -0.5f, -0.5f),
                          Eigen::Vector3f(6.0f, 6.0f, 6.0f));
    REQUIRE(result.size() == 2);
    REQUIRE(result.isConsistent());
}

TEST_CASE("transformScan 刚体变换", "[point_ops]") {
    LidarScan scan;
    scan.points = {Eigen::Vector3f(1, 0, 0)};
    auto& normals = scan.normals.emplace();
    normals = {Eigen::Vector3f(1, 0, 0)};

    // 沿 x 轴平移 3.0
    Eigen::Matrix<double, 6, 1> tangent;
    tangent << 3.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    SE3d T = SE3d::Tangent(tangent).exp();

    auto result = transformScan(scan, T);
    REQUIRE(result.size() == 1);
    REQUIRE_THAT(result.points[0].x(), WithinAbs(4.0, 1e-4));
    REQUIRE_THAT(result.points[0].y(), WithinAbs(0.0, 1e-4));
    // 纯平移不改变法向量方向
    REQUIRE(result.hasNormals());
    REQUIRE_THAT((*result.normals)[0].x(), WithinAbs(1.0, 1e-4));
}

TEST_CASE("statisticalOutlierRemoval 离群点移除", "[point_ops]") {
    LidarScan scan;
    // 密集簇
    for (int i = 0; i < 20; ++i) {
        scan.points.push_back(Eigen::Vector3f(
            static_cast<float>(i % 5) * 0.1f,
            static_cast<float>(i / 5) * 0.1f,
            0.0f));
    }
    // 明显离群点
    scan.points.push_back(Eigen::Vector3f(100.0f, 100.0f, 100.0f));
    scan.points.push_back(Eigen::Vector3f(-50.0f, -50.0f, -50.0f));

    auto result = statisticalOutlierRemoval(scan, 5, 1.0f);
    REQUIRE(result.size() == 20);
    REQUIRE(result.isConsistent());
}

TEST_CASE("estimateNormals 法向量估计", "[point_ops]") {
    LidarScan scan;
    // XY 平面上的 3x3 网格——法向量应近似 Z 轴方向
    for (int x = 0; x < 3; ++x) {
        for (int y = 0; y < 3; ++y) {
            scan.points.push_back(Eigen::Vector3f(
                static_cast<float>(x),
                static_cast<float>(y),
                0.0f));
        }
    }

    estimateNormals(scan, 5);
    REQUIRE(scan.hasNormals());
    REQUIRE(scan.isConsistent());

    for (const auto& n : *scan.normals) {
        // 法向量应接近 +Z 或 -Z
        REQUIRE_THAT(std::abs(n.z()), WithinAbs(1.0, 0.15));
    }
}
