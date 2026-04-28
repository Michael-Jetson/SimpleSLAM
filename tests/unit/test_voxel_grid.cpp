#include <SimpleSLAM/core/math/voxel_grid.hpp>

#include <catch2/catch_test_macros.hpp>
#include <Eigen/Core>

using namespace simpleslam;

TEST_CASE("toVoxelCoord 正坐标", "[voxel_grid]") {
    auto v = toVoxelCoord(Eigen::Vector3f(1.5f, 2.7f, 0.3f), 1.0f);
    REQUIRE(v.x == 1);
    REQUIRE(v.y == 2);
    REQUIRE(v.z == 0);
}

TEST_CASE("toVoxelCoord 负坐标", "[voxel_grid]") {
    auto v = toVoxelCoord(Eigen::Vector3f(-0.5f, -1.1f, -3.9f), 1.0f);
    REQUIRE(v.x == -1);
    REQUIRE(v.y == -2);
    REQUIRE(v.z == -4);
}

TEST_CASE("toVoxelCoord 自定义体素大小", "[voxel_grid]") {
    auto v = toVoxelCoord(Eigen::Vector3f(0.25f, 0.75f, 1.5f), 0.5f);
    REQUIRE(v.x == 0);
    REQUIRE(v.y == 1);
    REQUIRE(v.z == 3);
}

TEST_CASE("VoxelHash 一致性", "[voxel_grid]") {
    VoxelHash hasher;
    VoxelCoord a{1, 2, 3};
    VoxelCoord b{1, 2, 3};
    VoxelCoord c{3, 2, 1};

    REQUIRE(hasher(a) == hasher(b));
    REQUIRE(hasher(a) != hasher(c));
}

TEST_CASE("VoxelMap 插入和查找", "[voxel_grid]") {
    VoxelMap map;
    REQUIRE(map.empty());

    VoxelCoord coord{0, 0, 0};
    Eigen::Vector3f p1(0.1f, 0.2f, 0.3f);
    Eigen::Vector3f p2(0.4f, 0.5f, 0.6f);

    map.insert(coord, p1);
    map.insert(coord, p2);

    REQUIRE(map.size() == 1);
    REQUIRE(map.contains(coord));

    const Voxel* voxel = map.find(coord);
    REQUIRE(voxel != nullptr);
    REQUIRE(voxel->numPoints() == 2);
}

TEST_CASE("VoxelMap 查找不存在的体素", "[voxel_grid]") {
    VoxelMap map;
    REQUIRE(map.find(VoxelCoord{99, 99, 99}) == nullptr);
}

TEST_CASE("VoxelMap clear 清空", "[voxel_grid]") {
    VoxelMap map;
    map.insert(VoxelCoord{0, 0, 0}, Eigen::Vector3f::Zero());
    map.insert(VoxelCoord{1, 1, 1}, Eigen::Vector3f::Ones());
    REQUIRE(map.size() == 2);

    map.clear();
    REQUIRE(map.empty());
}

TEST_CASE("VoxelMap occupiedCoords 列出已占据坐标", "[voxel_grid]") {
    VoxelMap map;
    map.insert(VoxelCoord{0, 0, 0}, Eigen::Vector3f::Zero());
    map.insert(VoxelCoord{1, 0, 0}, Eigen::Vector3f::Ones());
    map.insert(VoxelCoord{0, 0, 0}, Eigen::Vector3f::Ones());

    auto coords = map.occupiedCoords();
    REQUIRE(coords.size() == 2);
}

TEST_CASE("Voxel centroid 质心计算", "[voxel_grid]") {
    Voxel v;
    v.points.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    v.points.push_back(Eigen::Vector3f(2.0f, 4.0f, 6.0f));
    auto c = v.centroid();
    REQUIRE(c.x() == 1.0f);
    REQUIRE(c.y() == 2.0f);
    REQUIRE(c.z() == 3.0f);
}

TEST_CASE("Morton 编码基本值", "[voxel_grid]") {
    REQUIRE(VoxelHash::mortonEncode(0, 0, 0) == 0);
    REQUIRE(VoxelHash::mortonEncode(1, 0, 0) == 1);
    REQUIRE(VoxelHash::mortonEncode(0, 1, 0) == 2);
    REQUIRE(VoxelHash::mortonEncode(0, 0, 1) == 4);
    REQUIRE(VoxelHash::mortonEncode(1, 1, 1) == 7);
}
