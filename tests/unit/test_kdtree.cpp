#include <SimpleSLAM/core/math/kdtree.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace simpleslam;
using Catch::Matchers::WithinAbs;

static std::vector<Eigen::Vector3f> makeCubeCorners() {
    return {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
        {1, 1, 0}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1},
    };
}

TEST_CASE("KDTree3f 基本属性", "[kdtree]") {
    auto pts = makeCubeCorners();
    KDTree3f tree(pts);
    REQUIRE(tree.size() == 8);
    REQUIRE_FALSE(tree.empty());
}

TEST_CASE("KDTree3f KNN 搜索最近邻", "[kdtree]") {
    auto pts = makeCubeCorners();
    KDTree3f tree(pts);

    Eigen::Vector3f query(0.0f, 0.0f, 0.0f);
    std::vector<size_t> indices;
    std::vector<float> sq_dists;
    tree.knnSearch(query, 1, indices, sq_dists);

    REQUIRE(indices.size() == 1);
    REQUIRE(indices[0] == 0);
    REQUIRE_THAT(sq_dists[0], WithinAbs(0.0, 1e-6));
}

TEST_CASE("KDTree3f KNN 搜索 k=3", "[kdtree]") {
    auto pts = makeCubeCorners();
    KDTree3f tree(pts);

    Eigen::Vector3f query(0.0f, 0.0f, 0.0f);
    std::vector<size_t> indices;
    std::vector<float> sq_dists;
    tree.knnSearch(query, 3, indices, sq_dists);

    REQUIRE(indices.size() == 3);
    // 最近的 3 个：原点本身(0) + 三个轴上的邻居(距离=1)
    REQUIRE_THAT(sq_dists[0], WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(sq_dists[1], WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(sq_dists[2], WithinAbs(1.0, 1e-6));
}

TEST_CASE("KDTree3f 半径搜索", "[kdtree]") {
    auto pts = makeCubeCorners();
    KDTree3f tree(pts);

    Eigen::Vector3f query(0.0f, 0.0f, 0.0f);
    std::vector<size_t> indices;
    std::vector<float> sq_dists;
    // 半径 1.01 应找到原点 + 3 个轴邻居 = 4 个
    tree.radiusSearch(query, 1.01f, indices, sq_dists);

    REQUIRE(indices.size() == 4);
}

TEST_CASE("KDTree3f k 大于点数时返回全部", "[kdtree]") {
    std::vector<Eigen::Vector3f> pts = {
        {0, 0, 0}, {1, 0, 0},
    };
    KDTree3f tree(pts);

    std::vector<size_t> indices;
    std::vector<float> sq_dists;
    tree.knnSearch(Eigen::Vector3f::Zero(), 100, indices, sq_dists);

    REQUIRE(indices.size() == 2);
}

TEST_CASE("KDTree3f rebuild 重建索引", "[kdtree]") {
    std::vector<Eigen::Vector3f> pts = {{0, 0, 0}};
    KDTree3f tree(pts);
    REQUIRE(tree.size() == 1);

    pts.push_back({1, 0, 0});
    tree.rebuild();
    REQUIRE(tree.size() == 2);

    std::vector<size_t> indices;
    std::vector<float> sq_dists;
    tree.knnSearch(Eigen::Vector3f(0.9f, 0.0f, 0.0f), 1, indices, sq_dists);
    REQUIRE(indices[0] == 1);
}
