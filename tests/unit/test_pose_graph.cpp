#include <SimpleSLAM/resources/pose_graph.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace simpleslam;

TEST_CASE("PoseGraph 添加节点和查询", "[pose_graph]") {
    PoseGraph graph;
    REQUIRE(graph.empty());

    graph.addNode({1, 0.0, SE3d{}});
    graph.addNode({2, 0.1, SE3d{}});
    REQUIRE(graph.nodeCount() == 2);
    REQUIRE_FALSE(graph.empty());

    auto node = graph.getNode(1);
    REQUIRE(node.has_value());
    REQUIRE(node->id == 1);

    REQUIRE_FALSE(graph.getNode(99).has_value());
}

TEST_CASE("PoseGraph 添加边", "[pose_graph]") {
    PoseGraph graph;
    graph.addNode({1, 0.0, SE3d{}});
    graph.addNode({2, 0.1, SE3d{}});
    graph.addEdge({1, 2, SE3d{}, Mat6d::Identity(), EdgeType::Odometry});
    graph.addEdge({1, 2, SE3d{}, Mat6d::Identity(), EdgeType::Loop});

    REQUIRE(graph.edgeCount() == 2);
    REQUIRE(graph.edgesOfType(EdgeType::Odometry).size() == 1);
    REQUIRE(graph.edgesOfType(EdgeType::Loop).size() == 1);
}

TEST_CASE("PoseGraph 批量更新位姿", "[pose_graph]") {
    PoseGraph graph;
    graph.addNode({1, 0.0, SE3d{}});
    graph.addNode({2, 0.1, SE3d{}});

    Eigen::Matrix<double, 6, 1> delta;
    delta << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    SE3d shifted = SE3d::Tangent(delta).exp();

    graph.updatePoses({{1, shifted}});
    auto node = graph.getNode(1);
    REQUIRE(node.has_value());
    REQUIRE(node->pose.translation().x() > 0.9);
}

TEST_CASE("PoseGraph snapshot 深拷贝", "[pose_graph]") {
    PoseGraph graph;
    graph.addNode({1, 0.0, SE3d{}});
    graph.addEdge({1, 1, SE3d{}, Mat6d::Identity(), EdgeType::Odometry});

    auto snap = graph.snapshot();
    REQUIRE(snap.nodes.size() == 1);
    REQUIRE(snap.edges.size() == 1);
}
