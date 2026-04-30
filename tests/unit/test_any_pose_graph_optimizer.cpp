#include <SimpleSLAM/core/concepts/any_pose_graph_optimizer.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

using namespace simpleslam;

namespace {

struct MockPGO {
    int odom_edge_count = 0;
    int loop_edge_count = 0;
    int optimize_count = 0;
    int reset_count = 0;
    bool should_converge = true;

    void addOdometryEdge(uint64_t, uint64_t, const SE3d&, const Mat6d&) {
        ++odom_edge_count;
    }

    void addLoopEdge(uint64_t, uint64_t, const SE3d&, const Mat6d&) {
        ++loop_edge_count;
    }

    OptimizationResult optimize() {
        ++optimize_count;
        OptimizationResult result;
        result.converged = should_converge;
        result.iterations = 5;
        result.final_error = 0.01;
        if (should_converge) {
            result.optimized_poses[1] = SE3d{};
            result.optimized_poses[2] = SE3d{};
        }
        return result;
    }

    void reset() { ++reset_count; }
};

}  // namespace

TEST_CASE("AnyPoseGraphOptimizer 转发 addOdometryEdge 和 addLoopEdge", "[any_pgo]") {
    MockPGO mock;
    mock.should_converge = false;
    AnyPoseGraphOptimizer opt(std::move(mock));

    SE3d rel;
    Mat6d info = Mat6d::Identity();
    opt.addOdometryEdge(1, 2, rel, info);
    opt.addLoopEdge(1, 5, rel, info);

    auto result = opt.optimize();
    REQUIRE(result.iterations == 5);
}

TEST_CASE("AnyPoseGraphOptimizer optimize 返回正确结果", "[any_pgo]") {
    AnyPoseGraphOptimizer opt(MockPGO{});

    auto result = opt.optimize();
    REQUIRE(result.converged);
    REQUIRE(result.optimized_poses.size() == 2);
    REQUIRE(result.final_error == Catch::Approx(0.01));
}

TEST_CASE("AnyPoseGraphOptimizer reset 转发", "[any_pgo]") {
    AnyPoseGraphOptimizer opt(MockPGO{});
    opt.reset();
    REQUIRE(opt.valid());
}

TEST_CASE("AnyPoseGraphOptimizer move 语义", "[any_pgo]") {
    AnyPoseGraphOptimizer a(MockPGO{});
    REQUIRE(a.valid());

    AnyPoseGraphOptimizer b = std::move(a);
    REQUIRE(b.valid());
    REQUIRE_FALSE(a.valid());

    auto result = b.optimize();
    REQUIRE(result.converged);
}

TEST_CASE("AnyPoseGraphOptimizer 满足 PoseGraphOptimizer concept", "[any_pgo]") {
    static_assert(PoseGraphOptimizer<AnyPoseGraphOptimizer>);
    REQUIRE(true);
}
