#include <SimpleSLAM/core/math/icp_solver.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using namespace simpleslam;

TEST_CASE("IcpSolver: zero residuals give zero update", "[icp_solver]") {
    IcpSolver solver;

    MatchResult result;
    result.residuals = {0.0, 0.0, 0.0};
    result.jacobians = {1, 0, 0, 0, 0, 0,
                        0, 1, 0, 0, 0, 0,
                        0, 0, 1, 0, 0, 0};
    result.num_valid = 3;

    auto dx = solver.solveOneStep(result);
    REQUIRE(dx.norm() < 1e-12);
}

TEST_CASE("IcpSolver: known translation residual", "[icp_solver]") {
    IcpSolver solver;

    MatchResult result;
    // p̂ = (1,0,0)，残差 r = (0.1, 0, 0)
    // J = [I₃ | -[p̂]×]，p̂=(1,0,0) → -[p̂]× 后三列 = [0,0,0; 0,0,1; 0,-1,0]
    result.residuals = {0.1, 0.0, 0.0};
    result.jacobians = {1, 0, 0, 0, 0,  0,
                        0, 1, 0, 0, 0,  1,
                        0, 0, 1, 0, -1, 0};
    result.num_valid = 3;

    auto dx = solver.solveOneStep(result);

    REQUIRE_THAT(dx(0), Catch::Matchers::WithinAbs(-0.1, 1e-10));
    REQUIRE_THAT(dx(1), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(dx(2), Catch::Matchers::WithinAbs(0.0, 1e-10));
}

TEST_CASE("IcpSolver: multiple matches accumulate", "[icp_solver]") {
    IcpSolver solver;

    MatchResult result;
    for (int k = 0; k < 2; ++k) {
        double px = static_cast<double>(k + 1);
        result.residuals.insert(result.residuals.end(), {0.1, 0.0, 0.0});
        result.jacobians.insert(result.jacobians.end(), {1, 0, 0, 0, 0, 0});
        result.jacobians.insert(result.jacobians.end(), {0, 1, 0, 0, 0, px});
        result.jacobians.insert(result.jacobians.end(), {0, 0, 1, 0, -px, 0});
    }
    result.num_valid = 6;

    auto dx = solver.solveOneStep(result);
    REQUIRE_THAT(dx(0), Catch::Matchers::WithinAbs(-0.1, 1e-6));
}

TEST_CASE("IcpSolver: Geman-McClure downweights outliers", "[icp_solver]") {
    IcpConfig cfg;
    cfg.kernel = RobustKernel::GemanMcClure;
    cfg.kernel_scale = 0.01;
    IcpSolver solver(cfg);

    MatchResult result;
    // 内点：残差 0.01
    result.residuals.insert(result.residuals.end(), {0.01, 0.0, 0.0});
    result.jacobians.insert(result.jacobians.end(), {1, 0, 0, 0, 0, 0});
    result.jacobians.insert(result.jacobians.end(), {0, 1, 0, 0, 0, 0});
    result.jacobians.insert(result.jacobians.end(), {0, 0, 1, 0, 0, 0});

    // 外点：残差 10.0
    result.residuals.insert(result.residuals.end(), {10.0, 0.0, 0.0});
    result.jacobians.insert(result.jacobians.end(), {1, 0, 0, 0, 0, 0});
    result.jacobians.insert(result.jacobians.end(), {0, 1, 0, 0, 0, 0});
    result.jacobians.insert(result.jacobians.end(), {0, 0, 1, 0, 0, 0});

    result.num_valid = 6;

    auto dx = solver.solveOneStep(result);
    REQUIRE(std::abs(dx(0)) < 0.5);
}

TEST_CASE("IcpSolver: empty result gives zero update", "[icp_solver]") {
    IcpSolver solver;
    MatchResult result;
    result.num_valid = 0;

    auto dx = solver.solveOneStep(result);
    REQUIRE(dx.allFinite());
}
