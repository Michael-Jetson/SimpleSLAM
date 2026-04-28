#include <SimpleSLAM/resources/current_state.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace simpleslam;
using Catch::Matchers::WithinAbs;

TEST_CASE("CurrentState odom publish 和 read", "[current_state]") {
    CurrentState state;
    auto snap = state.readOdom();
    REQUIRE(snap == nullptr);

    state.publishOdom(1.0, SE3d{});
    snap = state.readOdom();
    REQUIRE(snap != nullptr);
    REQUIRE_THAT(snap->timestamp, WithinAbs(1.0, 1e-9));
}

TEST_CASE("CurrentState correction publish 和 read", "[current_state]") {
    CurrentState state;
    auto corr = state.readCorrection();
    // 默认校正为单位变换
    REQUIRE_THAT(corr.translation().norm(), WithinAbs(0.0, 1e-9));

    Eigen::Matrix<double, 6, 1> delta;
    delta << 5.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    state.publishCorrection(SE3d::Tangent(delta).exp());

    corr = state.readCorrection();
    REQUIRE_THAT(corr.translation().x(), WithinAbs(5.0, 1e-6));
}

TEST_CASE("CurrentState correctedPose = correction * odom", "[current_state]") {
    CurrentState state;

    Eigen::Matrix<double, 6, 1> d_odom;
    d_odom << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    state.publishOdom(1.0, SE3d::Tangent(d_odom).exp());

    Eigen::Matrix<double, 6, 1> d_corr;
    d_corr << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    state.publishCorrection(SE3d::Tangent(d_corr).exp());

    auto result = state.correctedPose();
    REQUIRE_THAT(result.translation().x(), WithinAbs(3.0, 1e-6));
}
