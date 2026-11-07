// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file map_contract.hpp
/// 配准地图（`algo::LinearizableMap`）的契约测试模板。每个实现在自己的测试文件里一行实例化：
///
///     SIMPLESLAM_MAP_CONTRACT(MyMap)
///
/// 查的是所有地图都要守的约定，不查精度以外的实现细节：退化输入（空图、没有点、远离地图、
/// 共面、极稀疏）不崩溃、不出 NaN，没有约束时 0 行；H 对称半正定，g 是 cost 的梯度；只用接口
/// 的高斯牛顿能恢复小的位姿偏差；插入、清空、维护之后照常查询；结果逐位确定；不合法的输入
/// 被拒绝。近似实现（近似近邻等）用 `checkMapContract` 放宽恢复精度。

#include <SimpleSLAM/algo/map/map.hpp>
#include <SimpleSLAM/testing/scenes.hpp>

#include <Eigen/Eigenvalues>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace simpleslam::testing {

/// 恢复小的位姿偏差时允许的误差。默认按精确的实现查；近似实现可以放宽。
struct MapContractTolerance {
    double translation_m = 1e-4;
    double rotation_rad = 1e-4;
};

/// 高斯牛顿：解 H δ = −g，左乘更新。只用接口，任何地图实现都适用。
inline SE3d gaussNewton(const algo::MapQuery& map, const fw::PointCloud& points, SE3d T,
                        const algo::MatchOptions& options, int iterations = 30) {
    for (int k = 0; k < iterations; ++k) {
        const algo::LinearizedSystem sys = map.linearize(points, T, options);
        if (sys.rows < 6) break;
        const Vec6d delta = sys.H.ldlt().solve(-sys.g);
        T = expLeft(delta, T);
        if (delta.norm() < 1e-10) break;
    }
    return T;
}

inline bool sameSystem(const algo::LinearizedSystem& a, const algo::LinearizedSystem& b) {
    return a.H == b.H && a.g == b.g && a.cost == b.cost && a.rows == b.rows;
}

namespace detail {

/// 退化输入的结果：有限，H 对称半正定。不要求有约束，也不要求精度。
inline void checkStable(const algo::LinearizedSystem& sys) {
    CHECK(sys.H.allFinite());
    CHECK(sys.g.allFinite());
    CHECK(std::isfinite(sys.cost));
    if (sys.rows == 0) {
        CHECK(sys.H.isZero(0.0));
        return;
    }
    CHECK((sys.H - sys.H.transpose()).norm() <= 1e-12 * (1.0 + sys.H.norm()));
    const Eigen::SelfAdjointEigenSolver<Mat6d> eig(sys.H);
    CHECK(eig.eigenvalues().minCoeff() >= -1e-9 * (1.0 + eig.eigenvalues().maxCoeff()));
}

}  // namespace detail

/// 地图契约。`config` 是被测实现的配置，`tolerance` 是恢复精度。
template <class M>
void checkMapContract(const typename M::Config& config = {}, MapContractTolerance tolerance = {}) {
    using algo::LinearizedSystem;
    using algo::MatchOptions;
    using Kind = algo::RobustKernel::Kind;
    using Catch::Matchers::WithinAbs;
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    constexpr double kInf = std::numeric_limits<double>::infinity();
    constexpr std::array kKernels{Kind::kNone, Kind::kHuber, Kind::kGemanMcClure};
    const SE3d kIdentity = SE3d::Identity();
    const SE3d kTruth = expLeft(twist(0.5, -0.3, 0.2, 0.02, -0.01, 0.3), kIdentity);

    M map(config);
    algo::LinearizableMap& m = map;  // 经接口调用，和算法里的用法一样
    CHECK(m.typeName() == M::kTypeName);
    const fw::PointCloud scene = planePatches();
    const fw::PointCloud query = transformed(scene, kTruth.inverse());  // 传感器系的点
    const MatchOptions options{};

    SECTION("空图、没有点、远离地图：0 行，H、g、cost 为零，不是单位阵") {
        CHECK(m.empty());
        const LinearizedSystem empty_map = m.linearize(query, kTruth, options);
        CHECK(empty_map.rows == 0);
        CHECK(empty_map.H.isZero(0.0));
        CHECK(empty_map.g.isZero(0.0));
        CHECK(empty_map.cost == 0.0);

        m.insert(scene, kIdentity);
        CHECK(m.linearize(fw::PointCloud{}, kTruth, options).rows == 0);
        const SE3d far = expLeft(twist(1000.0, 0, 0, 0, 0, 0), kTruth);
        const LinearizedSystem far_away = m.linearize(query, far, options);
        CHECK(far_away.rows == 0);
        CHECK(far_away.H.isZero(0.0));
    }

    SECTION("共面、极稀疏：不要求精度，结果有限，H 对称半正定") {
        // 共面（走廊、隧道）：所有法向平行，H 秩亏，但不能有 NaN
        const fw::PointCloud plane = coplanar();
        m.insert(plane, kIdentity);
        const SE3d near = expLeft(twist(0.05, 0.04, 0.03, 0.01, -0.01, 0.02), kTruth);
        detail::checkStable(m.linearize(transformed(plane, kTruth.inverse()), near, options));

        // 极稀疏：地图与查询都只有几个点
        m.clear();
        const fw::PointCloud few = sparse();
        m.insert(few, kIdentity);
        detail::checkStable(m.linearize(transformed(few, kTruth.inverse()), near, options));
        detail::checkStable(m.linearize(sparse(1), kTruth, options));
    }

    SECTION("插入之后非空；clear 之后为空") {
        m.insert(scene, kIdentity);
        CHECK_FALSE(m.empty());
        CHECK(m.size() > 0);
        m.clear();
        CHECK(m.empty());
        CHECK(m.size() == 0);
        CHECK(m.linearize(query, kTruth, options).rows == 0);
    }

    SECTION("H 对称半正定，g 是 cost 的梯度") {
        m.insert(scene, kIdentity);
        const SE3d T0 = expLeft(twist(0.05, -0.04, 0.03, 0.01, -0.008, 0.012), kTruth);
        for (const Kind kind : kKernels) {
            const MatchOptions o{1.0, algo::RobustKernel{kind, 0.05}};
            const LinearizedSystem sys = m.linearize(query, T0, o);
            REQUIRE(sys.rows > 6);
            CHECK((sys.H - sys.H.transpose()).norm() <= 1e-12 * sys.H.norm());
            const Eigen::SelfAdjointEigenSolver<Mat6d> eig(sys.H);
            CHECK(eig.eigenvalues().minCoeff() >= -1e-9 * eig.eigenvalues().maxCoeff());

            const double eps = 1e-7;
            const double slack = 1e-5 * (1.0 + sys.g.norm());
            for (int i = 0; i < 6; ++i) {
                const Vec6d d = Vec6d::Unit(i) * eps;
                const double up = m.linearize(query, expLeft(d, T0), o).cost;
                const double down = m.linearize(query, expLeft(-d, T0), o).cost;
                INFO("核 " << static_cast<int>(kind) << "，分量 " << i);
                CHECK_THAT(sys.g(i), WithinAbs((up - down) / (2.0 * eps), slack));
            }
        }
    }

    SECTION("只用接口的高斯牛顿能恢复小的位姿偏差") {
        m.insert(scene, kIdentity);
        // 平移约 6 厘米、旋转约 0.4°：每个点的位移小于点间距的一半，对应关系一开始就基本正确
        const SE3d T0 = expLeft(twist(0.04, -0.03, 0.03, 0.004, -0.003, 0.005), kTruth);
        const SE3d T = gaussNewton(m, query, T0, options);
        CHECK(translationError(T, kTruth) < tolerance.translation_m);
        CHECK(rotationErrorRad(T, kTruth) < tolerance.rotation_rad);
        // 只有平移偏差时同样回到真值
        const SE3d shifted = expLeft(twist(0.08, 0.05, -0.06, 0, 0, 0), kTruth);
        CHECK(translationError(gaussNewton(m, query, shifted, options), kTruth)
              < tolerance.translation_m);
    }

    SECTION("同样的地图与输入，结果逐位相同") {
        M other(config);
        m.insert(scene, kIdentity);
        other.insert(scene, kIdentity);
        const SE3d T0 = expLeft(twist(0.1, 0, 0, 0, 0, 0.01), kTruth);
        const LinearizedSystem a = m.linearize(query, T0, options);
        CHECK(sameSystem(a, m.linearize(query, T0, options)));
        CHECK(sameSystem(a, other.linearize(query, T0, options)));
    }

    SECTION("维护不增加点数，维护之后照常查询") {
        m.insert(scene, kIdentity);
        const std::size_t before = m.size();
        m.maintain(Vec3d(2.0, 2.0, 1.0));
        CHECK(m.size() <= before);
        CHECK(m.linearize(query, kTruth, options).H.allFinite());
    }

    SECTION("拒绝不合法的输入") {
        m.insert(scene, kIdentity);
        fw::PointCloud nan = query;
        nan.x[5] = std::numeric_limits<float>::quiet_NaN();
        CHECK_THROWS_AS(m.linearize(nan, kTruth, options), std::invalid_argument);
        CHECK_THROWS_AS(m.insert(nan, kTruth), std::invalid_argument);

        fw::PointCloud ragged = query;
        ragged.intensity = {1.0F};
        CHECK_THROWS_AS(m.linearize(ragged, kTruth, options), std::invalid_argument);

        const SE3d bad_pose(Vec3d(kNaN, 0.0, 0.0), Eigen::Quaterniond::Identity());
        CHECK_THROWS_AS(m.linearize(query, bad_pose, options), std::invalid_argument);
        CHECK_THROWS_AS(m.insert(query, bad_pose), std::invalid_argument);

        const MatchOptions no_distance{0.0, {}};
        const MatchOptions huber_without_scale{1.0, {Kind::kHuber, 0.0}};
        const MatchOptions plain_without_scale{1.0, {Kind::kNone, 0.0}};  // 不加核时尺度不用
        CHECK_THROWS_AS(m.linearize(query, kTruth, no_distance), std::invalid_argument);
        CHECK_THROWS_AS(m.linearize(query, kTruth, huber_without_scale), std::invalid_argument);
        CHECK_NOTHROW(m.linearize(query, kTruth, plain_without_scale));

        CHECK_THROWS_AS(m.maintain(Vec3d(kInf, 0.0, 0.0)), std::invalid_argument);
    }
}

}  // namespace simpleslam::testing

/// 给一个地图实现生成契约测试：`SIMPLESLAM_MAP_CONTRACT(MyMap)`，默认配置、精确的恢复精度。
#define SIMPLESLAM_MAP_CONTRACT(Type)                                              \
    TEST_CASE("地图契约：" #Type, "[contract][map]") {                            \
        ::simpleslam::testing::checkMapContract<Type>();                           \
    }
