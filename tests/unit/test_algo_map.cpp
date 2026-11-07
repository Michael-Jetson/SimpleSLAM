// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

// 配准地图：鲁棒核、契约模板（每个内置地图自动受测）与体素点地图的具体行为。

#include <SimpleSLAM/algo/builtins.hpp>
#include <SimpleSLAM/testing/map_contract.hpp>

#include <Eigen/Geometry>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string_view>

using namespace simpleslam;
using algo::BuiltinMaps;
using algo::LinearizableMap;
using algo::LinearizedSystem;
using algo::MatchOptions;
using algo::RobustKernel;
using algo::Vec6d;
using algo::VoxelPointMap;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using testing::expLeft;
using testing::twist;
using Kind = RobustKernel::Kind;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr std::array kKernels{Kind::kNone, Kind::kHuber, Kind::kGemanMcClure};

const SE3d kIdentity = SE3d::Identity();

fw::PointCloud single(double x, double y, double z) {
    fw::PointCloud c;
    c.push(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    return c;
}

/// 故意写错的实现：接缝处的检查要把它拦下。
class BrokenMap final : public LinearizableMap {
public:
    enum class Fault { kNonFiniteCost, kNonZeroWithoutRows };
    explicit BrokenMap(Fault fault) : fault_(fault) {}

    [[nodiscard]] bool empty() const noexcept override { return false; }
    [[nodiscard]] std::size_t size() const noexcept override { return 1; }
    [[nodiscard]] std::string_view typeName() const noexcept override { return "broken"; }
    void clear() override {}

private:
    [[nodiscard]] LinearizedSystem doLinearize(const fw::PointCloud& /*points*/,
                                               const SE3d& /*T_map_sensor*/,
                                               const MatchOptions& /*options*/) const override {
        LinearizedSystem out;
        if (fault_ == Fault::kNonFiniteCost) {
            out.cost = kNaN;
            out.rows = 3;
        } else {
            out.H = Mat6d::Identity();  // 没有约束却给了单位阵
        }
        return out;
    }
    void doInsert(const fw::PointCloud& /*points*/, const SE3d& /*T_map_sensor*/) override {}
    void doMaintain(const Vec3d& /*center*/) override {}

    Fault fault_;
};

/// 抛的是 std::logic_error 本身，不是它的子类 std::invalid_argument（那是输入的错）。
template <class F>
bool throwsImplementationError(F&& f) {
    try {
        (void)f();
    } catch (const std::invalid_argument&) {
        return false;
    } catch (const std::logic_error&) {
        return true;
    }
    return false;
}

}  // namespace

// ── 鲁棒核 ──

TEST_CASE("鲁棒核：weight 是 rho 对残差平方的导数", "[map][kernel]") {
    for (const Kind kind : kKernels) {
        const RobustKernel k{kind, 0.3};
        CHECK(k.rho(0.0) == 0.0);
        CHECK(k.weight(0.0) == 1.0);
        for (const double e2 : {1e-4, 0.01, 0.2, 1.0, 9.0}) {
            const double h = 1e-7 * e2;
            const double fd = (k.rho(e2 + h) - k.rho(e2 - h)) / (2.0 * h);
            CHECK_THAT(k.weight(e2), WithinRel(fd, 1e-6));
        }
    }
}

TEST_CASE("鲁棒核：Huber 在阈值处连续，阈值内是二次", "[map][kernel]") {
    const RobustKernel k{Kind::kHuber, 0.5};
    CHECK(k.rho(0.04) == 0.04);
    CHECK(k.weight(0.04) == 1.0);
    CHECK_THAT(k.rho(0.25 * (1.0 + 1e-12)), WithinRel(0.25, 1e-9));
    CHECK_THAT(k.weight(4.0), WithinRel(0.25, 1e-12));  // s / |e| = 0.5 / 2
}

// ── 契约：所有地图都要满足 ──

TEMPLATE_LIST_TEST_CASE("地图契约", "[contract][map]", BuiltinMaps) {
    testing::checkMapContract<TestType>();
}

TEST_CASE("实现的结果不合法：接缝处报实现的错，不是输入的错", "[map]") {
    const fw::PointCloud points = single(1.0, 0.0, 0.0);
    const BrokenMap nan_cost(BrokenMap::Fault::kNonFiniteCost);
    const BrokenMap identity(BrokenMap::Fault::kNonZeroWithoutRows);
    CHECK(throwsImplementationError([&] { return nan_cost.linearize(points, kIdentity, {}); }));
    CHECK(throwsImplementationError([&] { return identity.linearize(points, kIdentity, {}); }));
}

// ── 体素点地图 ──

TEST_CASE("体素点地图：点到点残差与左扰动雅可比", "[map]") {
    VoxelPointMap map(VoxelPointMap::Config{});
    map.insert(single(1.0, 2.0, 3.0), kIdentity);

    // 查询点 (1.25, 2, 3)：残差 r = (0.25, 0, 0)，J = [I | −[p]×]
    const fw::PointCloud query = single(1.25, 2.0, 3.0);
    const LinearizedSystem sys = map.linearize(query, kIdentity, MatchOptions{});
    REQUIRE(sys.rows == 3);
    const Vec3d p(1.25, 2.0, 3.0);
    Eigen::Matrix<double, 3, 6> J;
    J.leftCols<3>().setIdentity();
    J.rightCols<3>() << 0.0, p.z(), -p.y(), -p.z(), 0.0, p.x(), p.y(), -p.x(), 0.0;
    const Vec3d r(0.25, 0.0, 0.0);
    CHECK((sys.H - J.transpose() * J).norm() < 1e-12);
    CHECK((sys.g - J.transpose() * r).norm() < 1e-12);
    CHECK_THAT(sys.cost, WithinAbs(0.5 * 0.0625, 1e-12));

    // Huber 阈值之外按 s/|e| 降权
    const MatchOptions huber{1.0, {Kind::kHuber, 0.1}};
    const LinearizedSystem weighted = map.linearize(query, kIdentity, huber);
    CHECK((weighted.H - 0.4 * J.transpose() * J).norm() < 1e-12);
}

TEST_CASE("体素点地图：超过对应距离的不算", "[map]") {
    VoxelPointMap map(VoxelPointMap::Config{});
    map.insert(single(0.5, 0.5, 0.5), kIdentity);
    const MatchOptions within_half_meter{0.5, {}};
    CHECK(map.linearize(single(0.5, 0.5, 1.2), kIdentity, within_half_meter).rows == 0);
    CHECK(map.linearize(single(0.5, 0.5, 0.9), kIdentity, within_half_meter).rows == 3);
}

TEST_CASE("体素点地图：插入按位姿变到地图系，按传感器距离筛", "[map]") {
    VoxelPointMap map(VoxelPointMap::Config{.max_range_m = 50.0});
    fw::PointCloud in = single(1.0, 0.0, 0.0);
    in.push(60.0F, 0.0F, 0.0F);  // 离传感器 60 米：不插入
    map.insert(in, expLeft(twist(100.0, 0, 0, 0, 0, 0), kIdentity));
    CHECK(map.size() == 1);
    // 地图系里在 (101, 0, 0)：在那里查询，残差为零
    const LinearizedSystem sys = map.linearize(single(101.0, 0.0, 0.0), kIdentity, MatchOptions{});
    REQUIRE(sys.rows == 3);
    CHECK(sys.cost == 0.0);
}

TEST_CASE("体素点地图：每体素有上限，太近的点不重复存", "[map]") {
    VoxelPointMap map(VoxelPointMap::Config{.voxel_size_m = 1.0, .max_points_per_voxel = 4});
    // 间距平方下限是 1/4，即间距 0.5 米
    fw::PointCloud close;
    close.push(0.1F, 0.1F, 0.1F);
    close.push(0.3F, 0.1F, 0.1F);  // 离第一个点 0.2 米：太近
    map.insert(close, kIdentity);
    CHECK(map.size() == 1);

    // 一个体素里 8 个点，两两间隔 0.6 米
    fw::PointCloud spread;
    for (const float x : {0.05F, 0.65F}) {
        for (const float y : {0.05F, 0.65F}) {
            for (const float z : {0.05F, 0.65F}) spread.push(x, y, z);
        }
    }
    map.clear();
    map.insert(spread, kIdentity);
    CHECK(map.size() == 4);  // 只存上限那么多
}

TEST_CASE("体素点地图：维护删掉离中心太远的体素", "[map]") {
    VoxelPointMap map(VoxelPointMap::Config{.max_range_m = 500.0, .max_distance_m = 50.0});
    fw::PointCloud in = single(1.0, 0.0, 0.0);
    in.push(200.0F, 0.0F, 0.0F);
    map.insert(in, kIdentity);
    REQUIRE(map.size() == 2);
    map.maintain(Vec3d::Zero());
    CHECK(map.size() == 1);
    CHECK(map.linearize(single(200.0, 0.0, 0.0), kIdentity, MatchOptions{}).rows == 0);
    CHECK(map.linearize(single(1.0, 0.0, 0.0), kIdentity, MatchOptions{}).rows == 3);
}

TEST_CASE("体素点地图：配置取值检查", "[map]") {
    const auto make = [](VoxelPointMap::Config c) { return VoxelPointMap(c); };
    CHECK_THROWS_AS(make({.voxel_size_m = 0.0}), std::invalid_argument);
    CHECK_THROWS_AS(make({.max_points_per_voxel = 0}), std::invalid_argument);
    CHECK_THROWS_AS(make({.max_range_m = kNaN}), std::invalid_argument);
    CHECK_THROWS_AS(make({.max_distance_m = -1.0}), std::invalid_argument);
    CHECK_NOTHROW(make({}));
}
