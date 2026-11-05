// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

// 配准地图：鲁棒核、契约模板（每个内置地图自动受测）与体素点地图的具体行为。

#include <SimpleSLAM/algo/builtins.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <manif/SE3.h>

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
using algo::MapQuery;
using algo::MatchOptions;
using algo::RobustKernel;
using algo::Vec6d;
using algo::VoxelPointMap;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using Kind = RobustKernel::Kind;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::array kKernels{Kind::kNone, Kind::kHuber, Kind::kGemanMcClure};

/// 切向量：平移在前、旋转在后，与 manif 一致。
Vec6d twist(double tx, double ty, double tz, double rx, double ry, double rz) {
    return (Vec6d() << tx, ty, tz, rx, ry, rz).finished();
}

SE3d expLeft(const Vec6d& delta, const SE3d& T) { return manif::SE3Tangentd(delta).exp() * T; }

const SE3d kIdentity = SE3d::Identity();
const SE3d kTruth = expLeft(twist(0.5, -0.3, 0.2, 0.02, -0.01, 0.3), kIdentity);

/// 三块互相垂直、互不相交的平面：地面 z = 0、墙 x = 0、墙 y = 0，各占 [1, 4) 米。
///
/// 三个法向约束全部 6 个自由度。平面互不相交，每个体素里只有一块平面；间距 0.25 米时
/// 一个体素 16 个点、两点间距平方 0.0625，默认配置的体素点地图一个点都不丢。于是在真值处
/// 每个查询点都有零残差的对应，高斯牛顿的不动点就是真值，恢复可以按严格的容差检查。
/// （平面在体素里相交、或者地图做了抽稀时，点到点残差的不动点会偏开真值几厘米，
/// 那是点到点配准本身的偏差，不是契约要守的东西。）
fw::PointCloud planePatches(double spacing = 0.25) {
    fw::PointCloud c;
    const int n = static_cast<int>(3.0 / spacing);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const auto a = static_cast<float>(1.0 + i * spacing);
            const auto b = static_cast<float>(1.0 + j * spacing);
            c.push(a, b, 0.0F);  // 地面
            c.push(0.0F, a, b);  // 墙 x = 0
            c.push(a, 0.0F, b);  // 墙 y = 0
        }
    }
    return c;
}

fw::PointCloud transformed(const fw::PointCloud& in, const SE3d& T) {
    fw::PointCloud out;
    out.reserve(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        const Vec3d p = T.act(Vec3d(in.x[i], in.y[i], in.z[i]));
        out.push(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    }
    return out;
}

fw::PointCloud single(double x, double y, double z) {
    fw::PointCloud c;
    c.push(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    return c;
}

/// 高斯牛顿：解 H δ = −g，左乘更新。只用接口，任何地图实现都适用。
SE3d gaussNewton(const MapQuery& map, const fw::PointCloud& points, SE3d T,
                 const MatchOptions& options, int iterations = 30) {
    for (int k = 0; k < iterations; ++k) {
        const LinearizedSystem sys = map.linearize(points, T, options);
        if (sys.rows < 6) break;
        const Vec6d delta = sys.H.ldlt().solve(-sys.g);
        T = expLeft(delta, T);
        if (delta.norm() < 1e-10) break;
    }
    return T;
}

double translationError(const SE3d& a, const SE3d& b) {
    return (a.translation() - b.translation()).norm();
}

double rotationErrorRad(const SE3d& a, const SE3d& b) {
    return Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle();
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

bool sameSystem(const LinearizedSystem& a, const LinearizedSystem& b) {
    return a.H == b.H && a.g == b.g && a.cost == b.cost && a.rows == b.rows;
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
    TestType map(typename TestType::Config{});
    LinearizableMap& m = map;  // 经接口调用，和算法里的用法一样
    CHECK(m.typeName() == TestType::kTypeName);
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
            const MatchOptions o{1.0, RobustKernel{kind, 0.05}};
            const LinearizedSystem sys = m.linearize(query, T0, o);
            REQUIRE(sys.rows > 6);
            CHECK((sys.H - sys.H.transpose()).norm() <= 1e-12 * sys.H.norm());
            const Eigen::SelfAdjointEigenSolver<Mat6d> eig(sys.H);
            CHECK(eig.eigenvalues().minCoeff() >= -1e-9 * eig.eigenvalues().maxCoeff());

            const double eps = 1e-7;
            const double tolerance = 1e-5 * (1.0 + sys.g.norm());
            for (int i = 0; i < 6; ++i) {
                const Vec6d d = Vec6d::Unit(i) * eps;
                const double up = m.linearize(query, expLeft(d, T0), o).cost;
                const double down = m.linearize(query, expLeft(-d, T0), o).cost;
                INFO("核 " << static_cast<int>(kind) << "，分量 " << i);
                CHECK_THAT(sys.g(i), WithinAbs((up - down) / (2.0 * eps), tolerance));
            }
        }
    }

    SECTION("只用接口的高斯牛顿能恢复小的位姿偏差") {
        m.insert(scene, kIdentity);
        // 平移约 6 厘米、旋转约 0.4°：每个点的位移小于点间距的一半，对应关系一开始就基本正确
        const SE3d T0 = expLeft(twist(0.04, -0.03, 0.03, 0.004, -0.003, 0.005), kTruth);
        const SE3d T = gaussNewton(m, query, T0, options);
        CHECK(translationError(T, kTruth) < 1e-4);
        CHECK(rotationErrorRad(T, kTruth) < 1e-4);
        // 平移恢复：只有平移偏差时同样回到真值
        const SE3d shifted = expLeft(twist(0.08, 0.05, -0.06, 0, 0, 0), kTruth);
        CHECK(translationError(gaussNewton(m, query, shifted, options), kTruth) < 1e-4);
    }

    SECTION("同样的地图与输入，结果逐位相同") {
        TestType other(typename TestType::Config{});
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
