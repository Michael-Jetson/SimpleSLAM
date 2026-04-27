/// @file test_geometry.cpp
/// 几何类型测试——验证 manif 别名和基本运算
///
/// manif API 要点（与 Sophus 的主要区别）：
///   - 指数映射：SE3d::Tangent(vec).exp()，不是 SE3d::Exp(vec)
///   - 作用于点：T.act(p)，不是 T * p
///   - 齐次矩阵：T.transform()，不是 T.matrix()
///   - 逆：T.inverse()（同 Sophus）
///   - 组合：T1 * T2 或 T1.compose(T2)

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <SimpleSLAM/core/types/common.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>

#include <numbers>

using namespace simpleslam;
using Catch::Matchers::WithinAbs;

TEST_CASE("SE3d 单位变换", "[geometry]") {
    SE3d identity = SE3d::Identity();
    Vec3d p(1.0, 2.0, 3.0);

    Vec3d result = identity.act(p);
    REQUIRE_THAT(result.x(), WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(result.y(), WithinAbs(2.0, 1e-12));
    REQUIRE_THAT(result.z(), WithinAbs(3.0, 1e-12));
}

TEST_CASE("SE3d 平移", "[geometry]") {
    // manif SE3d tangent: [tx, ty, tz, rx, ry, rz]（平移在前，旋转在后）
    Eigen::Matrix<double, 6, 1> tau;
    tau << 1.0, 2.0, 3.0, 0.0, 0.0, 0.0;
    SE3d T = SE3d::Tangent(tau).exp();

    Vec3d result = T.act(Vec3d::Zero());
    REQUIRE_THAT(result.x(), WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(result.y(), WithinAbs(2.0, 1e-10));
    REQUIRE_THAT(result.z(), WithinAbs(3.0, 1e-10));
}

TEST_CASE("SE3d compose 和 inverse", "[geometry]") {
    Eigen::Matrix<double, 6, 1> tau;
    tau << 1.0, 0.5, -0.3, 0.1, -0.2, 0.15;
    SE3d T = SE3d::Tangent(tau).exp();
    SE3d identity = T * T.inverse();

    // 平移和旋转都应接近零/单位
    REQUIRE_THAT(identity.translation().norm(), WithinAbs(0.0, 1e-10));
    REQUIRE_THAT((identity.rotation() - Mat3d::Identity()).norm(), WithinAbs(0.0, 1e-10));
}

TEST_CASE("SE3d 与 Mat4d 互转", "[geometry]") {
    Eigen::Matrix<double, 6, 1> tau;
    tau << 0.5, -1.0, 2.0, 0.1, 0.2, -0.1;
    SE3d T = SE3d::Tangent(tau).exp();

    Mat4d mat = T.transform();

    // 齐次矩阵最后一行 = [0 0 0 1]
    REQUIRE_THAT(mat(3, 0), WithinAbs(0.0, 1e-15));
    REQUIRE_THAT(mat(3, 3), WithinAbs(1.0, 1e-15));

    // 旋转部分为正交矩阵：R^T * R = I
    Mat3d R = mat.block<3, 3>(0, 0);
    REQUIRE_THAT((R.transpose() * R - Mat3d::Identity()).norm(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("SO3d 绕 Z 轴旋转 90 度", "[geometry]") {
    Eigen::Vector3d w{0.0, 0.0, std::numbers::pi / 2.0};
    SO3d R = SO3d::Tangent(w).exp();

    // (1,0,0) 绕 z 轴转 90 度 → (0,1,0)
    Vec3d result = R.act(Vec3d{1.0, 0.0, 0.0});
    REQUIRE_THAT(result.x(), WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(result.y(), WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(result.z(), WithinAbs(0.0, 1e-10));
}

TEST_CASE("Mat6d 类型别名", "[geometry]") {
    Mat6d mat = Mat6d::Identity();
    REQUIRE(mat.rows() == 6);
    REQUIRE(mat.cols() == 6);
    REQUIRE_THAT(mat(0, 0), WithinAbs(1.0, 1e-15));
    REQUIRE_THAT(mat(0, 5), WithinAbs(0.0, 1e-15));
}

TEST_CASE("PointXYZI 构造与内存布局", "[common]") {
    PointXYZI p;
    REQUIRE(p.x == 0.0f);
    REQUIRE(p.intensity == 0.0f);

    PointXYZI q{1.0f, 2.0f, 3.0f, 0.5f};
    REQUIRE(q.z == 3.0f);

    // 内存布局保证（与 GPU/PCL 互操作需要）
    static_assert(sizeof(PointXYZI) == 16);
    static_assert(std::is_trivially_copyable_v<PointXYZI>);
}

TEST_CASE("Timestamp 类型", "[common]") {
    Timestamp t = 1714100000.123456;
    REQUIRE(t > 0.0);
    REQUIRE_THAT(t, WithinAbs(1714100000.123456, 1e-6));
}
