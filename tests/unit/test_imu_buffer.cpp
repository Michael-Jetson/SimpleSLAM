#include <SimpleSLAM/odometry/imu_buffer.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

using namespace simpleslam;

namespace {

ImuSample makeSample(Timestamp t, double ax, double gy) {
    ImuSample s;
    s.timestamp = t;
    s.acc = Vec3d(ax, 0.0, 9.81);
    s.gyro = Vec3d(0.0, gy, 0.0);
    return s;
}

}  // namespace

TEST_CASE("ImuBuffer 空 buffer 属性", "[imu_buffer]") {
    ImuBuffer buf;
    REQUIRE(buf.empty());
    REQUIRE(buf.size() == 0);
    REQUIRE_FALSE(buf.oldestTimestamp().has_value());
    REQUIRE_FALSE(buf.newestTimestamp().has_value());
}

TEST_CASE("ImuBuffer addSample 和 size", "[imu_buffer]") {
    ImuBuffer buf;
    buf.addSample(makeSample(1.0, 0.1, 0.01));
    buf.addSample(makeSample(2.0, 0.2, 0.02));

    REQUIRE(buf.size() == 2);
    REQUIRE_FALSE(buf.empty());
    REQUIRE(buf.oldestTimestamp().value() == 1.0);
    REQUIRE(buf.newestTimestamp().value() == 2.0);
}

TEST_CASE("ImuBuffer addBatch 批量添加", "[imu_buffer]") {
    ImuBuffer buf;
    ImuBatch batch = {makeSample(1.0, 0.1, 0.01),
                      makeSample(2.0, 0.2, 0.02),
                      makeSample(3.0, 0.3, 0.03)};
    buf.addBatch(batch);

    REQUIRE(buf.size() == 3);
    REQUIRE(buf.oldestTimestamp().value() == 1.0);
    REQUIRE(buf.newestTimestamp().value() == 3.0);
}

TEST_CASE("ImuBuffer query 返回范围内样本", "[imu_buffer]") {
    ImuBuffer buf;
    buf.addSample(makeSample(1.0, 0.1, 0.01));
    buf.addSample(makeSample(2.0, 0.2, 0.02));
    buf.addSample(makeSample(3.0, 0.3, 0.03));
    buf.addSample(makeSample(4.0, 0.4, 0.04));

    auto result = buf.query(2.0, 3.0);
    REQUIRE(result.size() >= 2);
    REQUIRE(result.front().timestamp <= 2.0);
    REQUIRE(result.back().timestamp >= 3.0);
}

TEST_CASE("ImuBuffer query 空范围返回空", "[imu_buffer]") {
    ImuBuffer buf;
    buf.addSample(makeSample(1.0, 0.1, 0.01));
    buf.addSample(makeSample(2.0, 0.2, 0.02));

    auto result = buf.query(5.0, 6.0);
    REQUIRE(result.empty());
}

TEST_CASE("ImuBuffer interpolateAt 中间时间点", "[imu_buffer]") {
    ImuBuffer buf;
    buf.addSample(makeSample(1.0, 0.0, 0.0));
    buf.addSample(makeSample(3.0, 2.0, 1.0));

    auto result = buf.interpolateAt(2.0);
    REQUIRE(result.has_value());
    REQUIRE(result->timestamp == Catch::Approx(2.0));
    REQUIRE(result->acc.x() == Catch::Approx(1.0));
    REQUIRE(result->gyro.y() == Catch::Approx(0.5));
}

TEST_CASE("ImuBuffer interpolateAt 超范围返回 nullopt", "[imu_buffer]") {
    ImuBuffer buf;
    buf.addSample(makeSample(1.0, 0.1, 0.01));
    buf.addSample(makeSample(2.0, 0.2, 0.02));

    REQUIRE_FALSE(buf.interpolateAt(0.5).has_value());
    REQUIRE_FALSE(buf.interpolateAt(3.0).has_value());
}

TEST_CASE("ImuBuffer trimBefore 裁剪旧数据", "[imu_buffer]") {
    ImuBuffer buf;
    buf.addSample(makeSample(1.0, 0.1, 0.01));
    buf.addSample(makeSample(2.0, 0.2, 0.02));
    buf.addSample(makeSample(3.0, 0.3, 0.03));

    buf.trimBefore(2.0);
    REQUIRE(buf.size() == 2);
    REQUIRE(buf.oldestTimestamp().value() == 2.0);
}

TEST_CASE("ImuBuffer max_size 自动裁剪", "[imu_buffer]") {
    ImuBuffer buf(3);
    buf.addSample(makeSample(1.0, 0.1, 0.01));
    buf.addSample(makeSample(2.0, 0.2, 0.02));
    buf.addSample(makeSample(3.0, 0.3, 0.03));
    REQUIRE(buf.size() == 3);

    buf.addSample(makeSample(4.0, 0.4, 0.04));
    REQUIRE(buf.size() == 3);
    REQUIRE(buf.oldestTimestamp().value() == 2.0);
}
