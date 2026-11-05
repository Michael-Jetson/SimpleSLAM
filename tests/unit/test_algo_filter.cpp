// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

// 扫描滤波：契约模板（每个内置实现自动受测）与体素降采样的具体行为。

#include <SimpleSLAM/algo/builtins.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

using namespace simpleslam;
using algo::BuiltinScanFilters;
using algo::PassThrough;
using algo::ScanFilter;
using algo::VoxelDownsample;

namespace {

/// 一小团带全部可选列的点：坐标在 [-2, 2) 里，有正有负，跨多个体素。
fw::PointCloud sampleCloud() {
    fw::PointCloud c;
    for (int i = 0; i < 200; ++i) {
        const float x = static_cast<float>((i * 37) % 40) * 0.1F - 2.0F;
        const float y = static_cast<float>((i * 11) % 40) * 0.1F - 2.0F;
        const float z = static_cast<float>((i * 7) % 20) * 0.05F;
        c.push(x, y, z);
        c.intensity.push_back(static_cast<float>(i));
        c.time_offset_ns.push_back(i * 1000);
        c.ring.push_back(static_cast<std::uint16_t>(i % 16));
    }
    return c;
}

/// 故意写错的实现：输出的强度列比坐标多一个。接缝处的检查要把它拦下。
class RaggedFilter final : public ScanFilter {
public:
    [[nodiscard]] std::string_view typeName() const noexcept override { return "ragged"; }

private:
    [[nodiscard]] fw::PointCloud doApply(const fw::PointCloud& in) const override {
        fw::PointCloud out = in;
        out.intensity.assign(out.size() + 1, 1.0F);
        return out;
    }
};

bool sameCloud(const fw::PointCloud& a, const fw::PointCloud& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.intensity == b.intensity &&
           a.time_offset_ns == b.time_offset_ns && a.ring == b.ring;
}

}  // namespace

// ── 契约：所有扫描滤波都要满足 ──

TEMPLATE_LIST_TEST_CASE("扫描滤波契约", "[contract][filter]", BuiltinScanFilters) {
    const TestType filter(typename TestType::Config{});
    const ScanFilter& f = filter;  // 经接口调用，和算法里的用法一样
    CHECK(f.typeName() == TestType::kTypeName);

    SECTION("空进空出") { CHECK(f.apply(fw::PointCloud{}).empty()); }

    SECTION("输出合法、不多于输入，输入有的可选列输出也有") {
        const fw::PointCloud in = sampleCloud();
        const fw::PointCloud out = f.apply(in);
        CHECK(out.consistent());
        CHECK(out.size() <= in.size());
        REQUIRE_FALSE(out.empty());
        CHECK(out.intensity.size() == out.size());
        CHECK(out.time_offset_ns.size() == out.size());
        CHECK(out.ring.size() == out.size());
    }

    SECTION("没有可选列的输入，输出也没有") {
        fw::PointCloud in;
        in.push(0.0F, 0.0F, 0.0F);
        in.push(5.0F, 5.0F, 5.0F);
        const fw::PointCloud out = f.apply(in);
        CHECK(out.intensity.empty());
        CHECK(out.time_offset_ns.empty());
        CHECK(out.ring.empty());
    }

    SECTION("同样的输入得到同样的输出") {
        const fw::PointCloud in = sampleCloud();
        CHECK(sameCloud(f.apply(in), f.apply(in)));
    }

    SECTION("拒绝不合法的输入") {
        fw::PointCloud ragged = sampleCloud();
        ragged.intensity.pop_back();
        CHECK_THROWS_AS(f.apply(ragged), std::invalid_argument);

        fw::PointCloud nan = sampleCloud();
        nan.y[3] = std::numeric_limits<float>::quiet_NaN();
        CHECK_THROWS_AS(f.apply(nan), std::invalid_argument);

        fw::PointCloud inf = sampleCloud();
        inf.z[0] = std::numeric_limits<float>::infinity();
        CHECK_THROWS_AS(f.apply(inf), std::invalid_argument);
    }
}

TEST_CASE("实现的输出不合法：接缝处报实现的错，不是输入的错", "[filter]") {
    const RaggedFilter f;
    fw::PointCloud in;
    in.push(0.0F, 0.0F, 0.0F);
    // std::invalid_argument 也是 std::logic_error，所以先排除它
    try {
        (void)f.apply(in);
        FAIL("没有抛异常");
    } catch (const std::invalid_argument& e) {
        FAIL(e.what());
    } catch (const std::logic_error& e) {
        CHECK(std::string(e.what()).find("ragged") != std::string::npos);
    }
}

// ── 具体实现 ──

TEST_CASE("none 原样输出", "[filter]") {
    const PassThrough f;
    const fw::PointCloud in = sampleCloud();
    CHECK(sameCloud(f.apply(in), in));
}

TEST_CASE("体素降采样：每个体素留第一个落进来的点", "[filter]") {
    const VoxelDownsample f({.voxel_size_m = 1.0});

    fw::PointCloud in;
    in.push(0.2F, 0.2F, 0.2F);   // 体素 (0,0,0)，第一个
    in.push(0.8F, 0.1F, 0.5F);   // 同一个体素，丢掉
    in.push(1.5F, 0.0F, 0.0F);   // 体素 (1,0,0)
    in.push(-0.1F, 0.0F, 0.0F);  // 体素 (-1,0,0)：向下取整，不是向零取整
    in.push(0.9F, 0.9F, 0.9F);   // 体素 (0,0,0)，丢掉
    in.intensity = {10.0F, 11.0F, 12.0F, 13.0F, 14.0F};

    const fw::PointCloud out = f.apply(in);
    REQUIRE(out.size() == 3);
    // 保持输入顺序，可选列跟着保留下来的点走
    CHECK(out.x == std::vector<float>{0.2F, 1.5F, -0.1F});
    CHECK(out.intensity == std::vector<float>{10.0F, 12.0F, 13.0F});
}

TEST_CASE("体素降采样：输出里任意两点不在同一个体素", "[filter]") {
    const double voxel = 0.3;
    const double inv = 1.0 / voxel;
    const VoxelDownsample f({.voxel_size_m = voxel});
    const fw::PointCloud out = f.apply(sampleCloud());
    const auto cell = [&](float v) { return std::floor(static_cast<double>(v) * inv); };
    for (std::size_t i = 0; i < out.size(); ++i) {
        for (std::size_t j = i + 1; j < out.size(); ++j) {
            const bool same = cell(out.x[i]) == cell(out.x[j]) &&
                              cell(out.y[i]) == cell(out.y[j]) && cell(out.z[i]) == cell(out.z[j]);
            CHECK_FALSE(same);
        }
    }
}

TEST_CASE("体素降采样：远处的点不会因整数溢出出错", "[filter]") {
    const VoxelDownsample f({.voxel_size_m = 0.001});
    fw::PointCloud in;
    in.push(1e30F, -1e30F, 0.0F);
    in.push(0.0F, 0.0F, 0.0F);
    CHECK(f.apply(in).size() == 2);
}

TEST_CASE("体素降采样：体素边长必须是正的有限数", "[filter]") {
    for (const double bad : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()}) {
        CHECK_THROWS_AS(VoxelDownsample({.voxel_size_m = bad}), std::invalid_argument);
    }
}
