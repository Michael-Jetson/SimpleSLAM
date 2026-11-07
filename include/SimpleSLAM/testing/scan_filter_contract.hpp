// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file scan_filter_contract.hpp
/// 扫描滤波（`algo::ScanFilter`）的契约测试模板。每个实现在自己的测试文件里一行实例化：
///
///     SIMPLESLAM_SCAN_FILTER_CONTRACT(MyFilter)
///
/// 查的是所有扫描滤波都要守的约定：空进空出；输出合法、不多于输入，输入有的可选列输出也有，
/// 输入没有的输出也没有；同样的输入得到同样的输出；不合法的输入被拒绝。

#include <SimpleSLAM/algo/filter/scan_filter.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace simpleslam::testing {

/// 一小团带全部可选列的点：坐标在 [-2, 2) 里，有正有负，跨多个体素。
inline fw::PointCloud sampleCloud() {
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

inline bool sameCloud(const fw::PointCloud& a, const fw::PointCloud& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.intensity == b.intensity &&
           a.time_offset_ns == b.time_offset_ns && a.ring == b.ring;
}

/// 扫描滤波契约。`config` 是被测实现的配置。
template <class F>
void checkScanFilterContract(const typename F::Config& config = {}) {
    const F filter(config);
    const algo::ScanFilter& f = filter;  // 经接口调用，和算法里的用法一样
    CHECK(f.typeName() == F::kTypeName);

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

}  // namespace simpleslam::testing

/// 给一个扫描滤波实现生成契约测试：`SIMPLESLAM_SCAN_FILTER_CONTRACT(MyFilter)`，默认配置。
#define SIMPLESLAM_SCAN_FILTER_CONTRACT(Type)                                      \
    TEST_CASE("扫描滤波契约：" #Type, "[contract][filter]") {                     \
        ::simpleslam::testing::checkScanFilterContract<Type>();                    \
    }
