// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 时间体系：时钟域映射、缺映射时显式失败、时间区间覆盖判定。

#include <SimpleSLAM/framework/time.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace simpleslam::fw;

TEST_CASE("缺少映射时转换显式失败，而不是当成同一时钟", "[framework][time]") {
    ClockRegistry registry;
    const ClockDomainId lidar{7};

    CHECK_FALSE(registry.toReference(lidar, Timestamp{1000}).has_value());
    CHECK_FALSE(registry.synchronized(lidar));

    // 参考域自身恒等
    CHECK(registry.toReference(kReferenceClock, Timestamp{1000}) == Timestamp{1000});
    CHECK(registry.synchronized(kReferenceClock));
}

TEST_CASE("仿射映射带版本与不确定度", "[framework][time]") {
    ClockRegistry registry;
    ClockMapping mapping;
    mapping.domain = ClockDomainId{7};
    mapping.scale = 1.0;
    mapping.offset = std::chrono::milliseconds(5);
    mapping.version = 3;
    mapping.uncertainty = std::chrono::microseconds(50);
    mapping.synchronized = true;
    mapping.source = "hardware_trigger";
    registry.setMapping(mapping);

    const auto converted = registry.toReference(ClockDomainId{7}, std::chrono::seconds(1));
    REQUIRE(converted.has_value());
    CHECK(*converted == std::chrono::milliseconds(1005));
    CHECK(registry.synchronized(ClockDomainId{7}));

    // 硬件同步也要有非零不确定度，不默认误差为零
    CHECK(registry.mapping(ClockDomainId{7})->uncertainty.count() > 0);
    CHECK(registry.mapping(ClockDomainId{7})->version == 3);
}

TEST_CASE("时间区间的覆盖与包含", "[framework][time]") {
    const TimeRange outer{Timestamp{100}, Timestamp{200}};
    const TimeRange inner{Timestamp{120}, Timestamp{180}};
    const TimeRange overlapping{Timestamp{150}, Timestamp{250}};

    CHECK(outer.covers(inner));
    CHECK_FALSE(outer.covers(overlapping));
    CHECK(outer.contains(Timestamp{100}));
    CHECK_FALSE(outer.contains(Timestamp{200}));  // 半开区间
    CHECK(outer.span() == Timestamp{100});
    CHECK(TimeRange{Timestamp{5}, Timestamp{5}}.empty());
}
