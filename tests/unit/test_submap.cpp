#include <SimpleSLAM/backend/submap/submap.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace simpleslam;

TEST_CASE("Submap 基本属性", "[submap]") {
    Submap submap;
    submap.id = 1;
    REQUIRE(submap.isEmpty());
    REQUIRE(submap.keyframeCount() == 0);
    REQUIRE_FALSE(submap.isFrozen());
}

TEST_CASE("Submap 添加关键帧", "[submap]") {
    Submap submap;
    submap.keyframe_ids.push_back(10);
    submap.keyframe_ids.push_back(11);
    REQUIRE(submap.keyframeCount() == 2);
    REQUIRE_FALSE(submap.isEmpty());
}

TEST_CASE("Submap 冻结状态", "[submap]") {
    Submap submap;
    submap.frozen = true;
    submap.frozen_at = 100.0;
    REQUIRE(submap.isFrozen());
    REQUIRE(submap.frozen_at == 100.0);
}
