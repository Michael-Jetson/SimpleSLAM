#include <SimpleSLAM/resources/keyframe_store.hpp>

#include <catch2/catch_test_macros.hpp>
#include <string>

using namespace simpleslam;

TEST_CASE("KeyframeStore insert 和 get", "[keyframe_store]") {
    KeyframeStore store;
    KeyframeData kf;
    kf.id = 1;
    kf.timestamp = 10.0;
    store.insert(std::move(kf));

    REQUIRE(store.size() == 1);
    REQUIRE(store.contains(1));

    auto ptr = store.get(1);
    REQUIRE(ptr != nullptr);
    REQUIRE(ptr->id == 1);
    REQUIRE(ptr->timestamp == 10.0);
}

TEST_CASE("KeyframeStore 重复 ID 抛异常", "[keyframe_store]") {
    KeyframeStore store;
    KeyframeData kf;
    kf.id = 1;
    store.insert(kf);
    REQUIRE_THROWS(store.insert(kf));
}

TEST_CASE("KeyframeStore get 不存在返回 nullptr", "[keyframe_store]") {
    KeyframeStore store;
    REQUIRE(store.get(999) == nullptr);
}

TEST_CASE("KeyframeStore latest 返回最近插入", "[keyframe_store]") {
    KeyframeStore store;
    KeyframeData kf1, kf2;
    kf1.id = 1;
    kf2.id = 2;
    store.insert(std::move(kf1));
    store.insert(std::move(kf2));

    auto latest = store.latest();
    REQUIRE(latest != nullptr);
    REQUIRE(latest->id == 2);
}

TEST_CASE("KeyframeStore extension 扩展属性", "[keyframe_store]") {
    KeyframeStore store;
    KeyframeData kf;
    kf.id = 1;
    store.insert(std::move(kf));

    store.setExtension(1, "score", std::any(0.95));
    REQUIRE(store.hasExtension(1, "score"));
    REQUIRE_FALSE(store.hasExtension(1, "nonexistent"));

    auto* val = store.getExtension<double>(1, "score");
    REQUIRE(val != nullptr);
    REQUIRE(*val == 0.95);
}

TEST_CASE("KeyframeStore extension 重复 key 抛异常", "[keyframe_store]") {
    KeyframeStore store;
    KeyframeData kf;
    kf.id = 1;
    store.insert(std::move(kf));

    store.setExtension(1, "tag", std::any(42));
    REQUIRE_THROWS(store.setExtension(1, "tag", std::any(99)));
}

TEST_CASE("KeyframeStore allIds", "[keyframe_store]") {
    KeyframeStore store;
    for (uint64_t i = 0; i < 5; ++i) {
        KeyframeData kf;
        kf.id = i;
        store.insert(std::move(kf));
    }
    REQUIRE(store.allIds().size() == 5);
}
