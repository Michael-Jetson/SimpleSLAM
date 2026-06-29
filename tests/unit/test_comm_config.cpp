#include <SimpleSLAM/core/infra/comm_config.hpp>

#include <catch2/catch_test_macros.hpp>

#include <yaml-cpp/yaml.h>

using namespace simpleslam;

TEST_CASE("parseQoS / parseDropPolicy / parseDeliveryMode", "[comm_config]") {
    REQUIRE(parseQoS("event") == QoS::Event);
    REQUIRE(parseQoS("stream") == QoS::Stream);
    REQUIRE(parseQoS("latest") == QoS::Latest);
    REQUIRE_THROWS(parseQoS("bogus"));

    REQUIRE(parseDropPolicy("oldest") == DropPolicy::DropOldest);
    REQUIRE(parseDropPolicy("newest") == DropPolicy::DropNewest);
    REQUIRE_THROWS(parseDropPolicy("bogus"));

    REQUIRE(parseDeliveryMode("sync") == DeliveryMode::Sync);
    REQUIRE(parseDeliveryMode("async") == DeliveryMode::Async);
    REQUIRE_THROWS(parseDeliveryMode("bogus"));
}

TEST_CASE("parseSubscribeOptions 读全字段", "[comm_config]") {
    auto node = YAML::Load("{queue_depth: 5, drop: newest, throttle: 2, delivery: async}");
    auto opts = parseSubscribeOptions(node);
    REQUIRE(opts.queue_depth == 5);
    REQUIRE(opts.drop == DropPolicy::DropNewest);
    REQUIRE(opts.throttle_every == 2);
    REQUIRE(opts.delivery == DeliveryMode::Async);
}

TEST_CASE("loadTopicSpec 读 name+qos+options", "[comm_config]") {
    auto node = YAML::Load("{topic: sensor/lidar, qos: stream, queue_depth: 3, delivery: async}");
    auto spec = loadTopicSpec(node);
    REQUIRE(spec.name == "sensor/lidar");
    REQUIRE(spec.qos == QoS::Stream);
    REQUIRE(spec.options.queue_depth == 3);
    REQUIRE(spec.options.delivery == DeliveryMode::Async);
}

TEST_CASE("loadTopicSpec 字段缺省取默认", "[comm_config]") {
    auto node = YAML::Load("{topic: slam/x}");
    auto spec = loadTopicSpec(node);
    REQUIRE(spec.name == "slam/x");
    REQUIRE(spec.qos == QoS::Event);                      // 默认
    REQUIRE(spec.options.drop == DropPolicy::DropOldest);  // 默认
    REQUIRE(spec.options.throttle_every == 1);            // 默认
    REQUIRE(spec.options.delivery == DeliveryMode::Sync); // 默认
}

TEST_CASE("loadTopicSpec 缺 topic 字段抛异常", "[comm_config]") {
    auto node = YAML::Load("{qos: event}");
    REQUIRE_THROWS(loadTopicSpec(node));
}
