#include <SimpleSLAM/core/infra/topic.hpp>
#include <SimpleSLAM/core/infra/topic_names.hpp>

#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <string>
#include <vector>

using namespace simpleslam;

TEST_CASE("Publisher + Subscriber 基本发布-订阅", "[topic]") {
    Topic<int> topic("test/basic", QoS::Event);
    Publisher<int> pub(&topic);
    REQUIRE(pub.valid());

    std::vector<int> received;
    auto sub = topic.subscribe([&](MsgPtr<int> msg) {
        received.push_back(*msg);
    });

    pub.publish(42);
    // offline 模式：消息在 pending
    REQUIRE(received.empty());
    REQUIRE(topic.hasPending());

    topic.drainOnce();
    REQUIRE(received.size() == 1);
    REQUIRE(received[0] == 42);
}

TEST_CASE("Publisher emplace 就地构造", "[topic]") {
    Topic<std::string> topic("test/emplace", QoS::Event);
    Publisher<std::string> pub(&topic);

    std::string received;
    auto sub = topic.subscribe([&](MsgPtr<std::string> msg) {
        received = *msg;
    });

    pub.emplace("hello world");
    topic.drainOnce();
    REQUIRE(received == "hello world");
}

TEST_CASE("Publisher 拷贝发布", "[topic]") {
    Topic<int> topic("test/copy", QoS::Event);
    Publisher<int> pub(&topic);

    int received = 0;
    auto sub = topic.subscribe([&](MsgPtr<int> msg) { received = *msg; });

    const int val = 99;
    pub.publish(val);
    topic.drainOnce();
    REQUIRE(received == 99);
}

TEST_CASE("SubscriptionHandle RAII 退订", "[topic]") {
    Topic<int> topic("test/raii", QoS::Event);
    Publisher<int> pub(&topic);
    int count = 0;

    {
        auto sub = topic.subscribe([&](MsgPtr<int>) { ++count; });
        REQUIRE(topic.subscriberCount() == 1);
        pub.publish(1);
        topic.drainOnce();
        REQUIRE(count == 1);
    }
    // sub 析构 → 自动退订
    REQUIRE(topic.subscriberCount() == 0);

    pub.publish(2);
    topic.drainOnce();
    REQUIRE(count == 1);  // 不再收到
}

TEST_CASE("Topic 多订阅者", "[topic]") {
    Topic<int> topic("test/multi", QoS::Event);
    Publisher<int> pub(&topic);
    int count_a = 0, count_b = 0;

    auto sub_a = topic.subscribe([&](MsgPtr<int>) { ++count_a; });
    auto sub_b = topic.subscribe([&](MsgPtr<int>) { ++count_b; });

    pub.publish(1);
    topic.drainOnce();
    REQUIRE(count_a == 1);
    REQUIRE(count_b == 1);
}

TEST_CASE("Topic Latest 只保留最新值", "[topic]") {
    Topic<int> topic("test/latest", QoS::Latest);
    Publisher<int> pub(&topic);

    auto sub = topic.subscribe([](MsgPtr<int>) {});
    pub.publish(1);
    pub.publish(2);
    pub.publish(3);

    topic.drainOnce();
    topic.drainOnce();
    topic.drainOnce();

    auto val = topic.latest();
    REQUIRE(val != nullptr);
    REQUIRE(*val == 3);
}

TEST_CASE("Topic online 模式直接分发", "[topic]") {
    Topic<int> topic("test/online", QoS::Event);
    topic.setOfflineMode(false);
    Publisher<int> pub(&topic);

    int received = 0;
    auto sub = topic.subscribe([&](MsgPtr<int> msg) { received = *msg; });

    pub.publish(99);
    REQUIRE(received == 99);
    REQUIRE_FALSE(topic.hasPending());
}

TEST_CASE("Publisher publisherCount 追踪", "[topic]") {
    Topic<int> topic("test/pubcount", QoS::Event);
    REQUIRE(topic.publisherCount() == 0);

    {
        Publisher<int> pub1(&topic);
        REQUIRE(topic.publisherCount() == 1);
        Publisher<int> pub2(&topic);
        REQUIRE(topic.publisherCount() == 2);
    }
    REQUIRE(topic.publisherCount() == 0);
}

TEST_CASE("Publisher 移动语义", "[topic]") {
    Topic<int> topic("test/move", QoS::Event);
    Publisher<int> pub1(&topic);
    REQUIRE(topic.publisherCount() == 1);

    Publisher<int> pub2 = std::move(pub1);
    REQUIRE(topic.publisherCount() == 1);
    REQUIRE_FALSE(pub1.valid());
    REQUIRE(pub2.valid());
}

TEST_CASE("topic_names 常量", "[topic]") {
    REQUIRE(topic_names::kSlamKeyframe == "slam/keyframe");
    REQUIRE(topic_names::kSystemShutdown == "system/shutdown");
}

// ── 回调绑定方式测试 ──

struct TestReceiver {
    int value = 0;
    MsgPtr<int> held;
    void onConstRef(const int& v) { value = v; }
    void onMsgPtr(MsgPtr<int> msg) { held = std::move(msg); }
};

TEST_CASE("回调绑定：成员函数 const T&", "[topic]") {
    Topic<int> topic("test/member_ref", QoS::Event);
    Publisher<int> pub(&topic);
    TestReceiver recv;

    auto cb = detail::wrapCallback<int>(&TestReceiver::onConstRef, &recv);
    auto sub = topic.subscribe(std::move(cb));

    pub.publish(77);
    topic.drainOnce();
    REQUIRE(recv.value == 77);
}

TEST_CASE("回调绑定：成员函数 MsgPtr", "[topic]") {
    Topic<int> topic("test/member_ptr", QoS::Event);
    Publisher<int> pub(&topic);
    TestReceiver recv;

    auto cb = detail::wrapCallback<int>(&TestReceiver::onMsgPtr, &recv);
    auto sub = topic.subscribe(std::move(cb));

    pub.publish(88);
    topic.drainOnce();
    REQUIRE(recv.held != nullptr);
    REQUIRE(*recv.held == 88);
}

TEST_CASE("回调绑定：自由函数", "[topic]") {
    static int free_func_value = 0;
    auto free_fn = +[](const int& v) { free_func_value = v; };

    Topic<int> topic("test/free_fn", QoS::Event);
    Publisher<int> pub(&topic);

    auto cb = detail::wrapCallback<int>(free_fn);
    auto sub = topic.subscribe(std::move(cb));

    pub.publish(55);
    topic.drainOnce();
    REQUIRE(free_func_value == 55);
}
