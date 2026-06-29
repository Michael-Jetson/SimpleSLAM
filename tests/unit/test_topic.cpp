#include <SimpleSLAM/core/infra/comm/topic.hpp>
#include <SimpleSLAM/core/infra/comm/topic_names.hpp>

#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <stdexcept>
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

// ── 异常隔离（项目护城河）──

/// RAII：临时替换全局订阅者异常钩子，析构时复原（避免污染其他测试）
struct ErrorHandlerGuard {
    SubscriberErrorHandler saved;
    explicit ErrorHandlerGuard(SubscriberErrorHandler h)
        : saved(subscriberErrorHandler()) {
        subscriberErrorHandler() = std::move(h);
    }
    ~ErrorHandlerGuard() { subscriberErrorHandler() = std::move(saved); }
};

TEST_CASE("回调抛异常被隔离：离线 drain 不外抛、兄弟订阅者照常收、错误被上报", "[topic][isolation]") {
    std::vector<std::string> errors;
    ErrorHandlerGuard guard([&](const std::string& topic, const char* what) {
        errors.push_back(topic + ": " + what);
    });

    Topic<int> topic("test/isolation_offline", QoS::Event);
    Publisher<int> pub(&topic);

    int good_count = 0;
    auto sub_bad = topic.subscribe(
        [](MsgPtr<int>) { throw std::runtime_error("subscriber boom"); });
    auto sub_good = topic.subscribe([&](MsgPtr<int>) { ++good_count; });

    pub.publish(1);
    REQUIRE_NOTHROW(topic.drainOnce());  // drain 必须吞掉异常
    REQUIRE(good_count == 1);            // 兄弟订阅者仍然送达
    REQUIRE(errors.size() == 1);         // 非静默：错误被上报
    REQUIRE(errors[0].find("subscriber boom") != std::string::npos);
    REQUIRE(errors[0].find("test/isolation_offline") != std::string::npos);
}

TEST_CASE("回调抛异常被隔离：在线同步分发同样隔离", "[topic][isolation]") {
    std::vector<std::string> errors;
    ErrorHandlerGuard guard([&](const std::string&, const char* what) {
        errors.emplace_back(what);
    });

    Topic<int> topic("test/isolation_online", QoS::Event);
    topic.setOfflineMode(false);
    Publisher<int> pub(&topic);

    int good_count = 0;
    auto sub_bad = topic.subscribe(
        [](MsgPtr<int>) { throw std::runtime_error("subscriber boom"); });
    auto sub_good = topic.subscribe([&](MsgPtr<int>) { ++good_count; });

    REQUIRE_NOTHROW(pub.publish(7));  // publish→分发 不得外抛
    REQUIRE(good_count == 1);
    REQUIRE(errors.size() == 1);
}

// ── latching（QoS::Latest 迟到订阅者补发最后值）──

TEST_CASE("QoS::Latest latching：迟到订阅者订阅即补发最后值，不重放", "[topic][latching]") {
    Topic<int> topic("test/latching", QoS::Latest);
    Publisher<int> pub(&topic);

    pub.publish(42);
    topic.drainOnce();  // 42 成为 latest（此时尚无订阅者）

    int received = 0, calls = 0;
    auto sub = topic.subscribe([&](MsgPtr<int> m) { received = *m; ++calls; });

    REQUIRE(calls == 1);     // 订阅即补发 latest
    REQUIRE(received == 42);

    topic.drainOnce();       // 无新值 → 不重放
    REQUIRE(calls == 1);
}

TEST_CASE("非 Latest 话题无 latching：迟到订阅者收不到旧值", "[topic][latching]") {
    Topic<int> topic("test/no_latch", QoS::Event);
    Publisher<int> pub(&topic);

    pub.publish(42);
    topic.drainOnce();

    int calls = 0;
    auto sub = topic.subscribe([&](MsgPtr<int>) { ++calls; });
    REQUIRE(calls == 0);     // Event 话题不补发
    topic.drainOnce();
    REQUIRE(calls == 0);
}

TEST_CASE("QoS::Latest 合并：发布多次 drain 只推最新一次、不重放", "[topic][latest]") {
    Topic<int> topic("test/latest_coalesce", QoS::Latest);
    Publisher<int> pub(&topic);

    int calls = 0, last = 0;
    auto sub = topic.subscribe([&](MsgPtr<int> m) { ++calls; last = *m; });

    pub.publish(1);
    pub.publish(2);
    pub.publish(3);
    while (topic.hasPending()) topic.drainOnce();  // 一轮排空

    REQUIRE(calls == 1);   // 只推一次（不是 3 次）—— 蓝图 §7.2.4 不重放中间帧
    REQUIRE(last == 3);    // 推的是最新值
}

// ── skip-frame 节流（偷自高翔 AsyncMessageProcess）──

TEST_CASE("skip-frame 节流：throttle_every=N 每 N 条处理 1 条", "[topic][throttle]") {
    Topic<int> topic("test/throttle", QoS::Event);
    Publisher<int> pub(&topic);

    int calls = 0;
    SubscribeOptions opts;
    opts.throttle_every = 3;
    auto sub = topic.subscribe([&](MsgPtr<int>) { ++calls; }, opts);

    for (int i = 0; i < 9; ++i) {
        pub.publish(i);
        topic.drainOnce();
    }
    REQUIRE(calls == 3);  // 9 条，第 0/3/6 条处理 = 3 次
}

TEST_CASE("throttle_every=1 不跳帧（默认）", "[topic][throttle]") {
    Topic<int> topic("test/throttle1", QoS::Event);
    Publisher<int> pub(&topic);

    int calls = 0;
    auto sub = topic.subscribe([&](MsgPtr<int>) { ++calls; });  // 默认 throttle_every=1
    for (int i = 0; i < 5; ++i) {
        pub.publish(i);
        topic.drainOnce();
    }
    REQUIRE(calls == 5);
}
