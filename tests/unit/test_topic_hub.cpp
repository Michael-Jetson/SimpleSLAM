#include <SimpleSLAM/core/infra/comm/topic.hpp>
#include <SimpleSLAM/core/infra/comm/topic_names.hpp>

#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

using namespace simpleslam;

// 每个测试用隔离实例，不依赖全局单例状态
TEST_CASE("TopicHub 隔离实例 Publisher + Subscriber", "[topic_hub]") {
    TopicHub hub(true);
    auto pub = hub.createPublisher<int>("test/int");
    REQUIRE(hub.hasTopic("test/int"));

    int received = 0;
    auto sub = hub.subscribe<int>("test/int",
        [&](MsgPtr<int> msg) { received = *msg; });

    pub.publish(42);
    hub.drainAll();
    REQUIRE(received == 42);
}

TEST_CASE("TopicHub 类型不匹配抛异常", "[topic_hub]") {
    TopicHub hub(true);
    hub.createPublisher<int>("test/typed");
    REQUIRE_THROWS(hub.createPublisher<std::string>("test/typed"));
}

TEST_CASE("TopicHub drainAll BFS 顺序", "[topic_hub]") {
    TopicHub hub(true);
    auto pub_a = hub.createPublisher<int>("a");
    auto pub_b = hub.createPublisher<int>("b");

    std::vector<std::string> order;

    // topic a 的回调向 topic b 发布——应在下一轮处理
    auto sub_a = hub.subscribe<int>("a",
        [&](MsgPtr<int> msg) {
            order.push_back("a:" + std::to_string(*msg));
            pub_b.publish(*msg + 100);
        });
    auto sub_b = hub.subscribe<int>("b",
        [&](MsgPtr<int> msg) {
            order.push_back("b:" + std::to_string(*msg));
        });

    pub_a.publish(1);
    size_t drained = hub.drainAll();

    REQUIRE(drained == 2);
    REQUIRE(order.size() == 2);
    REQUIRE(order[0] == "a:1");
    REQUIRE(order[1] == "b:101");
}

TEST_CASE("TopicHub listTopics / stats 自省", "[topic_hub]") {
    TopicHub hub(true);

    auto pub = hub.createPublisher<int>("introspect/test");
    int received = 0;
    auto sub = hub.subscribe<int>("introspect/test",
        [&](const int& v) { received = v; });

    pub.publish(77);
    hub.drainAll();
    REQUIRE(received == 77);

    auto names = hub.listTopics();
    REQUIRE_FALSE(names.empty());

    auto s = hub.stats("introspect/test");
    REQUIRE(s.publisher_count == 1);
    REQUIRE(s.subscriber_count == 1);
    REQUIRE(s.message_count == 1);
}

TEST_CASE("TopicHub 成员函数订阅", "[topic_hub]") {
    struct Handler {
        int value = 0;
        void onMessage(const int& v) { value = v; }
    };

    TopicHub hub(true);
    auto pub = hub.createPublisher<int>("test/member");

    Handler handler;
    auto cb = detail::wrapCallback<int>(&Handler::onMessage, &handler);
    auto sub = hub.subscribe<int>("test/member", std::move(cb));

    pub.publish(33);
    hub.drainAll();
    REQUIRE(handler.value == 33);
}

TEST_CASE("TopicHub getLatest", "[topic_hub]") {
    TopicHub hub(true);
    auto pub = hub.createPublisher<int>("test/latest", QoS::Latest);

    // 发布前为 nullptr
    REQUIRE(hub.getLatest<int>("test/latest") == nullptr);

    pub.publish(10);
    pub.publish(20);
    hub.drainAll();

    auto val = hub.getLatest<int>("test/latest");
    REQUIRE(val != nullptr);
    REQUIRE(*val == 20);
}

TEST_CASE("TopicHub 成员函数指针订阅", "[topic_hub]") {
    struct Recv {
        std::string data;
        void onMsg(const std::string& s) { data = s; }
    };

    TopicHub hub(true);
    Recv recv;

    auto pub = hub.createPublisher<std::string>("test/member_ptr");
    auto sub = hub.subscribe<std::string>("test/member_ptr", &Recv::onMsg, &recv);

    pub.publish(std::string("hello"));
    hub.drainAll();
    REQUIRE(recv.data == "hello");
}

TEST_CASE("两个 TopicHub 实例互不串扰（无全局共享，ADR-001）", "[topic_hub][injection]") {
    TopicHub hubA(true);
    TopicHub hubB(true);

    int gotA = 0, gotB = 0;
    auto pubA = hubA.createPublisher<int>("x");
    auto subA = hubA.subscribe<int>("x", [&](MsgPtr<int> m) { gotA = *m; });
    auto subB = hubB.subscribe<int>("x", [&](MsgPtr<int> m) { gotB = *m; });

    pubA.publish(42);
    hubA.drainAll();
    hubB.drainAll();

    REQUIRE(gotA == 42);            // 自己 hub 收到
    REQUIRE(gotB == 0);            // 另一 hub 完全不可见——证明无全局共享态
    REQUIRE(hubA.hasTopic("x"));
    REQUIRE(hubB.hasTopic("x"));   // 各自有独立的 "x" 注册表，互不相干（非同一全局话题）
}

TEST_CASE("连通性检查：抓只发无订 / 只订无发（名字错配兜底）", "[topic_hub][wiring]") {
    TopicHub hub(true);
    auto pub = hub.createPublisher<int>("only_pub");                 // 发无订
    auto sub = hub.subscribe<int>("only_sub", [](MsgPtr<int>) {});   // 订无发
    auto pub2 = hub.createPublisher<int>("connected");
    auto sub2 = hub.subscribe<int>("connected", [](MsgPtr<int>) {}); // 两端齐

    auto dangling = hub.findDanglingTopics();
    bool has_only_pub = false, has_only_sub = false, has_connected = false;
    for (const auto& d : dangling) {
        if (d.name == "only_pub") has_only_pub = true;
        if (d.name == "only_sub") has_only_sub = true;
        if (d.name == "connected") has_connected = true;
    }
    REQUIRE(dangling.size() == 2);
    REQUIRE(has_only_pub);
    REQUIRE(has_only_sub);
    REQUIRE_FALSE(has_connected);  // 两端齐的不算悬空
}
