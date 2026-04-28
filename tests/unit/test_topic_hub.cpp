#include <SimpleSLAM/core/infra/topic_hub.hpp>
#include <SimpleSLAM/core/infra/topic_names.hpp>

#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

using namespace simpleslam;

// 每个测试用隔离实例，不依赖全局单例状态
TEST_CASE("TopicHub 隔离实例 Publisher + Subscriber", "[topic_hub]") {
    TopicHub hub(true);
    auto pub = hub.createPublisherImpl<int>("test/int");
    REQUIRE(hub.hasTopic("test/int"));

    int received = 0;
    auto sub = hub.subscribeImpl<int>("test/int",
        [&](MsgPtr<int> msg) { received = *msg; });

    pub.publish(42);
    hub.drainAll();
    REQUIRE(received == 42);
}

TEST_CASE("TopicHub 类型不匹配抛异常", "[topic_hub]") {
    TopicHub hub(true);
    hub.createPublisherImpl<int>("test/typed");
    REQUIRE_THROWS(hub.createPublisherImpl<std::string>("test/typed"));
}

TEST_CASE("TopicHub drainAll BFS 顺序", "[topic_hub]") {
    TopicHub hub(true);
    auto pub_a = hub.createPublisherImpl<int>("a");
    auto pub_b = hub.createPublisherImpl<int>("b");

    std::vector<std::string> order;

    // topic a 的回调向 topic b 发布——应在下一轮处理
    auto sub_a = hub.subscribeImpl<int>("a",
        [&](MsgPtr<int> msg) {
            order.push_back("a:" + std::to_string(*msg));
            pub_b.publish(*msg + 100);
        });
    auto sub_b = hub.subscribeImpl<int>("b",
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

TEST_CASE("TopicHub 全局单例 init/shutdown", "[topic_hub]") {
    TopicHub::init(true);

    auto pub = TopicHub::createPublisher<int>("global/test");
    int received = 0;
    auto sub = TopicHub::createSubscriber<int>("global/test",
        [&](const int& v) { received = v; });

    pub.publish(77);
    TopicHub::instance().drainAll();
    REQUIRE(received == 77);

    auto names = TopicHub::listTopics();
    REQUIRE_FALSE(names.empty());

    auto s = TopicHub::stats("global/test");
    REQUIRE(s.publisher_count == 1);
    REQUIRE(s.subscriber_count == 1);
    REQUIRE(s.message_count == 1);

    TopicHub::shutdown();
}

TEST_CASE("TopicHub 成员函数订阅", "[topic_hub]") {
    struct Handler {
        int value = 0;
        void onMessage(const int& v) { value = v; }
    };

    TopicHub hub(true);
    auto pub = hub.createPublisherImpl<int>("test/member");

    Handler handler;
    auto cb = detail::wrapCallback<int>(&Handler::onMessage, &handler);
    auto sub = hub.subscribeImpl<int>("test/member", std::move(cb));

    pub.publish(33);
    hub.drainAll();
    REQUIRE(handler.value == 33);
}

TEST_CASE("TopicHub getLatest", "[topic_hub]") {
    TopicHub hub(true);
    auto pub = hub.createPublisherImpl<int>("test/latest", QoS::Latest);

    // 发布前为 nullptr
    REQUIRE(hub.getLatestImpl<int>("test/latest") == nullptr);

    pub.publish(10);
    pub.publish(20);
    hub.drainAll();

    auto val = hub.getLatestImpl<int>("test/latest");
    REQUIRE(val != nullptr);
    REQUIRE(*val == 20);
}

TEST_CASE("TopicHub 静态 createSubscriber 成员函数绑定", "[topic_hub]") {
    struct Recv {
        std::string data;
        void onMsg(const std::string& s) { data = s; }
    };

    TopicHub::init(true);
    Recv recv;

    auto pub = TopicHub::createPublisher<std::string>("test/static_member");
    auto sub = TopicHub::createSubscriber<std::string>(
        "test/static_member", &Recv::onMsg, &recv);

    pub.publish(std::string("hello"));
    TopicHub::instance().drainAll();
    REQUIRE(recv.data == "hello");

    TopicHub::shutdown();
}
