#include <SimpleSLAM/core/infra/node_handle.hpp>

#include <catch2/catch_test_macros.hpp>
#include <string>

using namespace simpleslam;

TEST_CASE("NodeHandle advertise/subscribe/spinOnce", "[node]") {
    TopicHub hub(true);
    ServiceRegistry reg;
    NodeHandle nh(&hub, &reg);

    auto pub = nh.advertise<int>("chatter");
    int got = 0;
    auto sub = nh.subscribe<int>("chatter", [&](MsgPtr<int> m) { got = *m; });

    pub.publish(42);
    REQUIRE(got == 0);  // offline：spin 前不分发
    nh.spinOnce();
    REQUIRE(got == 42);
}

TEST_CASE("NodeHandle 成员函数订阅", "[node]") {
    struct H {
        int v = 0;
        void on(const int& x) { v = x; }
    };
    TopicHub hub(true);
    ServiceRegistry reg;
    NodeHandle nh(&hub, &reg);

    H h;
    auto pub = nh.advertise<int>("t");
    auto sub = nh.subscribe<int>("t", &H::on, &h);
    pub.publish(7);
    nh.spinOnce();
    REQUIRE(h.v == 7);
}

TEST_CASE("NodeHandle 服务 advertise + call + client", "[node]") {
    TopicHub hub(true);
    ServiceRegistry reg;
    NodeHandle nh(&hub, &reg);

    auto srv = nh.advertiseService<int, int>("double", [](const int& x) { return x * 2; });
    REQUIRE(nh.callService<int, int>("double", 21) == 42);

    auto client = nh.serviceClient<int, int>("double");
    REQUIRE(client.call(10) == 20);
}

TEST_CASE("NodeHandle spin 循环到停止条件", "[node]") {
    TopicHub hub(true);
    ServiceRegistry reg;
    NodeHandle nh(&hub, &reg);

    auto pub = nh.advertise<int>("s");
    int count = 0;
    auto sub = nh.subscribe<int>("s", [&](MsgPtr<int>) { ++count; });
    pub.publish(1);
    pub.publish(2);

    int iters = 0;
    nh.spin([&] { return ++iters <= 1; });  // 跑一轮（drainAll 处理全部 pending）就停
    REQUIRE(count == 2);
}
