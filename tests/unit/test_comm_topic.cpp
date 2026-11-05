// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 话题：投递不在发布调用栈内发生、FIFO、queue_size 的语义、latch 走订阅方收件箱、
/// 回调与上报路径的异常隔离、跨话题的模块级 FIFO。

#include <SimpleSLAM/comm/topic.hpp>

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <vector>

using namespace simpleslam::comm;

namespace {

struct Msg {
    int value = 0;
};

int valueOf(const AnyMsg& m) { return static_cast<const Msg*>(m.get())->value; }

struct Rig {
    std::shared_ptr<Hub> hub = std::make_shared<Hub>();
    MailboxPtr box;
    explicit Rig(ErrorReporter reporter = nullptr)
        : box(std::make_shared<Mailbox>("/m", hub, std::move(reporter))) {}

    [[nodiscard]] std::shared_ptr<Topic> topic(const std::string& name, ErrorReporter reporter = nullptr,
                                               std::size_t backlog_warn = 0) const {
        return std::make_shared<Topic>(name, std::type_index(typeid(Msg)), hub, std::move(reporter), backlog_warn);
    }
    void drainAll() const {
        while (auto ready = hub->popReady()) ready->drainSome();
    }
};

void publish(const std::shared_ptr<Topic>& topic, int value, bool latch = false) {
    topic->publish(std::make_shared<const Msg>(Msg{value}), latch);
}

}  // namespace

TEST_CASE("发布不在调用栈内执行回调", "[comm][topic]") {
    Rig rig;
    auto topic = rig.topic("/t");
    std::vector<int> seen;
    auto sub = topic->subscribe(rig.box, 10, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });

    publish(topic, 1);
    CHECK(seen.empty());

    rig.drainAll();
    CHECK(seen == std::vector<int>{1});
}

TEST_CASE("同一订阅端按发布先后 FIFO", "[comm][topic]") {
    Rig rig;
    auto topic = rig.topic("/t");
    std::vector<int> seen;
    auto sub = topic->subscribe(rig.box, 10, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });

    for (int i = 1; i <= 5; ++i) publish(topic, i);
    rig.drainAll();
    CHECK(seen == std::vector<int>{1, 2, 3, 4, 5});
}

TEST_CASE("queue_size 为 N 时满了丢最旧并计数", "[comm][topic]") {
    Rig rig;
    auto topic = rig.topic("/t");
    std::vector<int> seen;
    auto sub = topic->subscribe(rig.box, 2, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });

    for (int i = 1; i <= 4; ++i) publish(topic, i);
    CHECK(sub->stats().dropped == 2);  // 丢弃必须可观测

    rig.drainAll();
    CHECK(seen == std::vector<int>{3, 4});
}

TEST_CASE("queue_size 为 0 时不丢，积压每翻一倍提醒一次", "[comm][topic]") {
    Rig rig;
    std::vector<std::string> reports;
    auto topic = rig.topic("/t", [&](std::string_view, std::string_view what) { reports.emplace_back(what); }, 4);
    std::vector<int> seen;
    auto sub = topic->subscribe(rig.box, 0, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });

    for (int i = 0; i < 100; ++i) publish(topic, i);
    CHECK(reports.size() == 5);  // 4、8、16、32、64
    CHECK(sub->stats().high_water == 100);
    CHECK(sub->stats().dropped == 0);

    rig.drainAll();
    CHECK(seen.size() == 100);
    CHECK(sub->stats().delivered == 100);
}

TEST_CASE("queue_size 为 1 时只留最新，覆盖不排新格", "[comm][topic]") {
    Rig rig;
    auto topic = rig.topic("/t");
    std::vector<int> seen;
    auto sub = topic->subscribe(rig.box, 1, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });

    for (int i = 1; i <= 5; ++i) publish(topic, i);
    CHECK(rig.box->stats().depth == 1);  // 五次发布只占一格
    rig.drainAll();
    CHECK(seen == std::vector<int>{5});  // 不重放中间帧
}

TEST_CASE("latch 补发走订阅方的收件箱", "[comm][topic]") {
    Rig rig;
    auto topic = rig.topic("/t");
    publish(topic, 7, true);

    std::vector<int> seen;
    auto sub = topic->subscribe(rig.box, 10, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });
    CHECK(seen.empty());  // 补发不得在 subscribe 的调用线程上同步执行

    rig.drainAll();
    CHECK(seen == std::vector<int>{7});
}

TEST_CASE("回调异常在收件箱的执行边界逐条隔离并计数，上报出口自己抛异常也不逃逸", "[comm][topic]") {
    int reported = 0;
    Rig rig([&](std::string_view, std::string_view) {
        ++reported;
        throw std::runtime_error("上报出口自己炸了");
    });
    auto topic = rig.topic("/t");
    std::vector<int> seen;
    auto bad = topic->subscribe(rig.box, 10, [](const AnyMsg&) { throw std::runtime_error("boom"); });
    auto good = topic->subscribe(rig.box, 10, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });

    CHECK_NOTHROW(publish(topic, 5));
    CHECK_NOTHROW(rig.drainAll());

    CHECK(reported == 1);
    CHECK(rig.box->stats().failures == 1);  // 接住不等于没发生：丢了一份工作，要数得出来
    CHECK(seen == std::vector<int>{5});     // 一个订阅者抛异常不影响另一个
}

TEST_CASE("回调内再次发布同一话题不丢消息", "[comm][topic]") {
    Rig rig;
    auto topic = rig.topic("/t");
    std::vector<int> seen;
    auto sub = topic->subscribe(rig.box, 10, [&](const AnyMsg& m) {
        seen.push_back(valueOf(m));
        if (valueOf(m) == 1) publish(topic, 2);
    });

    publish(topic, 1);
    rig.drainAll();
    CHECK(seen == std::vector<int>{1, 2});
}

TEST_CASE("Publisher 统计订阅者，没有话题时发布是空操作", "[comm][topic]") {
    Rig rig;
    auto topic = rig.topic("/t");
    Publisher<Msg> pub(topic, PublishOptions{});
    CHECK(pub.valid());
    CHECK(pub.subscriberCount() == 0);

    auto sub = topic->subscribe(rig.box, 10, [](const AnyMsg&) {});
    CHECK(pub.subscriberCount() == 1);
    CHECK(topic->publisherCount() == 1);

    Publisher<Msg> empty;
    CHECK_NOTHROW(empty.publish(Msg{1}));
    CHECK_FALSE(empty.valid());
}

TEST_CASE("入口发布端在停止输入之后不再发，派生发布端不受影响", "[comm][topic]") {
    Rig rig;
    auto topic = rig.topic("/t");
    std::vector<int> seen;
    auto sub = topic->subscribe(rig.box, 0, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });
    Publisher<Msg> ingress(topic, PublishOptions{.ingress = true});
    Publisher<Msg> derived(topic, PublishOptions{});

    ingress.publish(Msg{1});
    rig.hub->stopInputs();
    ingress.publish(Msg{2});
    derived.publish(Msg{3});
    rig.drainAll();
    CHECK(seen == std::vector<int>{1, 3});
}

TEST_CASE("同一收件箱跨话题也按发布先后 FIFO", "[comm][topic][mailbox]") {
    // 各订阅端自持队列、各自整批处理时，按 A(1) B(2) A(3) 发布，模块实际收到 1、3、2。
    Rig rig;
    auto topic_a = rig.topic("/a");
    auto topic_b = rig.topic("/b");
    std::vector<int> seen;
    auto sub_a = topic_a->subscribe(rig.box, 0, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });
    auto sub_b = topic_b->subscribe(rig.box, 0, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });

    publish(topic_a, 1);
    publish(topic_b, 2);
    publish(topic_a, 3);
    rig.drainAll();
    CHECK(seen == std::vector<int>{1, 2, 3});
}

TEST_CASE("丢最旧时新消息继承被挤出那条的位置", "[comm][topic][mailbox]") {
    // 为每次挤出补一格会让格数无界增长，而那正是容量约束要防的事。需要因果顺序的消息用 0。
    Rig rig;
    auto topic_a = rig.topic("/a");
    auto topic_b = rig.topic("/b");
    std::vector<int> seen;
    auto sub_a = topic_a->subscribe(rig.box, 1, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });
    auto sub_b = topic_b->subscribe(rig.box, 0, [&](const AnyMsg& m) { seen.push_back(valueOf(m)); });

    publish(topic_a, 1);
    publish(topic_b, 2);
    publish(topic_a, 3);  // 挤掉 1，占用 1 的位置
    rig.drainAll();
    CHECK(seen == std::vector<int>{3, 2});  // 而不是存活消息的发布顺序 {2, 3}
    CHECK(rig.box->stats().depth == 0);
}

TEST_CASE("丢最旧之后，该订阅端存活的条目也会前移", "[comm][topic][mailbox]") {
    // 格子指向订阅端而不是某一条消息。容量 2，依次发布 A1 B2 A3 A4：只丢了 A1，
    // 而 A3 与 B2 的相对顺序同样变了。
    Rig rig;
    auto topic_a = rig.topic("/a");
    auto topic_b = rig.topic("/b");
    std::vector<std::string> seen;
    auto sub_a = topic_a->subscribe(rig.box, 2, [&](const AnyMsg& m) { seen.push_back("A" + std::to_string(valueOf(m))); });
    auto sub_b = topic_b->subscribe(rig.box, 2, [&](const AnyMsg& m) { seen.push_back("B" + std::to_string(valueOf(m))); });

    publish(topic_a, 1);
    publish(topic_b, 2);
    publish(topic_a, 3);
    publish(topic_a, 4);
    rig.drainAll();

    REQUIRE(seen.size() == 3);
    CHECK(seen[0] == "A3");
    CHECK(seen[1] == "B2");
    CHECK(seen[2] == "A4");
}
