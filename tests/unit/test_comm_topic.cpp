// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 话题：投递不在发布调用栈内发生、FIFO、三种交付策略的接纳结果、
/// latch 走订阅方执行上下文、回调与上报路径的异常隔离、跨话题的模块级 FIFO。

#include <SimpleSLAM/comm/execution.hpp>
#include <SimpleSLAM/comm/mailbox.hpp>
#include <SimpleSLAM/comm/topic.hpp>

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>
#include <vector>

using namespace simpleslam::comm;

namespace {

struct Msg {
    int value = 0;
};

/// 把所有就绪任务跑完。
std::size_t drainAll(IoContext& io) {
    if (io.stopped()) io.restart();
    return io.poll();
}

}  // namespace

TEST_CASE("发布不在调用栈内执行回调", "[comm][topic]") {
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    Topic<Msg> topic("/t", nullptr);

    std::vector<int> seen;
    auto sub = topic.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); }, {});

    topic.publish(std::make_shared<const Msg>(Msg{1}), false);
    CHECK(seen.empty());  // 还没 poll，回调不应执行

    drainAll(io);
    CHECK(seen == std::vector<int>{1});
}

TEST_CASE("同一订阅端按发布先后 FIFO", "[comm][topic]") {
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    Topic<Msg> topic("/t", nullptr);

    std::vector<int> seen;
    auto sub = topic.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); },
                               SubscribeOptions{10, Delivery::kDropOldest});

    for (int i = 1; i <= 5; ++i) topic.publish(std::make_shared<const Msg>(Msg{i}), false);
    drainAll(io);
    CHECK(seen == std::vector<int>{1, 2, 3, 4, 5});
}

TEST_CASE("DropOldest 满载丢最旧并上报", "[comm][topic]") {
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    std::vector<std::string> reports;
    Topic<Msg> topic("/t", [&](std::string_view, std::string_view what) {
        reports.emplace_back(what);
    });

    std::vector<int> seen;
    auto sub = topic.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); },
                               SubscribeOptions{2, Delivery::kDropOldest});

    for (int i = 1; i <= 4; ++i) topic.publish(std::make_shared<const Msg>(Msg{i}), false);
    CHECK(reports.size() == 2);  // 丢弃必须可观测

    drainAll(io);
    CHECK(seen == std::vector<int>{3, 4});
}

TEST_CASE("Reliable 满载拒新并让发布方知道", "[comm][topic]") {
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    Topic<Msg> topic("/t", nullptr);

    std::vector<int> seen;
    auto sub = topic.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); },
                               SubscribeOptions{2, Delivery::kReliable});

    CHECK(topic.publish(std::make_shared<const Msg>(Msg{1}), false).ok());
    CHECK(topic.publish(std::make_shared<const Msg>(Msg{2}), false).ok());

    const auto full = topic.publish(std::make_shared<const Msg>(Msg{3}), false);
    CHECK_FALSE(full.ok());
    CHECK(full.rejected_full == 1);

    drainAll(io);
    CHECK(seen == std::vector<int>{1, 2});  // 拒收的那条没有悄悄挤掉别人
}

TEST_CASE("Latest 只保留最新值并合并唤醒", "[comm][topic]") {
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    Topic<Msg> topic("/t", nullptr);

    std::vector<int> seen;
    auto sub = topic.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); },
                               SubscribeOptions{1, Delivery::kLatest});

    topic.publish(std::make_shared<const Msg>(Msg{1}), false);
    const auto second = topic.publish(std::make_shared<const Msg>(Msg{2}), false);
    topic.publish(std::make_shared<const Msg>(Msg{3}), false);
    CHECK(second.coalesced == 1);

    drainAll(io);
    CHECK(seen == std::vector<int>{3});  // 不重放中间帧
}

TEST_CASE("latch 补发走订阅方的执行上下文", "[comm][topic]") {
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    Topic<Msg> topic("/t", nullptr);

    topic.publish(std::make_shared<const Msg>(Msg{7}), true);

    std::vector<int> seen;
    auto sub = topic.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); }, {});
    // 关键：补发不得在 subscribe 的调用线程上同步执行
    CHECK(seen.empty());

    drainAll(io);
    CHECK(seen == std::vector<int>{7});
}

TEST_CASE("回调异常在收件箱的执行边界逐条隔离并计数，上报出口自己抛异常也不逃逸",
          "[comm][topic]") {
    IoContext io;
    int reported = 0;
    auto exec = Mailbox::create(io, "/m", [&](std::string_view, std::string_view) {
        ++reported;
        throw std::runtime_error("上报出口自己炸了");
    });
    Topic<Msg> topic("/t", nullptr);

    std::vector<int> seen;
    auto bad = topic.subscribe(exec, [](const MsgPtr<Msg>&) { throw std::runtime_error("boom"); },
                               {});
    auto good = topic.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); }, {});

    CHECK_NOTHROW(topic.publish(std::make_shared<const Msg>(Msg{5}), false));
    CHECK_NOTHROW(drainAll(io));

    CHECK(reported == 1);
    CHECK(exec->isolatedFailures() == 1);  // 接住不等于没发生：丢了一份工作，要数得出来
    CHECK(seen == std::vector<int>{5});    // 一个订阅者抛异常不影响另一个
}

TEST_CASE("回调内再次发布同一话题不丢消息", "[comm][topic]") {
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    Topic<Msg> topic("/t", nullptr);

    std::vector<int> seen;
    auto sub = topic.subscribe(exec, [&](const MsgPtr<Msg>& m) {
        seen.push_back(m->value);
        if (m->value == 1) topic.publish(std::make_shared<const Msg>(Msg{2}), false);
    }, {});

    topic.publish(std::make_shared<const Msg>(Msg{1}), false);
    drainAll(io);
    CHECK(seen == std::vector<int>{1, 2});
}

TEST_CASE("Publisher 统计订阅者并在无话题时返回 closed", "[comm][topic]") {
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    auto topic = std::make_shared<Topic<Msg>>("/t", nullptr);

    Publisher<Msg> pub(topic, PublishOptions{});
    CHECK(pub.valid());
    CHECK(pub.subscriberCount() == 0);

    auto sub = topic->subscribe(exec, [](const MsgPtr<Msg>&) {}, {});
    CHECK(pub.subscriberCount() == 1);
    CHECK(topic->publisherCount() == 1);

    Publisher<Msg> empty;
    const auto result = empty.publish(Msg{1});
    CHECK(result.closed == 1);
}

TEST_CASE("同一收件箱跨话题也按发布先后 FIFO", "[comm][topic][mailbox]") {
    // 各订阅端自持队列、各自整批处理时，按 A(1) B(2) A(3) 发布，
    // 模块实际收到 1、3、2。这不违反"同一订阅 FIFO"，但违反 ADR-0010 的"各模块 FIFO"。
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    Topic<Msg> topic_a("/a", nullptr);
    Topic<Msg> topic_b("/b", nullptr);

    std::vector<int> seen;
    auto sub_a = topic_a.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); },
                                   SubscribeOptions{10, Delivery::kReliable});
    auto sub_b = topic_b.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); },
                                   SubscribeOptions{10, Delivery::kReliable});

    topic_a.publish(std::make_shared<const Msg>(Msg{1}), false);
    topic_b.publish(std::make_shared<const Msg>(Msg{2}), false);
    topic_a.publish(std::make_shared<const Msg>(Msg{3}), false);
    drainAll(io);

    CHECK(seen == std::vector<int>{1, 2, 3});
}

TEST_CASE("kLatest 的覆盖不排新格，收件箱不会空转", "[comm][topic][mailbox]") {
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    Topic<Msg> topic("/t", nullptr);

    std::vector<int> seen;
    auto sub = topic.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); },
                               SubscribeOptions{1, Delivery::kLatest});

    for (int i = 1; i <= 5; ++i) topic.publish(std::make_shared<const Msg>(Msg{i}), false);
    CHECK(exec->depth() == 1);  // 五次发布只占一格：覆盖不增加待处理条数
    drainAll(io);
    CHECK(seen == std::vector<int>{5});
}

TEST_CASE("可丢档的越序边界：新载荷复用被挤出那条的位置", "[comm][topic][mailbox]") {
    // 这是收窄之后的承诺，不是缺陷：为每次挤出补一格会让格数无界增长，
    // 而那正是容量约束要防的事。需要因果顺序的消息请用 kReliable。
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    Topic<Msg> topic_a("/a", nullptr);
    Topic<Msg> topic_b("/b", nullptr);

    std::vector<int> seen;
    auto sub_a = topic_a.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); },
                                   SubscribeOptions{1, Delivery::kDropOldest});
    auto sub_b = topic_b.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); },
                                   SubscribeOptions{4, Delivery::kReliable});

    topic_a.publish(std::make_shared<const Msg>(Msg{1}), false);
    topic_b.publish(std::make_shared<const Msg>(Msg{2}), false);
    topic_a.publish(std::make_shared<const Msg>(Msg{3}), false);  // 挤掉 1，占用 1 的位置
    drainAll(io);

    CHECK(seen == std::vector<int>{3, 2});  // 而不是存活消息的发布顺序 {2, 3}
    CHECK(exec->depth() == 0);
}

TEST_CASE("话题关闭后拒绝新发布，已接纳的仍会投递", "[comm][topic]") {
    IoContext io;
    auto exec = Mailbox::create(io, "/m", nullptr);
    Topic<Msg> topic("/t", nullptr);

    std::vector<int> seen;
    auto sub = topic.subscribe(exec, [&](const MsgPtr<Msg>& m) { seen.push_back(m->value); },
                               SubscribeOptions{10, Delivery::kReliable});

    topic.publish(std::make_shared<const Msg>(Msg{1}), false);
    topic.close();
    const auto after = topic.publish(std::make_shared<const Msg>(Msg{2}), false);
    CHECK(after.closed == 1);
    CHECK(after.accepted == 0);

    drainAll(io);
    CHECK(seen == std::vector<int>{1});  // 停输入不等于丢掉手上的活
}

TEST_CASE("可丢档的越序范围：存活条目也会前移", "[comm][topic]") {
    // 格子指向的是订阅端而不是某一条消息，所以丢弃之后该订阅端仍然存活的条目
    // 会继承更早的格位，整体前移到其他话题之前。
    // 容量 2、DropOldest，依次发布 A1 B2 A3 A4：只丢了 A1，
    // 而 A3 与 B2 的相对顺序同样变了。
    IoContext io;
    auto mailbox = Mailbox::create(io, "/m", nullptr);
    Topic<int> a("/a", nullptr);
    Topic<int> b("/b", nullptr);

    std::vector<std::string> seen;
    auto sub_a = a.subscribe(mailbox, [&](const MsgPtr<int>& v) {
        seen.push_back("A" + std::to_string(*v));
    }, SubscribeOptions{2, Delivery::kDropOldest});
    auto sub_b = b.subscribe(mailbox, [&](const MsgPtr<int>& v) {
        seen.push_back("B" + std::to_string(*v));
    }, SubscribeOptions{2, Delivery::kDropOldest});

    a.publish(std::make_shared<const int>(1), false);
    b.publish(std::make_shared<const int>(2), false);
    a.publish(std::make_shared<const int>(3), false);
    a.publish(std::make_shared<const int>(4), false);
    io.run();

    REQUIRE(seen.size() == 3);
    CHECK(seen[0] == "A3");
    CHECK(seen[1] == "B2");
    CHECK(seen[2] == "A4");
}

TEST_CASE("可靠订阅端的接纳单独计数", "[comm][topic]") {
    // "没人满载"不等于"主消费者收下了"：观察者用可丢档，它不拒收，
    // 所以要单独看可靠档那一路。
    IoContext io;
    auto mailbox = Mailbox::create(io, "/m", nullptr);
    Topic<int> topic("/t", nullptr);
    auto primary = topic.subscribe(mailbox, [](const MsgPtr<int>&) {},
                                   SubscribeOptions{1, Delivery::kReliable});
    auto observer = topic.subscribe(mailbox, [](const MsgPtr<int>&) {},
                                    SubscribeOptions{1, Delivery::kDropOldest});

    const auto first = topic.publish(std::make_shared<const int>(1), false);
    CHECK(first.reliable_subscribers == 1);
    CHECK(first.reliable_accepted == 1);
    CHECK(first.deliveredTo(1));

    const auto second = topic.publish(std::make_shared<const int>(2), false);
    CHECK(second.reliable_accepted == 0);  // 可靠档满了
    CHECK_FALSE(second.deliveredTo(1));
    CHECK(second.dropped_oldest == 1);     // 观察者照收不误
}
