// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 生命周期与有界性的硬约束：
/// 关闭分两段让因果链走完、驱动模式可顺序切换、发布成功的判据是主消费者接纳、
/// 控制类工作有界且可合并、取消定时器就释放、关闭之后的服务调用在入口可见地失败。

#include <SimpleSLAM/comm/runtime.hpp>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

using namespace simpleslam::comm;

namespace {

struct Ping {
    int value = 0;
};
struct Pong {
    int value = 0;
};

using Tally = std::shared_ptr<std::atomic<int>>;

/// A 收到 Ping 就发一条 Pong：一条最短的因果链。
///
/// 计数放在模块之外：shutdown() 会析构模块，关完再去读模块的成员就是读已释放内存。
class Relay final : public Module {
public:
    explicit Relay(Tally counter) : counter_(std::move(counter)) {}

protected:
    void onInit() override {
        out_ = advertise<Pong>("/pong");
        subscribe<Ping>("/ping", [this](const Ping& p) {
            counter_->fetch_add(1);
            out_.publish(Pong{p.value});
        }, SubscribeOptions{8, Delivery::kReliable});
    }

private:
    Tally counter_;
    Publisher<Pong> out_;
};

class Sink final : public Module {
public:
    explicit Sink(Tally counter) : counter_(std::move(counter)) {}

protected:
    void onInit() override {
        subscribe<Pong>("/pong", [this](const Pong&) { counter_->fetch_add(1); },
                        SubscribeOptions{8, Delivery::kReliable});
    }

private:
    Tally counter_;
};

class Counter final : public Module {
public:
    std::size_t ticks = 0;
    void startTimer(Duration period) {
        timer_ = createTimer(period, [this] { ++ticks; });
    }
    void stopTimer() { timer_.cancel(); }
    [[nodiscard]] std::size_t depth() const { return mailbox()->depth(); }

protected:
    void onInit() override {}

private:
    TimerHandle timer_;
};

}  // namespace

TEST_CASE("停止分两段：已接纳的输入把因果链走完", "[comm][lifecycle]") {
    // 一次性全关会让 A 已经处理完的那一帧产不出 B 的结果，
    // 最终图和轨迹静悄悄地少一截，而精度门禁正是拿它们下结论的。
    auto relayed = std::make_shared<std::atomic<int>>(0);
    auto sunk = std::make_shared<std::atomic<int>>(0);
    Runtime runtime(RuntimeOptions{1, true});
    runtime.addModule<Relay>("/relay", relayed);
    runtime.addModule<Sink>("/sink", sunk);
    runtime.init();

    auto ping = Publisher<Ping>(runtime.core().topic<Ping>("/ping"), PublishOptions{false, true});
    REQUIRE(ping.publish(Ping{1}).deliveredTo(1));

    runtime.shutdown();
    CHECK(relayed->load() == 1);
    CHECK(sunk->load() == 1);  // 派生话题在因果链排空之后才关
}

TEST_CASE("关闭之后发布不算成功", "[comm][lifecycle]") {
    // 话题关闭且零订阅者时所有计数都是 0，"没人满载"仍然为真——
    // 关键数据链路要的是"指定消费者确实收下了"。
    Runtime runtime(RuntimeOptions{1, true});
    runtime.init();
    auto topic = runtime.core().topic<Ping>("/ping");
    topic->markIngress();
    Publisher<Ping> ping(topic, PublishOptions{false, true});

    runtime.requestStop();
    const auto result = ping.publish(Ping{1});
    CHECK(result.topic_closed);
    CHECK_FALSE(result.ok());
    CHECK_FALSE(result.deliveredTo(1));
}

TEST_CASE("spinOnce 之后仍能改用 spin", "[comm][lifecycle]") {
    // io_context 跑空之后处于 stopped，不 restart 的话 spin() 会立刻返回零处理数
    auto relayed = std::make_shared<std::atomic<int>>(0);
    Runtime runtime(RuntimeOptions{1, false});
    runtime.addModule<Relay>("/relay", relayed);
    runtime.addModule<Sink>("/sink", std::make_shared<std::atomic<int>>(0));
    runtime.init();
    CHECK(runtime.spinOnce() == 0);  // 先把 io_context 跑空

    Publisher<Ping> ping(runtime.core().topic<Ping>("/ping"), PublishOptions{false, true});
    ping.publish(Ping{1});

    auto stopper = std::thread([&runtime] {
        while (runtime.state() != RuntimeState::kRunning) {
            std::this_thread::yield();
        }
        runtime.requestStop();
    });
    runtime.spin();
    stopper.join();
    CHECK(relayed->load() == 1);
}

TEST_CASE("周期唤醒可合并，控制类工作不会无界堆积", "[comm][lifecycle]") {
    // 模块长时间不被驱动时，一个周期定时器就能把收件箱撑到上千格。
    // 唤醒的语义是"去看一眼"，同一个定时器至多留一格待执行。
    Runtime runtime(RuntimeOptions{1, true});
    auto& counter = runtime.addModule<Counter>("/counter");
    runtime.init();
    counter.startTimer(std::chrono::milliseconds(10));

    auto* clock = runtime.virtualClock();
    REQUIRE(clock != nullptr);
    for (int i = 1; i <= 200; ++i) {
        clock->advanceTo(Time{static_cast<std::int64_t>(i) * 10'000'000});
        runtime.timers().fireDue(clock->now());  // 只触发，不驱动收件箱
    }
    CHECK(counter.depth() <= 1);
    CHECK(counter.ticks == 0);

    runtime.spinOnce();
    CHECK(counter.ticks == 1);  // 两百次到期合并成一次
}

TEST_CASE("取消定时器就释放条目与捕获对象", "[comm][lifecycle]") {
    TimerQueue timers;
    auto payload = std::make_shared<int>(42);
    const auto id = timers.schedule(Time{1'000'000'000}, Duration{1'000'000'000},
                                    [payload] { (void)payload; });
    CHECK(timers.size() == 1);
    CHECK(payload.use_count() == 2);

    timers.cancel(id);
    CHECK(timers.size() == 0);
    CHECK_FALSE(timers.nextDue().has_value());
    CHECK(payload.use_count() == 1);  // 回调连同它捕获的东西一起放掉了
}

TEST_CASE("插入更早的 deadline 会把等待者叫醒", "[comm][lifecycle]") {
    TimerQueue timers;
    std::size_t woke = 0;
    timers.setWaker([&woke] { ++woke; });

    (void)timers.schedule(Time{10'000'000'000}, Duration{0}, [] {});
    CHECK(woke == 1);
    (void)timers.schedule(Time{20'000'000'000}, Duration{0}, [] {});
    CHECK(woke == 1);  // 更晚的不必叫醒
    (void)timers.schedule(Time{5'000'000'000}, Duration{0}, [] {});
    CHECK(woke == 2);  // 更早的必须叫醒，否则要睡到旧的 deadline
}

TEST_CASE("收件箱关闭后控制类工作被拒并计数", "[comm][lifecycle]") {
    IoContext io;
    auto mailbox = Mailbox::create(io, "/m", nullptr, 4);
    for (int i = 0; i < 4; ++i) CHECK(mailbox->postTask([] {}));
    CHECK_FALSE(mailbox->postTask([] {}));  // 满了：明确拒绝，不静默丢
    CHECK(mailbox->droppedTasks() == 1);

    mailbox->close();
    CHECK(mailbox->undeliveredTasks() == 4);  // 关闭时从队列里清掉的
    CHECK_FALSE(mailbox->postTask([] {}));
    CHECK(mailbox->undeliveredTasks() == 5);  // 关闭之后才投来而被拒的，同样算没送达
}
