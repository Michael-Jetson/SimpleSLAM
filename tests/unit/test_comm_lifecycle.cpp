// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 生命周期：已接纳的输入把因果链走完、驱动方式可顺序切换、定时器合并与取消、收件箱关闭。

#include <SimpleSLAM/comm/runtime.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <atomic>
#include <chrono>
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

/// 收到 Ping 就发一条 Pong：一条最短的因果链。
class Relay final : public Module {
public:
    explicit Relay(Tally counter) : counter_(std::move(counter)) {}

protected:
    void onInit() override {
        out_ = advertise<Pong>("/pong");
        subscribe<Ping>("/ping", 0, [this](const Ping& p) {
            counter_->fetch_add(1);
            out_.publish(Pong{p.value});
        });
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
        subscribe<Pong>("/pong", 0, [this](const Pong&) { counter_->fetch_add(1); });
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

private:
    TimerHandle timer_;
};

}  // namespace

TEST_CASE("停止之后已接纳的输入把因果链走完", "[comm][lifecycle]") {
    // 停止时如果连派生消息也不处理，已经进系统的那一帧产不出下游的结果，
    // 最终图和轨迹静悄悄地少一截，而评测正是拿它们算精度的。
    auto relayed = std::make_shared<std::atomic<int>>(0);
    auto sunk = std::make_shared<std::atomic<int>>(0);
    Runtime runtime(RuntimeOptions{true});
    runtime.addModule<Relay>("/relay", relayed);
    runtime.addModule<Sink>("/sink", sunk);
    runtime.init();

    auto ping = runtime.inputPublisher<Ping>("/ping");
    ping.publish(Ping{1});

    runtime.shutdown();
    CHECK(relayed->load() == 1);
    CHECK(sunk->load() == 1);
    CHECK(runtime.shutdownDrained());
}

TEST_CASE("在线停止时同样走完因果链", "[comm][lifecycle]") {
    auto relayed = std::make_shared<std::atomic<int>>(0);
    auto sunk = std::make_shared<std::atomic<int>>(0);
    Runtime runtime;
    runtime.addModule<Relay>("/relay", relayed);
    runtime.addModule<Sink>("/sink", sunk);
    runtime.init();
    runtime.start();

    auto ping = runtime.inputPublisher<Ping>("/ping");
    for (int i = 0; i < 1000; ++i) ping.publish(Ping{i});
    runtime.requestStop();
    ping.publish(Ping{-1});  // 停止之后入口不再发
    runtime.shutdown();

    CHECK(relayed->load() == 1000);
    CHECK(sunk->load() == 1000);
}

TEST_CASE("spinOnce 之后仍能改用 spin", "[comm][lifecycle]") {
    auto relayed = std::make_shared<std::atomic<int>>(0);
    Runtime runtime;
    runtime.addModule<Relay>("/relay", relayed);
    runtime.addModule<Sink>("/sink", std::make_shared<std::atomic<int>>(0));
    runtime.init();
    CHECK(runtime.spinOnce() == 0);

    auto ping = runtime.inputPublisher<Ping>("/ping");
    ping.publish(Ping{1});

    auto stopper = std::thread([&runtime] {
        while (runtime.state() != RuntimeState::kRunning) std::this_thread::yield();
        runtime.requestStop();
    });
    runtime.spin();
    stopper.join();
    CHECK(relayed->load() == 1);
}

TEST_CASE("spin 之前排下的工作由各模块的线程接手，长回调不挡别的模块", "[comm][lifecycle]") {
    class Slow final : public Module {
    public:
        std::atomic<bool> entered{false};
        std::atomic<bool> release{false};
        void onInit() override {
            subscribe<Ping>("/slow", 0, [this](const Ping&) {
                entered = true;
                while (!release.load()) std::this_thread::yield();
            });
        }
    };
    class Fast final : public Module {
    public:
        std::atomic<int> seen{0};
        void onInit() override {
            subscribe<Ping>("/fast", 0, [this](const Ping&) { ++seen; });
        }
    };
    Runtime runtime;
    auto& slow = runtime.addModule<Slow>("/slow_mod");
    auto& fast = runtime.addModule<Fast>("/fast_mod");
    runtime.init();
    runtime.inputPublisher<Ping>("/slow").publish(Ping{});
    runtime.inputPublisher<Ping>("/fast").publish(Ping{});

    runtime.start();
    for (int i = 0; i < 5000 && !(slow.entered.load() && fast.seen.load() == 1); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(slow.entered.load());
    CHECK(fast.seen.load() == 1);  // 慢模块还卡着，快模块已经处理完了
    slow.release = true;
    runtime.shutdown();
}

TEST_CASE("周期唤醒可合并，定时器不会把收件箱撑大", "[comm][lifecycle]") {
    // 模块长时间不被驱动时，一个周期定时器就能把收件箱撑到上千格。
    // 唤醒的语义是"去看一眼"，同一个定时器至多留一格待执行。
    Runtime runtime(RuntimeOptions{true});
    auto& counter = runtime.addModule<Counter>("/counter");
    runtime.init();
    counter.startTimer(std::chrono::milliseconds(10));

    auto* clock = runtime.virtualClock();
    REQUIRE(clock != nullptr);
    for (int i = 1; i <= 200; ++i) {
        clock->advanceTo(Time{static_cast<std::int64_t>(i) * 10'000'000});
        fireDueTimers({&counter.mailbox()->timers()}, clock->now());  // 只触发，不处理收件箱
    }
    CHECK(counter.mailbox()->stats().depth <= 1);
    CHECK(counter.ticks == 0);

    runtime.spinOnce();
    CHECK(counter.ticks == 1);  // 两百次到期合并成一次
}

TEST_CASE("取消定时器之后不再触发，到期时释放捕获对象", "[comm][lifecycle]") {
    TimerQueue timers(std::make_shared<std::atomic<std::uint64_t>>(0));
    auto payload = std::make_shared<int>(42);
    int fired = 0;
    const auto handle = timers.schedule(Time{1'000}, Duration{1'000}, [payload, &fired] {
        (void)payload;
        ++fired;
    });
    CHECK(payload.use_count() == 2);

    CHECK(fireDueTimers({&timers}, Time{1'000}) == 1);
    CHECK(fired == 1);
    handle.cancel();
    CHECK(fireDueTimers({&timers}, Time{2'000}) == 0);
    CHECK(fired == 1);
    CHECK_FALSE(timers.nextDue().has_value());
    CHECK(payload.use_count() == 1);  // 回调连同它捕获的东西一起放掉了
}

TEST_CASE("周期定时器积压时只补一次", "[comm][lifecycle]") {
    TimerQueue timers(std::make_shared<std::atomic<std::uint64_t>>(0));
    int fired = 0;
    timers.schedule(Time{10}, Duration{10}, [&fired] { ++fired; });
    CHECK(fireDueTimers({&timers}, Time{55}) == 1);
    CHECK(fired == 1);
    CHECK(timers.nextDue() == Time{60});  // 跳到现在之后的下一个节拍
}

TEST_CASE("同一时刻到期的定时器按登记先后触发，跨模块也一样", "[comm][lifecycle]") {
    auto sequence = std::make_shared<std::atomic<std::uint64_t>>(0);
    TimerQueue a(sequence);
    TimerQueue b(sequence);
    std::string order;
    a.schedule(Time{10}, Duration{0}, [&order] { order += "1"; });
    b.schedule(Time{10}, Duration{0}, [&order] { order += "2"; });
    a.schedule(Time{10}, Duration{0}, [&order] { order += "3"; });
    b.schedule(Time{5}, Duration{0}, [&order] { order += "0"; });
    fireDueTimers({&a, &b}, Time{10});
    CHECK(order == "0123");
}

TEST_CASE("收件箱关闭后拒绝新工作，丢弃的格计入统计", "[comm][lifecycle]") {
    auto hub = std::make_shared<Hub>();
    auto mailbox = std::make_shared<Mailbox>("/m", hub, nullptr);
    for (int i = 0; i < 4; ++i) CHECK(mailbox->post([] {}));
    CHECK(mailbox->stats().depth == 4);
    CHECK_FALSE(hub->idle());

    mailbox->close();
    CHECK(mailbox->stats().discarded == 4);
    CHECK(hub->idle());  // 丢弃的工作不再算在途

    CHECK_FALSE(mailbox->post([] {}));
    CHECK(mailbox->stats().discarded == 5);
}

TEST_CASE("取消分两段：终态投给哪个收件箱都一样进不去", "[comm][lifecycle]") {
    // 被丢弃的任务析构时可能再投递。逐个关闭的话，投给还没关的那个就会被执行，
    // 结果取决于装配顺序。先把入口全关上，再统一析构。
    auto hub = std::make_shared<Hub>();
    auto first = std::make_shared<Mailbox>("/first", hub, nullptr);
    auto second = std::make_shared<Mailbox>("/second", hub, nullptr);

    auto on_destroy = std::shared_ptr<int>(new int(0), [second](int* p) {
        delete p;
        second->post([] {});  // 析构时往另一个收件箱投
    });
    REQUIRE(first->post([on_destroy] {}));
    on_destroy.reset();

    first->closeAdmission();
    second->closeAdmission();
    first->releaseDiscarded();
    second->releaseDiscarded();

    CHECK(second->stats().depth == 0);
    CHECK(second->stats().discarded == 1);
}

TEST_CASE("spin() 还在起模块线程时，另一个线程可以 shutdown", "[comm][lifecycle]") {
    struct Idle final : Module {};
    for (int i = 0; i < 100; ++i) {
        Runtime runtime(RuntimeOptions{});
        for (int m = 0; m < 8; ++m) runtime.addModule<Idle>("/m" + std::to_string(m));
        runtime.init();
        std::thread driver([&] { runtime.spin(); });
        std::thread closer([&] {
            runtime.shutdown(i % 2 == 0 ? ShutdownMode::kCancel : ShutdownMode::kFinish);
        });
        closer.join();
        driver.join();
        CHECK(runtime.state() == RuntimeState::kDone);
    }
}

TEST_CASE("Runtime 销毁之后收件箱、Hub 和定时器回调捕获的对象都释放了", "[comm][lifecycle]") {
    struct Echo {
        int value = 0;
    };
    struct Server final : Module {
        void onInit() override { advertiseService<Echo, Echo>("/echo", &Server::handle); }
        Echo handle(const Echo& e) { return e; }
    };
    struct Holder final : Module {
        explicit Holder(std::shared_ptr<int> t) : token(std::move(t)) {}
        void onInit() override {
            // 周期定时器、一次性定时器都还没到期，收件箱里还排着一次超时很长的服务调用
            (void)createTimer(std::chrono::hours(1), [t = token] { (void)t; });
            (void)createOneShot(std::chrono::hours(1), [t = token] { (void)t; });
            client = serviceClient<Echo, Echo>("/echo");
        }
        std::shared_ptr<int> token;
        ServiceClient<Echo, Echo> client;
    };
    auto token = std::make_shared<int>(1);
    std::weak_ptr<Mailbox> box;
    std::weak_ptr<Hub> hub;
    const auto mode = GENERATE(ShutdownMode::kFinish, ShutdownMode::kCancel);
    {
        Runtime runtime(RuntimeOptions{.offline = true});
        runtime.addModule<Server>("/server");
        auto& holder = runtime.addModule<Holder>("/holder", token);
        runtime.init();
        box = holder.mailbox();
        hub = runtime.core().hub();
        holder.token.reset();
        REQUIRE(holder.client.asyncCall(Echo{1}, std::chrono::hours(1), [t = token](auto) { (void)t; })
                == ServiceStatus::kOk);
        REQUIRE(token.use_count() > 1);
        runtime.shutdown(mode);
        CHECK(token.use_count() == 1);  // 关闭时就放掉了，不用等析构
    }
    CHECK(box.expired());
    CHECK(hub.expired());
}

TEST_CASE("就绪队列不持有收件箱：没排空就丢掉也能释放", "[comm][lifecycle]") {
    auto hub = std::make_shared<Hub>();
    auto owned = std::make_shared<Mailbox>("/m", hub, ErrorReporter{});
    std::weak_ptr<Mailbox> weak = owned;
    REQUIRE(owned->post([] {}));
    owned.reset();
    CHECK(weak.expired());
    CHECK(hub->popReady() == nullptr);  // 已经不在了，跳过
}

TEST_CASE("关闭之后登记定时器是空操作", "[comm][lifecycle]") {
    auto hub = std::make_shared<Hub>();
    auto box = std::make_shared<Mailbox>("/m", hub, ErrorReporter{});
    box->close();
    const auto token = std::make_shared<int>(1);
    auto handle = box->scheduleTimer(Time{0}, Duration{0}, [t = token] { (void)t; });
    handle.cancel();
    CHECK(token.use_count() == 1);
    CHECK_FALSE(box->timers().nextDue().has_value());
}
