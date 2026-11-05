// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 运行时：模块串行执行、虚拟时钟定时器、离线双跑一致、参数帧边界生效、
/// 接线诊断，以及"停止 → 静止 → 销毁"三段式关闭协议。

#include <SimpleSLAM/comm/runtime.hpp>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace simpleslam::comm;

namespace {

struct Tick {
    int value = 0;
};

/// 处理时断言"同一模块内不会并发"，并记录到达顺序。
class Counter final : public Module {
public:
    std::atomic<int> concurrent{0};
    std::atomic<int> violations{0};
    std::atomic<int> processed{0};
    std::vector<int> order;

    void onInit() override {
        subscribe("/tick", &Counter::onTick, SubscribeOptions{4096, Delivery::kDropOldest});
    }

    void onTick(const Tick& tick) {
        if (concurrent.fetch_add(1) != 0) ++violations;
        order.push_back(tick.value);
        ++processed;
        std::this_thread::yield();
        concurrent.fetch_sub(1);
    }
};

class Ticker final : public Module {
public:
    int fired = 0;

    void onInit() override {
        timer_ = createTimer(std::chrono::milliseconds(10), &Ticker::onTimer);
    }
    void onTimer() { ++fired; }

    TimerHandle timer_;
};

class Configurable final : public Module {
public:
    std::vector<std::int64_t> observed;

    void onInit() override {
        iterations_ = param<std::int64_t>("~iterations", 20);
        subscribe("/tick", &Configurable::onTick, {});
    }
    void onTick(const Tick&) {
        // 一次回调之内读到的快照必须稳定
        observed.push_back(paramSnapshot()->get<std::int64_t>(resolve("~iterations")).value_or(-1));
    }

    std::int64_t iterations_ = 0;
};

/// 在自己的回调里调用 shutdown()。入口应当识别出这是运行时线程，降级为只停输入。
class SelfStopper final : public Module {
public:
    Runtime* runtime = nullptr;
    void onInit() override { subscribe("/tick", &SelfStopper::onTick, {}); }
    void onTick(const Tick&) {
        if (runtime != nullptr) runtime->shutdown();
    }
};

/// 回调会一直卡住，直到外部放行。用来验证 shutdown() 必须等回调退出。
class Blocking final : public Module {
public:
    struct Shared {
        std::mutex mutex;
        std::condition_variable cv;
        bool entered = false;
        bool released = false;
        std::atomic<bool> destroyed{false};
        std::atomic<bool> callback_done{false};
    };

    explicit Blocking(std::shared_ptr<Shared> shared) : shared_(std::move(shared)) {}
    ~Blocking() override { shared_->destroyed.store(true); }

    void onInit() override { subscribe("/tick", &Blocking::onTick, {}); }

    void onTick(const Tick&) {
        {
            std::lock_guard lock(shared_->mutex);
            shared_->entered = true;
        }
        shared_->cv.notify_all();
        std::unique_lock lock(shared_->mutex);
        shared_->cv.wait(lock, [this] { return shared_->released; });
        // 回调还没退出时模块必须仍然活着
        CHECK_FALSE(shared_->destroyed.load());
        shared_->callback_done.store(true);
    }

private:
    std::shared_ptr<Shared> shared_;
};

}  // namespace

TEST_CASE("同一模块的处理严格串行", "[comm][runtime]") {
    Runtime runtime(RuntimeOptions{4, /*virtual_clock=*/false});
    auto& counter = runtime.addModule<Counter>("/counter");
    runtime.init();

    auto publisher = Publisher<Tick>(runtime.core().topic<Tick>("/tick"), PublishOptions{});

    std::thread spinner([&] { runtime.spin(); });
    const int kPerThread = 500;
    std::thread p1([&] {
        for (int i = 0; i < kPerThread; ++i) publisher.publish(Tick{i});
    });
    std::thread p2([&] {
        for (int i = 0; i < kPerThread; ++i) publisher.publish(Tick{i});
    });
    p1.join();
    p2.join();

    while (counter.processed.load() < 2 * kPerThread) std::this_thread::yield();
    runtime.requestStop();
    spinner.join();

    CHECK(counter.violations.load() == 0);
    CHECK(counter.processed.load() == 2 * kPerThread);
}

TEST_CASE("虚拟时钟驱动定时器，不依赖墙钟等待", "[comm][runtime]") {
    Runtime runtime(RuntimeOptions{1, /*virtual_clock=*/true});
    auto& ticker = runtime.addModule<Ticker>("/ticker");
    runtime.init();

    CHECK(ticker.fired == 0);
    runtime.spinFor(std::chrono::milliseconds(35));
    CHECK(ticker.fired == 3);  // 10ms 周期，推进 35ms

    ticker.timer_.cancel();
    runtime.spinFor(std::chrono::milliseconds(100));
    CHECK(ticker.fired == 3);  // 取消之后不再触发
}

TEST_CASE("离线双跑的处理顺序逐条一致", "[comm][runtime]") {
    auto run = [] {
        Runtime runtime(RuntimeOptions{1, true});
        auto& a = runtime.addModule<Counter>("/a");
        auto& b = runtime.addModule<Counter>("/b");
        runtime.init();

        auto publisher = Publisher<Tick>(runtime.core().topic<Tick>("/tick"), PublishOptions{});
        for (int i = 0; i < 20; ++i) {
            publisher.publish(Tick{i});
            runtime.spinOnce();
        }
        std::vector<int> combined = a.order;
        combined.insert(combined.end(), b.order.begin(), b.order.end());
        return combined;
    };

    CHECK(run() == run());
}

TEST_CASE("参数提交在帧边界生效", "[comm][runtime]") {
    Runtime runtime(RuntimeOptions{1, true});
    auto& module = runtime.addModule<Configurable>("/cfg");
    runtime.init();

    auto publisher = Publisher<Tick>(runtime.core().topic<Tick>("/tick"), PublishOptions{});
    publisher.publish(Tick{0});
    runtime.spinOnce();
    REQUIRE(module.observed.size() == 1);
    CHECK(module.observed[0] == 20);

    const auto result = runtime.setParams({{"/cfg/iterations", std::int64_t{40}}});
    REQUIRE(result.ok());
    CHECK(result.revision == 1);

    publisher.publish(Tick{1});
    runtime.spinOnce();
    runtime.spinOnce();
    REQUIRE(module.observed.size() == 2);
    CHECK(module.observed[1] == 40);
}

TEST_CASE("接线诊断能抓出只发无订与只订无发", "[comm][runtime]") {
    Runtime runtime(RuntimeOptions{1, true});
    runtime.addModule<Counter>("/counter");  // 订阅 /tick
    runtime.init();

    auto dangling = runtime.danglingTopics();
    REQUIRE(dangling.size() == 1);
    CHECK(dangling[0] == "/tick");  // 只订无发

    auto publisher = Publisher<Tick>(runtime.core().topic<Tick>("/tick"), PublishOptions{});
    CHECK(runtime.danglingTopics().empty());
}

TEST_CASE("两个运行时互不干扰", "[comm][runtime]") {
    Runtime first(RuntimeOptions{1, true});
    Runtime second(RuntimeOptions{1, true});
    auto& a = first.addModule<Counter>("/counter");
    auto& b = second.addModule<Counter>("/counter");
    first.init();
    second.init();

    auto publisher = Publisher<Tick>(first.core().topic<Tick>("/tick"), PublishOptions{});
    publisher.publish(Tick{1});
    first.spinOnce();
    second.spinOnce();

    CHECK(a.processed.load() == 1);
    CHECK(b.processed.load() == 0);  // 同名话题在另一个运行时里不串
}

/// 计数器活得比模块久：shutdown() 之后模块已被销毁，读模块引用就是 use-after-free。
/// 旧用例正是这么写的，ASan 抓到了；所以计数放在模块之外的共享对象里。
class OutlivingCounter final : public Module {
public:
    explicit OutlivingCounter(std::shared_ptr<std::atomic<int>> counter)
        : counter_(std::move(counter)) {}

    void onInit() override { subscribe("/tick", &OutlivingCounter::onTick, {}); }
    void onTick(const Tick&) { counter_->fetch_add(1); }

private:
    std::shared_ptr<std::atomic<int>> counter_;
};

TEST_CASE("停止后不再接纳新消息", "[comm][runtime][lifecycle]") {
    auto processed = std::make_shared<std::atomic<int>>(0);
    Runtime runtime(RuntimeOptions{1, true});
    runtime.addModule<OutlivingCounter>("/counter", processed);
    runtime.init();

    auto publisher = Publisher<Tick>(runtime.core().topic<Tick>("/tick"), PublishOptions{});
    publisher.publish(Tick{1});
    runtime.spinOnce();
    CHECK(processed->load() == 1);

    runtime.requestStop();
    CHECK(runtime.state() == RuntimeState::kStopping);
    const auto rejected = publisher.publish(Tick{2});
    CHECK(rejected.accepted == 0);  // 话题的接纳口已关
    runtime.spinOnce();
    CHECK(processed->load() == 1);

    runtime.shutdown();
    CHECK(runtime.state() == RuntimeState::kTornDown);
    publisher.publish(Tick{3});
    runtime.spinOnce();
    CHECK(processed->load() == 1);
}

TEST_CASE("已接纳的消息在关闭时仍被处理完", "[comm][runtime][lifecycle]") {
    auto processed = std::make_shared<std::atomic<int>>(0);
    {
        Runtime runtime(RuntimeOptions{1, true});
        runtime.addModule<OutlivingCounter>("/counter", processed);
        runtime.init();
        auto publisher = Publisher<Tick>(runtime.core().topic<Tick>("/tick"), PublishOptions{});
        for (int i = 0; i < 5; ++i) publisher.publish(Tick{i});
        // 一条都还没 spin：全部积在收件箱里
        CHECK(processed->load() == 0);
        runtime.shutdown();
    }
    CHECK(*processed == 5);  // 停输入 → 排空 → 才销毁
}

TEST_CASE("关闭等回调退出后才收尾，析构时才销毁模块", "[comm][runtime][lifecycle]") {
    auto shared = std::make_shared<Blocking::Shared>();
    auto runtime = std::make_unique<Runtime>(RuntimeOptions{1, /*virtual_clock=*/false});
    runtime->addModule<Blocking>("/blocking", shared);
    runtime->init();

    auto publisher = Publisher<Tick>(runtime->core().topic<Tick>("/tick"), PublishOptions{});
    std::thread spinner([&] { runtime->spin(); });
    publisher.publish(Tick{1});

    {
        std::unique_lock lock(shared->mutex);
        shared->cv.wait(lock, [&] { return shared->entered; });
    }

    // 回调正卡着：另一个线程发起关闭，它必须等待而不是就地销毁模块
    std::thread closer([&] { runtime->shutdown(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK_FALSE(shared->destroyed.load());  // 还在回调里，模块不能被销毁

    {
        std::lock_guard lock(shared->mutex);
        shared->released = true;
    }
    shared->cv.notify_all();

    closer.join();
    spinner.join();
    CHECK(shared->callback_done.load());
    // 收尾完成后模块保留为只读的最终状态，Runtime 析构时才销毁
    CHECK_FALSE(shared->destroyed.load());
    runtime.reset();
    CHECK(shared->destroyed.load());
}

TEST_CASE("关闭阶段不再接纳新的执行者", "[comm][runtime][lifecycle]") {
    // 等已有参与者退出 ≠ 禁止新参与者进入。
    // 静止之后、销毁之前，另一个线程调 spinOnce() 仍能拿到执行权。
    auto processed = std::make_shared<std::atomic<int>>(0);
    Runtime runtime(RuntimeOptions{1, true});
    runtime.addModule<OutlivingCounter>("/counter", processed);
    runtime.init();

    auto publisher = Publisher<Tick>(runtime.core().topic<Tick>("/tick"), PublishOptions{});
    publisher.publish(Tick{1});
    CHECK(runtime.spinOnce() > 0);
    CHECK(processed->load() == 1);
    CHECK_FALSE(runtime.executionClosed());

    runtime.shutdown();
    CHECK(runtime.executionClosed());
    // 入口已封：即便有已接纳的工作，外部线程也拿不到执行权
    CHECK(runtime.spinOnce() == 0);
    CHECK(processed->load() == 1);
}

TEST_CASE("在运行时线程内调用 shutdown 只停输入，不自等", "[comm][runtime][lifecycle]") {
    Runtime runtime(RuntimeOptions{1, true});
    auto& stopper = runtime.addModule<SelfStopper>("/stopper");
    stopper.runtime = &runtime;
    runtime.init();

    bool reported = false;
    runtime.setErrorReporter([&](std::string_view, std::string_view) { reported = true; });

    auto publisher = Publisher<Tick>(runtime.core().topic<Tick>("/tick"), PublishOptions{});
    publisher.publish(Tick{1});
    runtime.spinOnce();

    CHECK(reported);
    CHECK(runtime.state() == RuntimeState::kStopping);  // 只停输入，没有拆掉自己
    runtime.shutdown();                                 // 从控制线程关，这次才真正销毁
    CHECK(runtime.state() == RuntimeState::kTornDown);
}
