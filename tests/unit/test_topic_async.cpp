#include <SimpleSLAM/core/infra/comm/topic.hpp>

#include <catch2/catch_test_macros.hpp>
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace simpleslam;

static MsgPtr<int> mk(int v) { return std::make_shared<const int>(v); }

/// RAII：临时替换全局订阅者异常钩子
struct ErrorHandlerGuard {
    SubscriberErrorHandler saved;
    explicit ErrorHandlerGuard(SubscriberErrorHandler h)
        : saved(subscriberErrorHandler()) {
        subscriberErrorHandler() = std::move(h);
    }
    ~ErrorHandlerGuard() { subscriberErrorHandler() = std::move(saved); }
};

// ── Inbox 有界队列（确定性，无线程）──

TEST_CASE("Inbox DropOldest：满了丢最旧", "[inbox]") {
    Inbox<int> box(3, DropPolicy::DropOldest);
    bool dropped = false;
    for (int i = 1; i <= 5; ++i) dropped |= box.push(mk(i));
    REQUIRE(dropped);

    std::vector<MsgPtr<int>> out;
    REQUIRE(box.drainBatch(out) == 3);
    REQUIRE(out.size() == 3);
    REQUIRE(*out[0] == 3);  // 丢了 1、2，保留最新 3、4、5
    REQUIRE(*out[2] == 5);
}

TEST_CASE("Inbox DropNewest：满了拒绝新来的", "[inbox]") {
    Inbox<int> box(3, DropPolicy::DropNewest);
    for (int i = 1; i <= 5; ++i) box.push(mk(i));

    std::vector<MsgPtr<int>> out;
    REQUIRE(box.drainBatch(out) == 3);
    REQUIRE(*out[0] == 1);  // 保留最早 1、2、3，拒绝 4、5
    REQUIRE(*out[2] == 3);
}

TEST_CASE("Inbox drainBatch 清空且可重复抽取", "[inbox]") {
    Inbox<int> box(10, DropPolicy::DropOldest);
    box.push(mk(1));
    box.push(mk(2));
    std::vector<MsgPtr<int>> out;
    REQUIRE(box.drainBatch(out) == 2);
    out.clear();
    REQUIRE(box.drainBatch(out) == 0);  // 已清空
}

// ── 异步 worker 投递（多线程）──

static void waitUntil(std::atomic<int>& counter, int target, int max_ms = 2000) {
    for (int s = 0; s < max_ms && counter.load() < target; ++s)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

TEST_CASE("DeliveryMode::Async：发布非阻塞，worker 串行送达全部、保序", "[async]") {
    Topic<int> topic("test/async", QoS::Event);
    topic.setOfflineMode(false);  // 在线模式才启用 async worker
    Publisher<int> pub(&topic);

    std::atomic<int> count{0};
    std::atomic<int> last{0};
    SubscribeOptions opts;
    opts.delivery = DeliveryMode::Async;
    opts.queue_depth = 1000;  // 足够大，不丢
    auto sub = topic.subscribe(
        [&](MsgPtr<int> m) {
            last.store(*m);
            count.fetch_add(1);
        },
        opts);

    for (int i = 1; i <= 100; ++i) pub.publish(i);  // 应立即返回（非阻塞）

    waitUntil(count, 100);
    REQUIRE(count.load() == 100);
    REQUIRE(last.load() == 100);  // 串行 actor，FIFO 保序 → 最后是 100
}

TEST_CASE("DeliveryMode::Async：worker 内回调抛异常被隔离、不杀线程", "[async]") {
    std::atomic<int> errors{0};
    ErrorHandlerGuard guard(
        [&](const std::string&, const char*) { errors.fetch_add(1); });

    Topic<int> topic("test/async_throw", QoS::Event);
    topic.setOfflineMode(false);
    Publisher<int> pub(&topic);

    std::atomic<int> good{0};
    SubscribeOptions opts;
    opts.delivery = DeliveryMode::Async;
    opts.queue_depth = 100;
    auto sub = topic.subscribe(
        [&](MsgPtr<int> m) {
            if (*m % 2 == 0) throw std::runtime_error("even boom");
            good.fetch_add(1);
        },
        opts);

    for (int i = 1; i <= 10; ++i) pub.publish(i);

    std::atomic<int> done{0};
    // good + errors 都计入完成度
    for (int s = 0; s < 2000 && (good.load() + errors.load()) < 10; ++s) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        done.store(good.load() + errors.load());
    }

    REQUIRE(good.load() == 5);    // 奇数 5 个正常处理
    REQUIRE(errors.load() == 5);  // 偶数 5 个抛异常被隔离上报，worker 存活
}
