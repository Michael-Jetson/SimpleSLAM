/// @file test_comm_concurrency.cpp
/// 回归：comm hub 并发四修（深度审计 D）。
///
/// - QoS 首触者：订阅者默认 Event 建话题后，发布者显式 QoS 应纠正之。
/// - drainAll：快照锁外 drain（回调可重入 hub 不死锁）+ 轮数上限（回调发布环不无限循环）。
/// - 在线 Sync 派发：回调在 topic 锁外执行（回调内向本话题 publish 不自死锁）。
/// - async Event Inbox 满丢弃必须上报（法则4，不静默吞）。
///
/// 注：死锁/无限循环类（Fix2/3）修前表现为挂起，故为反挂起回归——通过即证明不挂。

#include <SimpleSLAM/core/infra/comm/topic.hpp>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

using namespace simpleslam;

namespace {
MsgPtr<int> mk(int v) { return std::make_shared<const int>(v); }

void waitUntil(std::atomic<int>& c, int target, int max_ms = 2000) {
    for (int s = 0; s < max_ms && c.load() < target; ++s)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

struct ErrorHandlerGuard {
    SubscriberErrorHandler saved;
    explicit ErrorHandlerGuard(SubscriberErrorHandler h)
        : saved(subscriberErrorHandler()) {
        subscriberErrorHandler() = std::move(h);
    }
    ~ErrorHandlerGuard() { subscriberErrorHandler() = std::move(saved); }
};
}  // namespace

// ── Fix 1：QoS 首触者纠正 ──

TEST_CASE("QoS 首触者：订阅者默认 Event 建话题，发布者显式 Latest 纠正之", "[comm][qos]") {
    TopicHub hub(/*offline=*/true);

    int got1 = 0;
    auto sub1 = hub.subscribeImpl<int>("t", [&](MsgPtr<int> m) { got1 = *m; });

    auto pub = hub.createPublisherImpl<int>("t", QoS::Latest);  // 应采纳 Latest
    pub.publish(mk(42));
    hub.drainAll();
    REQUIRE(got1 == 42);

    // 迟到订阅者应收到 latching 补发——仅 Latest 话题有此行为
    int got2 = 0;
    auto sub2 = hub.subscribeImpl<int>("t", [&](MsgPtr<int> m) { got2 = *m; });
    REQUIRE(got2 == 42);  // 修前：话题仍 Event → 无 latching → got2==0
}

// ── Fix 2：drainAll 锁外 drain + 轮数上限 ──

TEST_CASE("drainAll：回调中重入 hub（getLatest）不死锁", "[comm][drain]") {
    TopicHub hub(/*offline=*/true);
    auto pub = hub.createPublisherImpl<int>("t", QoS::Event);

    int seen = 0;
    auto sub = hub.subscribeImpl<int>("t", [&](MsgPtr<int> m) {
        (void)hub.getLatestImpl<int>("t");  // 重入 hub：修前持 hub 锁跑回调 → 死锁挂起
        seen = *m;
    });

    pub.publish(mk(7));
    hub.drainAll();  // 修前挂起；修后完成
    REQUIRE(seen == 7);
}

TEST_CASE("drainAll：回调发布环有轮数上限，不无限循环", "[comm][drain]") {
    TopicHub hub(/*offline=*/true);
    auto pubA = hub.createPublisherImpl<int>("A", QoS::Event);
    auto pubB = hub.createPublisherImpl<int>("B", QoS::Event);

    auto subA = hub.subscribeImpl<int>("A", [&](MsgPtr<int> m) { pubB.publish(mk(*m + 1)); });
    auto subB = hub.subscribeImpl<int>("B", [&](MsgPtr<int> m) { pubA.publish(mk(*m + 1)); });

    pubA.publish(mk(0));
    size_t total = hub.drainAll();  // 修前：A↔B 无限循环挂起；修后：上限后中断返回
    REQUIRE(total > 0);
}

// ── Fix 3：在线 Sync 派发锁外 ──

TEST_CASE("在线 Sync 回调中向本话题 publish 不自死锁", "[comm][online]") {
    Topic<int> topic("t", QoS::Event);
    topic.setOfflineMode(false);  // 在线：publish 直接派发
    Publisher<int> pub(&topic);

    int count = 0;
    bool reentered = false;
    auto sub = topic.subscribe([&](MsgPtr<int> m) {
        ++count;
        if (!reentered) {
            reentered = true;
            pub.publish(mk(*m + 1));  // 修前：回调持 topic mutex_ → 此 publish 自死锁
        }
    });

    pub.publish(mk(1));   // 修前挂起；修后完成
    REQUIRE(count == 2);  // 原始 + 一次重入
}

// ── Fix 4：async Event 丢弃上报 ──

TEST_CASE("async Event Inbox 满丢弃时上报（不静默）", "[comm][async]") {
    std::atomic<int> drops{0};
    ErrorHandlerGuard guard([&](const std::string&, const char*) { drops.fetch_add(1); });

    Topic<int> topic("t", QoS::Event);  // Event=可靠传递，丢弃须上报
    topic.setOfflineMode(false);
    Publisher<int> pub(&topic);

    std::atomic<int> entered{0};
    std::atomic<bool> gate{false};
    SubscribeOptions opts;
    opts.delivery = DeliveryMode::Async;
    opts.queue_depth = 2;  // 小队列易溢出
    auto sub = topic.subscribe(
        [&](MsgPtr<int>) {
            entered.fetch_add(1);
            while (!gate.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        },
        opts);

    pub.publish(mk(1));          // worker 取走 1 进回调阻塞
    waitUntil(entered, 1);       // 确保 worker 卡在回调里
    for (int i = 2; i <= 20; ++i) pub.publish(mk(i));  // 填满 depth=2 后溢出 → 丢弃上报
    int observed = drops.load();  // post() 同步上报，此刻已计数
    gate.store(true);            // 放行 worker（务必在 REQUIRE 前，否则 join 挂起）
    REQUIRE(observed > 0);       // 修前：丢弃静默 → observed==0
}

// ── Fix 5：拆解期 UAF（话题先于句柄/Publisher 销毁）──

TEST_CASE("拆解：话题先于句柄/Publisher 销毁不 UAF", "[comm][lifetime]") {
    SubscriptionHandle h;
    Publisher<int> p;
    {
        TopicHub hub(/*offline=*/true);
        p = hub.createPublisherImpl<int>("t", QoS::Event);
        h = hub.subscribeImpl<int>("t", [](MsgPtr<int>) {});
    }  // hub 析构 → 话题释放，但 h/p 仍存活

    // 死话题上操作：liveness 令牌过期 → no-op，不触及已释放内存
    p.publish(mk(1));
    CHECK_FALSE(p.valid());
    // 作用域末 h、p 析构：修前 self->unsubscribe / removePublisher 触及已死话题 → UAF
    SUCCEED();
}
