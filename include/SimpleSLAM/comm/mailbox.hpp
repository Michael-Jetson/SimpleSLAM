// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file mailbox.hpp
/// 模块收件箱：模块级 FIFO 的唯一入口。不同话题的消息、参数换代、定时器到期、服务响应
/// 都排进同一条队列。在线时由模块自己的线程取走，离线时由控制线程按就绪顺序步进。
///
/// 收件箱不拒收：只有关闭之后 post 才返回 false。容量由订阅端决定。

#include <SimpleSLAM/comm/clock.hpp>
#include <SimpleSLAM/comm/types.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace simpleslam::comm {

class Mailbox;
using MailboxPtr = std::shared_ptr<Mailbox>;

/// 收件箱一格指向的投递目标（一个订阅端）。
class IDeliveryTarget {
public:
    virtual ~IDeliveryTarget() = default;
    IDeliveryTarget(const IDeliveryTarget&) = delete;
    IDeliveryTarget& operator=(const IDeliveryTarget&) = delete;
    virtual void deliverOne() = 0;

protected:
    IDeliveryTarget() = default;
};

/// 所有收件箱共用的小对象。句柄可能比 Runtime 活得久，所以它由 shared_ptr 持有。
class Hub {
public:
    /// 离线步进时打乱模块之间的先后，每个模块内部的 FIFO 不变。种子相同则顺序相同，0 表示不打乱。
    void setScheduleSeed(std::uint64_t seed) noexcept;
    void pushReady(MailboxPtr box);
    [[nodiscard]] MailboxPtr popReady();
    void clearReady();

    /// 在途计数：从排进收件箱到处理完。归零表示系统静止。
    void addWork() noexcept { work_.fetch_add(1, std::memory_order_acq_rel); }
    void finishWork(std::size_t n = 1);
    [[nodiscard]] bool idle() const noexcept { return work_.load(std::memory_order_acquire) == 0; }
    bool waitIdle(std::chrono::nanoseconds timeout);

    [[nodiscard]] const std::shared_ptr<std::atomic<std::uint64_t>>& timerSequence() const noexcept {
        return timer_sequence_;
    }

    void stopInputs() noexcept { inputs_stopped_.store(true, std::memory_order_release); }
    [[nodiscard]] bool inputsStopped() const noexcept {
        return inputs_stopped_.load(std::memory_order_acquire);
    }

    /// 记下一次被接住的异常。stop 为真时停止输入，并留给驱动调用重新抛出。
    void recordFailure(ModuleFailure failure, bool stop);
    [[nodiscard]] std::optional<ModuleFailure> firstFailure() const;
    /// 取走待重新抛出的异常，只取一次。
    [[nodiscard]] std::exception_ptr takeFatal();
    void setStopHook(std::function<void()> hook);

private:
    std::shared_ptr<std::atomic<std::uint64_t>> timer_sequence_ =
        std::make_shared<std::atomic<std::uint64_t>>(0);

    mutable std::mutex failure_mutex_;
    std::optional<ModuleFailure> first_failure_;
    std::exception_ptr fatal_;
    std::function<void()> stop_hook_;

    std::mutex ready_mutex_;
    std::deque<std::weak_ptr<Mailbox>> ready_;  ///< 收件箱归模块所有，这里持有就成环了
    std::uint64_t rng_ = 0;
    std::atomic<std::size_t> work_{0};
    std::mutex idle_mutex_;
    std::condition_variable idle_cv_;
    std::atomic<bool> inputs_stopped_{false};
};

struct MailboxStats {
    std::uint64_t callbacks = 0;
    std::uint64_t busy_ns = 0;
    std::uint64_t max_callback_ns = 0;
    std::uint64_t failures = 0;
    std::uint64_t discarded = 0;
    std::size_t depth = 0;
};

/// 收件箱的主人在每个回调前后要做的事：模块在这里设上参数作用域，回调结束后提交插件记下的写。
class ICallbackScope {
public:
    virtual ~ICallbackScope() = default;
    virtual void enterCallback() noexcept = 0;
    /// `completed` 为 false 表示回调抛了异常。
    virtual void leaveCallback(bool completed) noexcept = 0;

protected:
    ICallbackScope() = default;
    ICallbackScope(const ICallbackScope&) = default;
    ICallbackScope& operator=(const ICallbackScope&) = default;
};

class Mailbox final : public std::enable_shared_from_this<Mailbox> {
public:
    /// 离线一次步进最多处理的格数。超出则排回就绪队列尾部，避免一个模块饿死其他模块。
    static constexpr std::size_t kDrainBudget = 64;

    Mailbox(std::string owner, std::shared_ptr<Hub> hub, ErrorReporter reporter,
            ExceptionPolicy policy = ExceptionPolicy::kIsolate)
        : owner_(std::move(owner)),
          hub_(std::move(hub)),
          reporter_(std::move(reporter)),
          policy_(policy),
          timers_(hub_->timerSequence()) {}

    Mailbox(const Mailbox&) = delete;
    Mailbox& operator=(const Mailbox&) = delete;

    [[nodiscard]] const std::string& owner() const noexcept { return owner_; }
    /// 每个回调前后都调它。装配时设一次，`scope` 的寿命不短于收件箱的回调。
    void setCallbackScope(ICallbackScope* scope) noexcept { scope_ = scope; }
    void setExceptionPolicy(ExceptionPolicy policy) noexcept {
        policy_.store(policy, std::memory_order_relaxed);
    }

    /// 排一条已接纳的消息。target 的寿命由收件箱在处理期间持有。
    bool postSlot(std::shared_ptr<IDeliveryTarget> target) {
        return push(Item{std::move(target), {}});
    }
    /// 排一件控制类工作：参数换代、定时器回调、服务请求与响应。
    bool post(std::function<void()> task) { return push(Item{nullptr, std::move(task)}); }

    /// 离线：跑一批。只由控制线程调用。
    std::size_t drainSome();

    /// 本模块的定时器。到期时在排进这个收件箱，和消息同一条 FIFO。
    [[nodiscard]] TimerQueue& timers() noexcept { return timers_; }
    /// 登记定时器，并让模块线程按新的最早到期时刻重新等待。已关闭时不登记，返回空句柄。
    TimerHandle scheduleTimer(Time due, Duration period, std::function<void()> fire,
                              TimerHandle::Alive alive = nullptr);

    /// 在线：模块线程的主循环，直到 stopThread() 或 close()。
    void threadLoop(IClock& clock);
    void stopThread();

    /// 停止收件。已排队未处理的格被丢弃，任务在锁外析构，定时器一并清掉：
    /// 回调和超时可能持有这个收件箱，不清就成环，谁也放不掉。
    void close() {
        closeAdmission();
        releaseDiscarded();
    }
    /// close() 的第一段：拒绝新工作，取走待丢弃的格，但先不析构。被丢弃的任务析构时可能结案，
    /// 并把终态投给另一个收件箱。取消时先把全部收件箱都关上再统一析构，结果才不取决于装配顺序。
    void closeAdmission();
    /// close() 的第二段：在锁外析构第一段取走的格，清掉定时器。
    void releaseDiscarded();
    [[nodiscard]] bool closed() const;

    [[nodiscard]] MailboxStats stats() const;

private:
    struct Item {
        std::shared_ptr<IDeliveryTarget> target;
        std::function<void()> task;
    };

    bool push(Item item);
    void run(Item& item) noexcept;
    void fail(const char* what) noexcept;

    std::string owner_;
    std::shared_ptr<Hub> hub_;
    ErrorReporter reporter_;
    std::atomic<ExceptionPolicy> policy_;
    TimerQueue timers_;
    ICallbackScope* scope_ = nullptr;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Item> items_;
    std::deque<Item> discarded_items_;
    bool queued_ = false;       ///< 离线：已在就绪队列里
    bool threaded_ = false;     ///< 在线：模块线程在跑
    bool thread_stop_ = false;
    bool timers_changed_ = false;  ///< 有新定时器：模块线程要重新算等多久
    bool closed_ = false;

    std::atomic<std::uint64_t> callbacks_{0};
    std::atomic<std::uint64_t> busy_ns_{0};
    std::atomic<std::uint64_t> max_callback_ns_{0};
    std::atomic<std::uint64_t> failures_{0};
    std::atomic<std::uint64_t> discarded_{0};
};

}  // namespace simpleslam::comm
