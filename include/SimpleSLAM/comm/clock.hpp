// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file clock.hpp
/// 时间与定时器。时间一律是纳秒整型。在线用单调时钟，离线用虚拟时钟按数据时间推进。

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

namespace simpleslam::comm {

using Time = std::chrono::nanoseconds;
using Duration = std::chrono::nanoseconds;

class IClock {
public:
    virtual ~IClock() = default;
    [[nodiscard]] virtual Time now() const = 0;
};

class SteadyClock final : public IClock {
public:
    SteadyClock() : start_(std::chrono::steady_clock::now()) {}
    [[nodiscard]] Time now() const override {
        return std::chrono::duration_cast<Time>(std::chrono::steady_clock::now() - start_);
    }

private:
    std::chrono::steady_clock::time_point start_;
};

/// 只能前进。
class VirtualClock final : public IClock {
public:
    [[nodiscard]] Time now() const override { return Time{ns_.load(std::memory_order_acquire)}; }
    void advanceTo(Time t) {
        std::int64_t current = ns_.load(std::memory_order_relaxed);
        while (t.count() > current
               && !ns_.compare_exchange_weak(current, t.count(), std::memory_order_acq_rel)) {
        }
    }

private:
    std::atomic<std::int64_t> ns_{0};
};

/// 取消之后不再开始新的触发。条目在本该到期时才被清掉。
class TimerHandle {
public:
    using Alive = std::shared_ptr<std::atomic<bool>>;
    TimerHandle() = default;
    explicit TimerHandle(Alive alive) : alive_(std::move(alive)) {}
    void cancel() const {
        if (alive_) alive_->store(false, std::memory_order_release);
    }

private:
    Alive alive_;
};

/// 一个模块的定时器。到期在模块自己的线程上处理（离线时由控制线程步进），通信层不为定时器占线程。
class TimerQueue {
public:
    /// 一个到期项。at + seq 决定先后：离线时各模块的到期项按它合并，序列因此确定。
    struct Due {
        Time at{};
        std::uint64_t seq = 0;
        Duration period{0};
        std::function<void()> fire;
        TimerHandle::Alive alive;
        TimerQueue* owner = nullptr;
    };

    /// sequence 由同一运行时的所有队列共用。
    explicit TimerQueue(std::shared_ptr<std::atomic<std::uint64_t>> sequence)
        : sequence_(std::move(sequence)) {}

    /// period 为零表示一次性定时器。
    TimerHandle schedule(Time due, Duration period, std::function<void()> fire,
                         TimerHandle::Alive alive = nullptr);

    /// 取出所有到期项，不触发。
    void takeDue(Time now, std::vector<Due>& out);
    /// 周期定时器放回去。积压时只补一次。
    void rearm(const Due& due, Time now);
    /// 离线确定时钟原点时用：装配期排下的定时器改从原点起算。
    void shiftAll(Duration offset);
    /// 清掉全部条目，捕获的对象在锁外析构。
    void clear();
    [[nodiscard]] std::optional<Time> nextDue();

private:
    struct Entry {
        Duration period{0};
        std::function<void()> fire;
        TimerHandle::Alive alive;
    };
    using Key = std::pair<std::int64_t, std::uint64_t>;

    std::mutex mutex_;
    std::shared_ptr<std::atomic<std::uint64_t>> sequence_;
    std::map<Key, Entry> entries_;
};

/// 触发这些队列里全部到期的定时器，按到期时刻和登记先后。回调在锁外执行。
std::size_t fireDueTimers(const std::vector<TimerQueue*>& queues, Time now);

}  // namespace simpleslam::comm
