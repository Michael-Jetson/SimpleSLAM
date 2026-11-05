// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file clock.hpp
/// 时间与定时器。时间一律是纳秒整型，不用 double。
///
/// 定时器不塞进 IClock：IClock 只负责读时间，排队与到期判定归 TimerQueue，
/// 由 Runtime 驱动。在线用真实时钟，离线用虚拟时钟按数据时间推进，
/// 同一份模块代码两种模式都跑，且离线不依赖墙钟等待。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace simpleslam::comm {

/// 自时钟纪元起的时间点，纳秒整型。
using Time = std::chrono::nanoseconds;
/// 时间间隔，纳秒整型。
using Duration = std::chrono::nanoseconds;

class IClock {
public:
    virtual ~IClock() = default;
    [[nodiscard]] virtual Time now() const = 0;
    /// 虚拟时钟由外部推进，Runtime 据此决定是否可以在离线模式下跳过等待。
    [[nodiscard]] virtual bool isVirtual() const noexcept = 0;
};

/// 单调时钟，返回进程启动以来的纳秒数。
class SteadyClock final : public IClock {
public:
    SteadyClock() : start_(std::chrono::steady_clock::now()) {}

    [[nodiscard]] Time now() const override {
        return std::chrono::duration_cast<Time>(std::chrono::steady_clock::now() - start_);
    }
    [[nodiscard]] bool isVirtual() const noexcept override { return false; }

private:
    std::chrono::steady_clock::time_point start_;
};

/// 虚拟时钟：只能前进，由 Runtime 或测试推进。
class VirtualClock final : public IClock {
public:
    explicit VirtualClock(Time start = Time{0}) : ns_(start.count()) {}

    [[nodiscard]] Time now() const override { return Time{ns_.load(std::memory_order_acquire)}; }
    [[nodiscard]] bool isVirtual() const noexcept override { return true; }

    /// 推进到 t（不回退）。
    void advanceTo(Time t) {
        std::int64_t current = ns_.load(std::memory_order_relaxed);
        while (t.count() > current
               && !ns_.compare_exchange_weak(current, t.count(), std::memory_order_acq_rel)) {
        }
    }
    void advanceBy(Duration d) { advanceTo(now() + d); }

private:
    std::atomic<std::int64_t> ns_;
};

/// 定时器队列。fire 回调只应做投递（post 到属主执行上下文），不在队列锁内做业务。
///
/// 两条约定：
///
/// 1. **取消即释放，且在任何阶段都有效**。取消直接摘除条目并置死 alive 标志，
///    回调连同它捕获的对象一起放掉。只把 id 记进一个"已取消"集合是不够的：条目要
///    留到本该到期的那一刻才被清掉，周期定时器取消后 size() 恒为 1，捕获的模块指针
///    一直存活，而那个集合只增不减。
///    已经被 fireDue() 取出、正排队等着执行的那一条同样可取消：它的索引保留到这一轮
///    触发结束，所以同一批到期的两个定时器里，先执行的那个取消后一个，后一个就不会
///    再触发。**线性化点**是 cancel() 返回：此后不会再开始该定时器的任何一次触发；
///    cancel() 调用时已经在执行的那一次不受影响。
/// 2. **更早的 deadline 要叫醒等待者**。在线循环按当前最早到期时刻 wait_for；
///    插入一个更早的定时器如果不唤醒它，就要一直睡到旧的 deadline。队列不认识
///    运行时的条件变量，所以由使用方装一个 waker 进来。
class TimerQueue {
public:
    using Id = std::uint64_t;
    static constexpr Id kInvalidId = 0;

    /// 最早到期时刻可能提前时调用的唤醒出口。由运行时装入（notify 其定时器 cv）。
    /// 在队列锁外调用，因此 waker 里再回调队列不会自锁。
    void setWaker(std::function<void()> waker) {
        std::lock_guard lock(mutex_);
        waker_ = std::move(waker);
    }

    /// period 为零表示一次性定时器。返回的 id 可用于取消。
    Id schedule(Time due, Duration period, std::function<void()> fire) {
        std::function<void()> waker;
        Id id = kInvalidId;
        {
            std::lock_guard lock(mutex_);
            id = ++next_id_;
            Entry entry;
            entry.id = id;
            entry.period = period;
            entry.fire = std::move(fire);
            entry.alive = std::make_shared<std::atomic<bool>>(true);
            index_.emplace(id, Handle{due, entry.alive});
            const bool earlier = entries_.empty() || due < entries_.begin()->first;
            entries_.emplace(due, std::move(entry));
            if (earlier) waker = waker_;
        }
        if (waker) waker();
        return id;
    }

    /// 取消。摘除条目、释放捕获对象，并让已经取出待执行的那一次不再触发。
    void cancel(Id id) {
        std::function<void()> released;  ///< 锁外析构：回调可能捕获了大对象
        {
            std::lock_guard lock(mutex_);
            auto it = index_.find(id);
            if (it == index_.end()) return;
            it->second.alive->store(false, std::memory_order_release);
            auto range = entries_.equal_range(it->second.due);
            for (auto e = range.first; e != range.second; ++e) {
                if (e->second.id != id) continue;
                released = std::move(e->second.fire);
                entries_.erase(e);
                break;
            }
            index_.erase(it);
        }
    }

    /// 取出所有到期项并在锁外调用其 fire。返回触发条数。
    ///
    /// 一次性条目在这一轮触发结束之前仍留在索引里，因此同批里先执行的回调仍能取消
    /// 后执行的那一条；触发结束后再把它们从索引里清掉。回调对象的析构同样在锁外：
    /// 一次性的回调整个搬出去，周期的条目整个搬走再放回，锁内不留任何要析构捕获对象的副本。
    std::size_t fireDue(Time now) {
        std::vector<std::pair<std::shared_ptr<std::atomic<bool>>, std::function<void()>>> due;
        std::vector<Id> one_shots;
        {
            std::lock_guard lock(mutex_);
            for (auto it = entries_.begin(); it != entries_.end() && it->first <= now;) {
                if (it->second.period.count() > 0) {
                    due.emplace_back(it->second.alive, it->second.fire);  // 执行用的副本
                    // 周期定时器：以"上次到期时间 + 周期"推进，积压时不补发多次。
                    Time next = it->first + it->second.period;
                    if (next <= now) {
                        const auto missed = (now - it->first) / it->second.period;
                        next = it->first + it->second.period * (missed + 1);
                    }
                    Entry moved = std::move(it->second);
                    index_.insert_or_assign(moved.id, Handle{next, moved.alive});
                    it = entries_.erase(it);
                    entries_.emplace(next, std::move(moved));
                } else {
                    due.emplace_back(it->second.alive, std::move(it->second.fire));
                    one_shots.push_back(it->second.id);  // 索引留到本轮结束，仍可取消
                    it = entries_.erase(it);
                }
            }
        }
        std::size_t fired = 0;
        for (auto& [alive, fire] : due) {
            // 取出之后、执行之前被取消的，不再触发
            if (!alive->load(std::memory_order_acquire)) continue;
            fire();
            ++fired;
        }
        if (!one_shots.empty()) {
            std::lock_guard lock(mutex_);
            for (const Id id : one_shots) index_.erase(id);
        }
        return fired;
    }

    /// 把所有待触发的条目整体后移 offset。离线运行确定时钟原点时用：装配期按 0 时刻
    /// 排下的定时器改从原点起算，而不是在长时间戳的第一步集体"补"触发一次。
    void shiftAll(Duration offset) {
        std::lock_guard lock(mutex_);
        std::multimap<Time, Entry> shifted;
        for (auto& [due, entry] : entries_) {
            index_.insert_or_assign(entry.id, Handle{due + offset, entry.alive});
            shifted.emplace(due + offset, std::move(entry));
        }
        entries_.swap(shifted);  // 换下来的都是搬空了的条目，锁内析构不跑用户代码
    }

    [[nodiscard]] std::optional<Time> nextDue() const {
        std::lock_guard lock(mutex_);
        if (entries_.empty()) return std::nullopt;
        return entries_.begin()->first;  // 已取消的条目不在队列里
    }

    [[nodiscard]] std::size_t size() const {
        std::lock_guard lock(mutex_);
        return entries_.size();
    }

private:
    struct Entry {
        Id id = kInvalidId;
        Duration period{0};
        std::function<void()> fire;
        std::shared_ptr<std::atomic<bool>> alive;
    };
    struct Handle {
        Time due{};
        std::shared_ptr<std::atomic<bool>> alive;
    };

    mutable std::mutex mutex_;
    std::multimap<Time, Entry> entries_;
    std::unordered_map<Id, Handle> index_;
    std::function<void()> waker_;
    Id next_id_ = kInvalidId;
};

}  // namespace simpleslam::comm
