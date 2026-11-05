// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file mailbox.hpp
/// 模块收件箱：模块级 FIFO 的唯一入口。
///
/// 串行执行只保证"同一模块的回调不并发"，并不保证"模块看到的顺序 = 发布顺序"：
/// 若每个订阅端各自持队列、各自整批处理，先发布的 B 可能排在后发布的 A 之后。
/// 因此把所有已接纳的投递排进同一条队列——不同话题的消息、参数换代、定时器到期、
/// 服务响应，全都经过它。这才是 ADR-0010 所说的"固定模块序 + 各模块 FIFO"。
///
/// ## 有界性
///
/// 消息侧的容量约束在订阅端完成：收件箱只排"已被接纳"的消息，每条恰好一格，
/// kLatest 的覆盖不产生新格。**控制类工作**（参数换代、定时器到期、服务响应）
/// 走的是 postTask()，它自己也要有界——一个模块不被驱动时，仅一个周期定时器
/// 触发一千次就能堆一千格。kDrainBudget 限制的是"一次处理多少"，不是"能积压多少"。
///
/// 控制类按入口分两类：
///
/// - **可合并**（coalesce_key 非零）：同键至多保留一格待执行。周期唤醒用它——
///   醒来要做的事是"去看一眼当前状态"，积压一千次唤醒没有意义。
/// - **必须逐条保留**（coalesce_key 为零）：参数换代、服务终态。满载时**明确拒绝**
///   并让调用方看见，不静默丢。
///
/// 两类共用同一个 max_tasks 预算，和消息共用同一条 FIFO。
///
/// ## 异常的最后一道边界
///
/// 用户回调（订阅、定时器、完成通知）抛出的异常都在这里接住：执行边界只有这一处，
/// 接住即上报并计数。**接住不等于没发生**——那一份工作丢了，所以它必须能被数出来，
/// 运行结局据此判断处理是否完整。有领域语义的失败不走这里：服务处理函数的异常
/// 要变成服务错误响应，算法的异常要变成那一包的失败终态，它们在各自的位置处理；
/// 这里只兜底谁都没处理的那些。

#include <SimpleSLAM/comm/execution.hpp>
#include <SimpleSLAM/comm/types.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>

namespace simpleslam::comm {

/// 收件箱一格所指向的投递目标（通常是一个订阅端）。
class DeliveryTarget {
public:
    virtual ~DeliveryTarget() = default;
    DeliveryTarget(const DeliveryTarget&) = delete;
    DeliveryTarget& operator=(const DeliveryTarget&) = delete;

    /// 取出并处理恰好一条已接纳的消息。回调的异常不在这里吞：交给收件箱的执行边界
    /// 统一隔离并计数（见文件头）。
    virtual void deliverOne() = 0;

protected:
    DeliveryTarget() = default;
};

class Mailbox final : public std::enable_shared_from_this<Mailbox> {
public:
    /// 一次 drain 任务最多处理的格数。超出则重新排队，避免一个模块饿死其他模块。
    static constexpr std::size_t kDrainBudget = 64;
    /// 控制类工作的默认上限。消息侧另有订阅端容量，两者相加才是收件箱深度上限。
    static constexpr std::size_t kDefaultMaxTasks = 1024;

    static std::shared_ptr<Mailbox> create(IoContext& io, std::string owner,
                                           ErrorReporter reporter,
                                           std::size_t max_tasks = kDefaultMaxTasks) {
        return std::shared_ptr<Mailbox>(
            new Mailbox(io, std::move(owner), std::move(reporter), max_tasks));
    }

    Mailbox(const Mailbox&) = delete;
    Mailbox& operator=(const Mailbox&) = delete;

    [[nodiscard]] SerialExecutor& executor() noexcept { return executor_; }
    [[nodiscard]] const std::string& owner() const noexcept { return owner_; }

    /// 排一条已接纳的消息。target 的寿命由收件箱在处理期间持有。
    /// 容量已在订阅端把过关，这里不再设限，返回值只反映收件箱是否已关闭。
    bool postMessage(std::shared_ptr<DeliveryTarget> target) {
        return push(Item{std::move(target), {}, 0});
    }

    /// 排一件控制类工作：参数换代、定时器回调、服务响应。与消息同一条 FIFO。
    ///
    /// coalesce_key 非零时，同键的待执行工作至多保留一格：已有待执行的就丢掉这次
    /// （周期唤醒的语义是"去看一眼"，积压多次没有意义），返回 true。
    /// 返回 false 表示**没有排进去**：收件箱已关闭，或控制类工作已达上限。
    /// 必须逐条保留的工作遇到 false 要自己升级为明确失败，不能当作已投递。
    [[nodiscard]] bool postTask(std::function<void()> task, std::uint64_t coalesce_key = 0) {
        return push(Item{nullptr, std::move(task), coalesce_key});
    }

    /// 停止收件。已排队但未处理的格被丢弃——模块停止后不再产生业务回调。
    ///
    /// 被丢弃的**控制类**工作计入 undeliveredTasks()：服务终态、参数换代属于这一类，
    /// 静默丢掉会让"每次调用恰好一个终态"这种承诺悄悄失效。
    void close() {
        std::deque<Item> discarded;
        {
            std::lock_guard lock(mutex_);
            closed_ = true;
            discarded.swap(items_);
            pending_keys_.clear();
            tasks_ = 0;
        }
        for (const auto& item : discarded) {
            if (item.target == nullptr && item.task) ++undelivered_tasks_;
        }
        // 在锁外析构：被丢弃的消息可能持有大块点云，不应在收件箱锁内释放
    }

    [[nodiscard]] bool closed() const {
        std::lock_guard lock(mutex_);
        return closed_;
    }
    [[nodiscard]] std::size_t depth() const {
        std::lock_guard lock(mutex_);
        return items_.size();
    }
    /// 待执行的控制类工作数（不含消息格）。
    [[nodiscard]] std::size_t pendingTasks() const {
        std::lock_guard lock(mutex_);
        return tasks_;
    }
    /// 因为达到上限而被拒的控制类工作累计数。
    [[nodiscard]] std::size_t droppedTasks() const noexcept { return dropped_tasks_; }
    /// 没能送达的控制类工作累计数：关闭时从队列里清掉的，加上关闭之后才投来而被拒的。
    /// "关了之后没人收到的终态"有多少，看它。
    [[nodiscard]] std::size_t undeliveredTasks() const noexcept { return undelivered_tasks_; }
    /// 在执行边界被接住的异常数：每一次都意味着一份工作没做完。
    [[nodiscard]] std::uint64_t isolatedFailures() const noexcept { return isolated_failures_; }

private:
    struct Item {
        std::shared_ptr<DeliveryTarget> target;
        std::function<void()> task;
        std::uint64_t coalesce_key = 0;
    };

    Mailbox(IoContext& io, std::string owner, ErrorReporter reporter, std::size_t max_tasks)
        : executor_(io),
          owner_(std::move(owner)),
          reporter_(std::move(reporter)),
          max_tasks_(max_tasks == 0 ? 1 : max_tasks) {}

    bool push(Item item) {
        bool wake = false;
        bool rejected = false;
        {
            std::lock_guard lock(mutex_);
            if (closed_) {
                // 关闭之后仍有人投控制类工作（例如在飞调用的完成通知）：它没能送达，
                // 同样要可见
                if (item.target == nullptr && item.task) ++undelivered_tasks_;
                return false;
            }
            if (item.target == nullptr) {  // 控制类工作才受任务预算约束
                if (item.coalesce_key != 0 && pending_keys_.count(item.coalesce_key) != 0) {
                    return true;  // 同键已有待执行的一格，本次并入它
                }
                if (tasks_ >= max_tasks_) {
                    rejected = true;
                } else {
                    ++tasks_;
                    if (item.coalesce_key != 0) pending_keys_.insert(item.coalesce_key);
                }
            }
            if (!rejected) {
                items_.push_back(std::move(item));
                if (!pending_) {
                    pending_ = true;
                    wake = true;
                }
            }
        }
        if (rejected) {
            ++dropped_tasks_;
            detail::reportSafely(reporter_, owner_, "收件箱控制类工作已达上限，本次投递被拒");
            return false;
        }
        if (wake) schedule();
        return true;
    }

    void schedule() {
        auto self = shared_from_this();
        if (executor_.post([self] { self->drain(); })) return;
        // 执行底座已经释放（Runtime 已销毁，收件箱必然已关）：这一格不会再有人处理
        std::lock_guard lock(mutex_);
        pending_ = false;
    }

    void drain() {
        for (std::size_t processed = 0; processed < kDrainBudget; ++processed) {
            Item item;
            {
                std::lock_guard lock(mutex_);
                if (closed_ || items_.empty()) {
                    pending_ = false;
                    return;
                }
                item = std::move(items_.front());
                items_.pop_front();
                if (item.target == nullptr) {
                    if (tasks_ > 0) --tasks_;
                    if (item.coalesce_key != 0) pending_keys_.erase(item.coalesce_key);
                }
            }
            run(item);
        }
        bool more = false;
        {
            std::lock_guard lock(mutex_);
            more = !closed_ && !items_.empty();
            if (!more) pending_ = false;
        }
        if (more) schedule();
    }

    /// 执行一格。模块里的一切回调都从这里进入，所以这里是异常的最后一道边界。
    void run(Item& item) noexcept {
        try {
            if (item.target) {
                item.target->deliverOne();
            } else if (item.task) {
                item.task();
            }
        } catch (const std::exception& e) {
            ++isolated_failures_;
            detail::reportSafely(reporter_, owner_, e.what());
        } catch (...) {
            ++isolated_failures_;
            detail::reportSafely(reporter_, owner_, "模块回调抛出未知异常");
        }
    }

    SerialExecutor executor_;
    std::string owner_;
    ErrorReporter reporter_;
    std::size_t max_tasks_;
    mutable std::mutex mutex_;
    std::deque<Item> items_;
    std::unordered_set<std::uint64_t> pending_keys_;
    std::size_t tasks_ = 0;
    std::atomic<std::size_t> dropped_tasks_{0};
    std::atomic<std::size_t> undelivered_tasks_{0};
    std::atomic<std::uint64_t> isolated_failures_{0};
    bool pending_ = false;
    bool closed_ = false;
};

using MailboxPtr = std::shared_ptr<Mailbox>;

}  // namespace simpleslam::comm
