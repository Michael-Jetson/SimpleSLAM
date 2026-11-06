// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/comm/mailbox.hpp>

#include <algorithm>
#include <chrono>
#include <utility>

namespace simpleslam::comm {

void Hub::recordFailure(ModuleFailure failure, bool stop) {
    std::function<void()> hook;
    {
        std::lock_guard lock(failure_mutex_);
        if (stop) {
            if (!fatal_) fatal_ = failure.cause;
            hook = stop_hook_;
        }
        if (!first_failure_) first_failure_ = std::move(failure);
    }
    if (hook) hook();
}

std::optional<ModuleFailure> Hub::firstFailure() const {
    std::lock_guard lock(failure_mutex_);
    return first_failure_;
}

std::exception_ptr Hub::takeFatal() {
    std::lock_guard lock(failure_mutex_);
    return std::exchange(fatal_, nullptr);
}

void Hub::setStopHook(std::function<void()> hook) {
    std::lock_guard lock(failure_mutex_);
    stop_hook_ = std::move(hook);
}

void Hub::pushReady(MailboxPtr box) {
    std::lock_guard lock(ready_mutex_);
    ready_.emplace_back(box);
}

void Hub::setScheduleSeed(std::uint64_t seed) noexcept {
    std::lock_guard lock(ready_mutex_);
    rng_ = seed == 0 ? 0 : seed * 2654435761ULL + 88172645463325252ULL;
}

MailboxPtr Hub::popReady() {
    std::lock_guard lock(ready_mutex_);
    while (!ready_.empty()) {
        std::size_t pick = 0;
        if (rng_ != 0) {
            rng_ ^= rng_ << 13;
            rng_ ^= rng_ >> 7;
            rng_ ^= rng_ << 17;
            pick = static_cast<std::size_t>(rng_ % ready_.size());
        }
        MailboxPtr box = ready_[pick].lock();
        ready_.erase(ready_.begin() + static_cast<std::ptrdiff_t>(pick));
        if (box) return box;
    }
    return nullptr;
}

void Hub::clearReady() {
    std::lock_guard lock(ready_mutex_);
    ready_.clear();
}

void Hub::finishWork(std::size_t n) {
    if (work_.fetch_sub(n, std::memory_order_acq_rel) == n) {
        { std::lock_guard lock(idle_mutex_); }
        idle_cv_.notify_all();
    }
}

bool Hub::waitIdle(std::chrono::nanoseconds timeout) {
    std::unique_lock lock(idle_mutex_);
    const auto done = [this] { return idle(); };
    if (timeout.count() > 0) return idle_cv_.wait_for(lock, timeout, done);
    idle_cv_.wait(lock, done);
    return true;
}

bool Mailbox::push(Item item) {
    bool make_ready = false;
    bool wake_thread = false;
    {
        std::lock_guard lock(mutex_);
        if (closed_) {
            ++discarded_;
            return false;
        }
        hub_->addWork();
        items_.push_back(std::move(item));
        if (threaded_) {
            wake_thread = true;
        } else if (!queued_) {
            queued_ = true;
            make_ready = true;
        }
    }
    if (wake_thread) cv_.notify_one();  // 解锁后再唤醒，线程醒来不用先等这把锁
    if (make_ready) hub_->pushReady(shared_from_this());
    return true;
}

std::size_t Mailbox::drainSome() {
    for (std::size_t n = 0; n < kDrainBudget; ++n) {
        Item item;
        {
            std::lock_guard lock(mutex_);
            if (closed_ || items_.empty()) {
                queued_ = false;
                return n;
            }
            item = std::move(items_.front());
            items_.pop_front();
        }
        run(item);
        item = Item{};
        hub_->finishWork();
    }
    bool more = false;
    {
        std::lock_guard lock(mutex_);
        more = !closed_ && !items_.empty();
        if (!more) queued_ = false;
    }
    if (more) hub_->pushReady(shared_from_this());
    return kDrainBudget;
}

TimerHandle Mailbox::scheduleTimer(Time due, Duration period, std::function<void()> fire,
                                   TimerHandle::Alive alive) {
    TimerHandle handle;
    {
        std::lock_guard lock(mutex_);
        if (closed_) return handle;
        handle = timers_.schedule(due, period, std::move(fire), std::move(alive));
        timers_changed_ = true;
    }
    cv_.notify_all();
    return handle;
}

void Mailbox::threadLoop(IClock& clock) {
    std::unique_lock lock(mutex_);
    threaded_ = true;
    queued_ = false;
    const auto woken = [this] { return thread_stop_ || closed_ || timers_changed_ || !items_.empty(); };
    for (;;) {
        // 先清标志再读最早到期时刻：清之后才登记的定时器，标志会再次置位
        timers_changed_ = false;
        // 输入停了就不再触发定时器：停止时不产生新工作
        const auto due = hub_->inputsStopped() ? std::nullopt : timers_.nextDue();
        if (due) {
            cv_.wait_for(lock, *due - clock.now(), woken);
        } else {
            cv_.wait(lock, woken);
        }
        if (thread_stop_ || closed_) break;
        if (due && *due <= clock.now() && !hub_->inputsStopped()) {
            lock.unlock();
            fireDueTimers({&timers_}, clock.now());
            lock.lock();
        }
        if (items_.empty()) continue;
        Item item = std::move(items_.front());
        items_.pop_front();
        lock.unlock();
        run(item);
        item = Item{};
        hub_->finishWork();
        lock.lock();
    }
    threaded_ = false;
    const bool more = !closed_ && !items_.empty();
    if (more) queued_ = true;
    lock.unlock();
    if (more) hub_->pushReady(shared_from_this());
}

void Mailbox::stopThread() {
    {
        std::lock_guard lock(mutex_);
        thread_stop_ = true;
    }
    cv_.notify_all();
}

void Mailbox::closeAdmission() {
    {
        std::lock_guard lock(mutex_);
        closed_ = true;
        for (auto& item : items_) discarded_items_.push_back(std::move(item));
        items_.clear();
    }
    cv_.notify_all();
}

void Mailbox::releaseDiscarded() {
    std::deque<Item> discarded;
    {
        std::lock_guard lock(mutex_);
        discarded.swap(discarded_items_);
    }
    const std::size_t n = discarded.size();
    discarded_ += n;
    discarded.clear();  // 任务析构时可能结案、再投递
    if (n > 0) hub_->finishWork(n);
    timers_.clear();
}

bool Mailbox::closed() const {
    std::lock_guard lock(mutex_);
    return closed_;
}

MailboxStats Mailbox::stats() const {
    MailboxStats s;
    s.callbacks = callbacks_.load(std::memory_order_relaxed);
    s.busy_ns = busy_ns_.load(std::memory_order_relaxed);
    s.max_callback_ns = max_callback_ns_.load(std::memory_order_relaxed);
    s.failures = failures_.load(std::memory_order_relaxed);
    s.discarded = discarded_.load(std::memory_order_relaxed);
    std::lock_guard lock(mutex_);
    s.depth = items_.size();
    return s;
}

/// 模块里的一切回调都从这里进入，这里是异常的最后一道边界。
void Mailbox::run(Item& item) noexcept {
    const auto start = std::chrono::steady_clock::now();
    if (scope_ != nullptr) scope_->enterCallback();
    bool completed = false;
    try {
        if (item.target) {
            item.target->deliverOne();
        } else if (item.task) {
            item.task();
        }
        completed = true;
    } catch (const std::exception& e) {
        fail(e.what());
    } catch (...) {
        fail("模块回调抛出未知异常");
    }
    if (scope_ != nullptr) scope_->leaveCallback(completed);
    const auto ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count());
    callbacks_.fetch_add(1, std::memory_order_relaxed);
    busy_ns_.fetch_add(ns, std::memory_order_relaxed);
    auto seen = max_callback_ns_.load(std::memory_order_relaxed);
    while (ns > seen && !max_callback_ns_.compare_exchange_weak(seen, ns, std::memory_order_relaxed)) {
    }
}

/// 只在 catch 块里调用。
void Mailbox::fail(const char* what) noexcept {
    failures_.fetch_add(1, std::memory_order_relaxed);
    detail::reportSafely(reporter_, owner_, what);
    try {
        hub_->recordFailure({owner_, what, std::current_exception()},
                            policy_.load(std::memory_order_relaxed) == ExceptionPolicy::kStop);
    } catch (...) {  // NOLINT(bugprone-empty-catch)
    }
}

}  // namespace simpleslam::comm
