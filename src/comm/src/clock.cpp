// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/comm/clock.hpp>

#include <algorithm>
#include <utility>

namespace simpleslam::comm {

TimerHandle TimerQueue::schedule(Time due, Duration period, std::function<void()> fire,
                                 TimerHandle::Alive alive) {
    if (!alive) alive = std::make_shared<std::atomic<bool>>(true);
    const auto seq = sequence_->fetch_add(1, std::memory_order_relaxed);
    {
        std::lock_guard lock(mutex_);
        entries_.emplace(Key{due.count(), seq}, Entry{period, std::move(fire), alive});
    }
    return TimerHandle(std::move(alive));
}

void TimerQueue::takeDue(Time now, std::vector<Due>& out) {
    std::lock_guard lock(mutex_);
    for (auto it = entries_.begin(); it != entries_.end() && it->first.first <= now.count();) {
        Entry entry = std::move(it->second);
        out.push_back({Time{it->first.first}, it->first.second, entry.period, std::move(entry.fire),
                       std::move(entry.alive), this});
        it = entries_.erase(it);
    }
}

void TimerQueue::rearm(const Due& due, Time now) {
    Time next = due.at + due.period;
    if (next <= now) next = due.at + due.period * ((now - due.at) / due.period + 1);
    const auto seq = sequence_->fetch_add(1, std::memory_order_relaxed);
    std::lock_guard lock(mutex_);
    entries_.emplace(Key{next.count(), seq}, Entry{due.period, due.fire, due.alive});
}

void TimerQueue::shiftAll(Duration offset) {
    std::lock_guard lock(mutex_);
    std::map<Key, Entry> shifted;
    for (auto& [key, entry] : entries_) {
        shifted.emplace(Key{key.first + offset.count(), key.second}, std::move(entry));
    }
    entries_.swap(shifted);
}

void TimerQueue::clear() {
    std::map<Key, Entry> doomed;
    std::lock_guard lock(mutex_);
    doomed.swap(entries_);
}

std::optional<Time> TimerQueue::nextDue() {
    std::lock_guard lock(mutex_);
    if (entries_.empty()) return std::nullopt;
    return Time{entries_.begin()->first.first};
}

std::size_t fireDueTimers(const std::vector<TimerQueue*>& queues, Time now) {
    std::vector<TimerQueue::Due> due;
    for (auto* queue : queues) queue->takeDue(now, due);
    if (due.empty()) return 0;
    std::sort(due.begin(), due.end(), [](const auto& a, const auto& b) {
        return a.at != b.at ? a.at < b.at : a.seq < b.seq;
    });
    for (const auto& item : due) {
        if (item.period.count() > 0 && item.alive->load(std::memory_order_acquire)) {
            item.owner->rearm(item, now);
        }
    }
    std::size_t fired = 0;
    for (auto& item : due) {
        if (!item.alive->load(std::memory_order_acquire)) continue;
        item.fire();
        ++fired;
    }
    return fired;
}

}  // namespace simpleslam::comm
