#pragma once
#include <queue>
#include <mutex>
#include <optional>
#include <functional>

namespace SimpleSLAM {
// MPSCQueue: Multi-Producer Single-Consumer queue
// Phase 1: mutex-based implementation
// Phase 3+: switch to moodycamel::ConcurrentQueue for lock-free performance
template<typename T>
class MPSCQueue {
    std::queue<T> queue_;
    mutable std::mutex mutex_;
public:
    void push(T item) {
        std::lock_guard lock(mutex_);
        queue_.push(std::move(item));
    }
    std::optional<T> try_pop() {
        std::lock_guard lock(mutex_);
        if (queue_.empty()) return std::nullopt;
        T item = std::move(queue_.front());
        queue_.pop();
        return std::move(item);
    }
    bool empty() const {
        std::lock_guard lock(mutex_);
        return queue_.empty();
    }
    size_t size() const {
        std::lock_guard lock(mutex_);
        return queue_.size();
    }
};
} // namespace SimpleSLAM
