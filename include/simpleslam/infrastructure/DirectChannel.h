#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <utility>

namespace SimpleSLAM {

template<typename T>
class DirectChannel {
public:
    explicit DirectChannel(size_t capacity)
        : capacity_(capacity) {}

    bool tryPush(std::unique_ptr<T> data) {
        if (!data) return false;  // reject null — drainAll uses truthiness as loop condition
        std::lock_guard lock(mutex_);
        if (queue_.size() >= capacity_) {
            return false;
        }
        queue_.push_back(std::move(data));
        return true;
    }

    std::unique_ptr<T> tryPop() {
        std::lock_guard lock(mutex_);
        if (queue_.empty()) {
            return nullptr;
        }

        auto result = std::move(queue_.front());
        queue_.pop_front();
        return result;
    }

    template<typename Func>
    size_t drainAll(Func&& handler) {
        // Swap-then-process: acquire lock only once, matching TopicBus BFS semantics.
        // Items pushed by producer during handler execution go into the fresh queue_
        // and will be processed in the next drainAll() call.
        std::deque<std::unique_ptr<T>> batch;
        {
            std::lock_guard lock(mutex_);
            batch.swap(queue_);
        }
        for (auto& data : batch) {
            handler(std::move(data));
        }
        return batch.size();
    }

private:
    size_t capacity_;
    std::deque<std::unique_ptr<T>> queue_;
    std::mutex mutex_;
};

} // namespace SimpleSLAM
