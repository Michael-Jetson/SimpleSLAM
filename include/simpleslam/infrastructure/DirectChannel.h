#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace SimpleSLAM {

template<typename T>
class DirectChannel {
public:
    explicit DirectChannel(size_t capacity)
        : capacity_(capacity) {
        if (capacity_ == 0) {
            throw std::invalid_argument("DirectChannel capacity must be greater than 0");
        }
    }

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
        // Swap-then-process: acquire the lock only once, then process the detached batch.
        // Items pushed during handler execution go into the fresh queue_ and are deferred
        // until the next drainAll() call. This mirrors TopicBus's next-round behavior,
        // but DirectChannel remains a point-to-point queue rather than a pub/sub dispatcher.
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
