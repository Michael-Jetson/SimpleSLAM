#pragma once

#include <any>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <simpleslam/core/ConcurrentQueue.h>
#include <simpleslam/core/IService.h>

namespace SimpleSLAM {

struct SubscribeOptions {
    std::string target_thread;
};

class ScopedSubscription;  // forward declare, defined after TopicBus

class TopicBus : public IService {
public:
    using Handle = std::uint64_t;
    using CallbackQueue = MPSCQueue<std::function<void()>>;
    using CallbackQueuePtr = std::shared_ptr<CallbackQueue>;

    TopicBus() = default;
    ~TopicBus() override = default;

    TopicBus(const TopicBus&) = delete;
    TopicBus& operator=(const TopicBus&) = delete;
    TopicBus(TopicBus&&) = delete;
    TopicBus& operator=(TopicBus&&) = delete;

    void registerThread(std::string name, CallbackQueuePtr queue);

    template<typename T, typename Callback>
    Handle subscribe(const std::string& topic, Callback&& callback, SubscribeOptions opts = {}) {
        using Message = std::remove_cvref_t<T>;
        static_assert(std::is_invocable_v<Callback&, const Message&>,
                      "TopicBus callback must accept const T&");

        std::function<void(const Message&)> typed_callback(std::forward<Callback>(callback));
        Listener listener;

        std::lock_guard lock(mutex_);

        if (typed_callback == nullptr) {
            throw std::invalid_argument("TopicBus subscribe requires a valid callback");
        }

        if (opts.target_thread.empty()) {
            listener = makeDirectListener<Message>(std::move(typed_callback));
        } else {
            auto queue_it = thread_queues_.find(opts.target_thread);
            if (queue_it == thread_queues_.end() || !queue_it->second) {
                throw std::invalid_argument(
                    "TopicBus target_thread is not registered: " + opts.target_thread);
            }
            listener = makeQueuedListener<Message>(queue_it->second, std::move(typed_callback));
        }

        const Handle handle = next_handle_++;
        listeners_[topic].push_back(ListenerEntry{handle, std::move(listener)});
        return handle;
    }

    void unsubscribe(const std::string& topic, Handle handle);

    /// RAII version of subscribe — declared here, defined after ScopedSubscription
    template<typename T, typename Callback>
    [[nodiscard]] ScopedSubscription scopedSubscribe(
        const std::string& topic, Callback&& callback, SubscribeOptions opts = {});

    void publish(const std::string& topic, std::any data);

    template<typename T>
        requires(!std::is_same_v<std::remove_cvref_t<T>, std::any>)
    void publish(const std::string& topic, T&& data) {
        publish(topic, std::any(std::forward<T>(data)));
    }

    void drainAll();

    [[nodiscard]] size_t pendingCount() const;

private:
    using Listener = std::function<void(const std::any&)>;

    struct ListenerEntry {
        Handle id = 0;
        Listener callback;
    };

    struct PendingEvent {
        std::string topic;
        std::any payload;
    };

    template<typename T>
    static Listener makeDirectListener(std::function<void(const T&)> callback) {
        return [callback = std::move(callback)](const std::any& data) {
            callback(std::any_cast<const T&>(data));
        };
    }

    template<typename T>
    static Listener makeQueuedListener(CallbackQueuePtr queue,
                                       std::function<void(const T&)> callback) {
        return [queue = std::move(queue), callback = std::move(callback)](const std::any& data) {
            auto payload = std::make_shared<std::any>(data);
            queue->push([callback, payload = std::move(payload)]() {
                callback(std::any_cast<const T&>(*payload));
            });
        };
    }

    void dispatchToListeners(const std::string& topic, const std::any& data);
    [[nodiscard]] std::vector<Listener> snapshotListenersLocked(const std::string& topic) const;

    std::unordered_map<std::string, std::vector<ListenerEntry>> listeners_;
    std::vector<PendingEvent> pending_events_;
    std::unordered_map<std::string, CallbackQueuePtr> thread_queues_;
    mutable std::mutex mutex_;
    Handle next_handle_ = 1;
};

/// RAII subscription handle — automatically unsubscribes on destruction.
/// Usage:
///   auto sub = bus.scopedSubscribe<KeyframeData>("NewKeyFrame", callback);
///   // sub goes out of scope → automatically unsubscribed
class ScopedSubscription {
public:
    ScopedSubscription() = default;
    ScopedSubscription(TopicBus* bus, std::string topic, TopicBus::Handle handle)
        : bus_(bus), topic_(std::move(topic)), handle_(handle) {}

    ~ScopedSubscription() { reset(); }

    // Move-only (no copy)
    ScopedSubscription(const ScopedSubscription&) = delete;
    ScopedSubscription& operator=(const ScopedSubscription&) = delete;

    ScopedSubscription(ScopedSubscription&& other) noexcept
        : bus_(other.bus_), topic_(std::move(other.topic_)), handle_(other.handle_) {
        other.bus_ = nullptr;
    }
    ScopedSubscription& operator=(ScopedSubscription&& other) noexcept {
        if (this != &other) {
            reset();
            bus_ = other.bus_;
            topic_ = std::move(other.topic_);
            handle_ = other.handle_;
            other.bus_ = nullptr;
        }
        return *this;
    }

    void reset() {
        if (bus_) {
            bus_->unsubscribe(topic_, handle_);
            bus_ = nullptr;
        }
    }

    [[nodiscard]] TopicBus::Handle handle() const { return handle_; }
    [[nodiscard]] bool active() const { return bus_ != nullptr; }

private:
    TopicBus* bus_ = nullptr;
    std::string topic_;
    TopicBus::Handle handle_ = 0;
};

// Deferred definition of scopedSubscribe (requires complete ScopedSubscription)
template<typename T, typename Callback>
ScopedSubscription TopicBus::scopedSubscribe(
    const std::string& topic, Callback&& callback, SubscribeOptions opts) {
    Handle h = subscribe<T>(topic, std::forward<Callback>(callback), std::move(opts));
    return ScopedSubscription(this, topic, h);
}

} // namespace SimpleSLAM
