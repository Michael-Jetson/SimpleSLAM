#include <simpleslam/infrastructure/TopicBus.h>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace SimpleSLAM {

void TopicBus::registerThread(std::string name, CallbackQueuePtr queue) {
    if (name.empty()) {
        throw std::invalid_argument("TopicBus registerThread requires a non-empty thread name");
    }
    if (!queue) {
        throw std::invalid_argument("TopicBus registerThread requires a valid queue");
    }

    std::lock_guard lock(mutex_);
    thread_queues_[std::move(name)] = std::move(queue);
}

void TopicBus::unsubscribe(const std::string& topic, Handle handle) {
    std::lock_guard lock(mutex_);

    auto it = listeners_.find(topic);
    if (it == listeners_.end()) {
        return;
    }

    auto& entries = it->second;
    entries.erase(std::remove_if(entries.begin(), entries.end(),
                                 [handle](const ListenerEntry& entry) {
                                     return entry.id == handle;
                                 }),
                  entries.end());

    if (entries.empty()) {
        listeners_.erase(it);
    }
}

void TopicBus::publish(const std::string& topic, std::any data) {
    std::lock_guard lock(mutex_);
    pending_events_.push_back(PendingEvent{topic, std::move(data)});
}

void TopicBus::drainAll() {
    std::vector<PendingEvent> batch;
    {
        std::lock_guard lock(mutex_);
        batch.swap(pending_events_);
    }

    for (auto& event : batch) {
        dispatchToListeners(event.topic, event.payload);
    }
}

size_t TopicBus::pendingCount() const {
    std::lock_guard lock(mutex_);
    return pending_events_.size();
}

void TopicBus::dispatchToListeners(const std::string& topic, const std::any& data) {
    auto snapshot = snapshotListenersLocked(topic);
    for (auto& listener : snapshot) {
        try {
            listener(data);
        } catch (const std::bad_any_cast& e) {
            // Type mismatch between publisher and subscriber — log and continue
            std::fprintf(stderr, "[TopicBus] bad_any_cast on topic '%s': %s\n",
                         topic.c_str(), e.what());
        } catch (const std::exception& e) {
            // Subscriber callback threw — isolate and continue dispatching
            std::fprintf(stderr, "[TopicBus] exception in listener on topic '%s': %s\n",
                         topic.c_str(), e.what());
        } catch (...) {
            std::fprintf(stderr, "[TopicBus] unknown exception in listener on topic '%s'\n",
                         topic.c_str());
        }
    }
}

std::vector<TopicBus::Listener> TopicBus::snapshotListenersLocked(const std::string& topic) const {
    std::lock_guard lock(mutex_);

    auto it = listeners_.find(topic);
    if (it == listeners_.end()) {
        return {};
    }

    std::vector<Listener> snapshot;
    snapshot.reserve(it->second.size());
    for (const auto& entry : it->second) {
        snapshot.push_back(entry.callback);
    }

    return snapshot;
}

} // namespace SimpleSLAM
