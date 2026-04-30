#include <SimpleSLAM/core/infra/topic_hub.hpp>

namespace simpleslam {

std::unique_ptr<TopicHub> TopicHub::global_instance_;
std::mutex TopicHub::global_mutex_;

TopicHub::TopicHub(bool offline_mode) : offline_mode_(offline_mode) {}

void TopicHub::init(bool offline_mode) {
    std::lock_guard lock(global_mutex_);
    global_instance_ = std::make_unique<TopicHub>(offline_mode);
}

void TopicHub::shutdown() {
    std::lock_guard lock(global_mutex_);
    global_instance_.reset();
}

TopicHub& TopicHub::instance() {
    std::lock_guard lock(global_mutex_);
    assert(global_instance_ && "TopicHub::init() not called");
    return *global_instance_;
}

size_t TopicHub::drainAll() {
    size_t total = 0;
    bool had_pending = true;
    while (had_pending) {
        had_pending = false;
        std::lock_guard lock(mutex_);
        for (auto& [_, entry] : topics_) {
            if (entry.base->hasPending()) {
                entry.base->drainOnce();
                ++total;
                had_pending = true;
            }
        }
    }
    return total;
}

std::vector<std::string> TopicHub::listTopics() {
    auto& hub = instance();
    std::lock_guard lock(hub.mutex_);
    std::vector<std::string> names;
    names.reserve(hub.topics_.size());
    for (const auto& [name, _] : hub.topics_) {
        names.push_back(name);
    }
    return names;
}

TopicHub::TopicStats TopicHub::stats(std::string_view name) {
    auto& hub = instance();
    std::lock_guard lock(hub.mutex_);
    std::string key(name);
    auto it = hub.topics_.find(key);
    if (it == hub.topics_.end()) {
        throw std::runtime_error("TopicHub: topic not found: " + key);
    }
    return {key,
            it->second.base->publisherCount(),
            it->second.base->subscriberCount(),
            it->second.base->messageCount()};
}

}  // namespace simpleslam
