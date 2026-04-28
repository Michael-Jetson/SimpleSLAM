#pragma once

/// @file topic_hub.hpp
/// TopicHub —— 全局单例话题注册表 + BFS 协调分发
///
/// 参考 spdlog 模式：有默认全局实例供日常使用，也可创建隔离实例供单元测试。
/// Runner 启动时 init()，关闭时 shutdown()。
/// 所有模块通过静态方法使用，不需要传入 TopicHub& 引用。

#include <SimpleSLAM/core/infra/topic.hpp>

#include <any>
#include <cassert>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace simpleslam {

class TopicHub {
public:
    // ── 全局单例 ──

    static void init(bool offline_mode = true);
    static void shutdown();
    static TopicHub& instance();

    // ── 静态 Publisher 创建（用默认 hub）──

    template <typename T>
    static Publisher<T> createPublisher(const std::string& name,
                                       QoS qos = QoS::Event) {
        return instance().createPublisherImpl<T>(name, qos);
    }

    // ── 静态 Subscriber 创建：成员函数 ──

    template <typename T, typename Obj, typename Method>
    static SubscriptionHandle createSubscriber(const std::string& name,
                                               Method method, Obj* obj,
                                               SubscribeOptions opts = {}) {
        auto cb = detail::wrapCallback<T>(method, obj);
        return instance().subscribeImpl<T>(name, std::move(cb), opts);
    }

    // ── 静态 Subscriber 创建：lambda / 自由函数 / std::function ──

    template <typename T, typename Callable>
    static SubscriptionHandle createSubscriber(const std::string& name,
                                               Callable&& callback,
                                               SubscribeOptions opts = {}) {
        auto cb = detail::wrapCallback<T>(std::forward<Callable>(callback));
        return instance().subscribeImpl<T>(name, std::move(cb), opts);
    }

    // ── 最新值查询 ──

    template <typename T>
    static MsgPtr<T> getLatest(const std::string& name) {
        return instance().getLatestImpl<T>(name);
    }

    // ── Offline BFS 协调 ──

    /// 逐轮分发 pending 消息，回调中新 publish 进入下一轮
    size_t drainAll();

    // ── 自省 ──

    static std::vector<std::string> listTopics();

    struct TopicStats {
        std::string name;
        size_t publisher_count;
        size_t subscriber_count;
        uint64_t message_count;
    };
    static TopicStats stats(const std::string& name);

    // ── 构造函数（隔离实例，测试用）──

    explicit TopicHub(bool offline_mode);
    ~TopicHub() = default;

    TopicHub(const TopicHub&) = delete;
    TopicHub& operator=(const TopicHub&) = delete;

    // ── 实例方法（测试/隔离用）──

    template <typename T>
    Publisher<T> createPublisherImpl(const std::string& name,
                                    QoS qos = QoS::Event) {
        auto* topic = getOrCreateTopic<T>(name, qos);
        return Publisher<T>(topic);
    }

    template <typename T>
    SubscriptionHandle subscribeImpl(const std::string& name,
                                     std::function<void(MsgPtr<T>)> cb,
                                     SubscribeOptions opts = {}) {
        auto* topic = getOrCreateTopic<T>(name, QoS::Event);
        return topic->subscribe(std::move(cb), opts);
    }

    template <typename T>
    MsgPtr<T> getLatestImpl(const std::string& name) {
        std::lock_guard lock(mutex_);
        auto it = topics_.find(name);
        if (it == topics_.end()) return nullptr;
        auto* typed = std::any_cast<Topic<T>*>(&it->second.typed);
        if (!typed) return nullptr;
        return (*typed)->latest();
    }

    [[nodiscard]] bool hasTopic(const std::string& name) const {
        std::lock_guard lock(mutex_);
        return topics_.count(name) > 0;
    }

private:
    struct TopicEntry {
        std::unique_ptr<ITopicBase> base;
        std::any typed;  ///< Topic<T>*
    };

    /// 获取或按需创建 Topic（首次 createPublisher/createSubscriber 时自动创建）
    template <typename T>
    Topic<T>* getOrCreateTopic(const std::string& name, QoS qos) {
        std::lock_guard lock(mutex_);
        auto it = topics_.find(name);
        if (it != topics_.end()) {
            auto* ptr = std::any_cast<Topic<T>*>(&it->second.typed);
            if (!ptr) {
                throw std::runtime_error(
                    "TopicHub: type mismatch for topic: " + name);
            }
            return *ptr;
        }
        auto topic = std::make_unique<Topic<T>>(name, qos);
        topic->setOfflineMode(offline_mode_);
        auto* raw = topic.get();
        TopicEntry entry;
        entry.base = std::move(topic);
        entry.typed = raw;
        topics_[name] = std::move(entry);
        return raw;
    }

    mutable std::mutex mutex_;
    std::unordered_map<std::string, TopicEntry> topics_;
    bool offline_mode_{true};

    static std::unique_ptr<TopicHub> global_instance_;
    static std::mutex global_mutex_;
};

}  // namespace simpleslam
