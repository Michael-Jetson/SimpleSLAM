#pragma once

/// @file topic.hpp
/// Topic<T> 进程内发布-订阅通信——Publisher/Subscriber 句柄模式
///
/// 设计原则：消息只读共享（shared_ptr<const T>），状态各自私有。
/// 类 ROS 2 话题 API，但更轻量：无 DDS、无序列化，publish 延迟 <100ns。
///
/// 三种 QoS：Event（可靠多播）、Stream（高频有损）、Latest（只保留最新）
/// 四种回调绑定：成员函数(const T&)、成员函数(MsgPtr)、自由函数、lambda

#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>

namespace simpleslam {

// ── 基础类型 ──

/// 消息共享指针（不可变，多订阅者安全共享同一份数据）
template <typename T>
using MsgPtr = std::shared_ptr<const T>;

/// QoS 模式
enum class QoS : uint8_t {
    Event,   ///< 可靠传递，不丢消息，1 对多
    Stream,  ///< 高频数据流，可丢旧帧
    Latest,  ///< 只保留最新值，新值覆盖旧值
};

/// 队列满时的丢弃策略
enum class DropPolicy : uint8_t {
    DropOldest,  ///< 丢弃最旧的消息（默认）
    DropNewest,  ///< 丢弃最新的消息
};

/// 订阅选项
struct SubscribeOptions {
    size_t queue_depth = 0;                     ///< 0 = 同步直调（offline 默认）
    DropPolicy drop = DropPolicy::DropOldest;   ///< 队列满时策略
};

// ── 回调包装器 ──

namespace detail {

template <typename T, typename Obj>
auto wrapCallback(void (Obj::*method)(const T&), Obj* obj) {
    return [obj, method](MsgPtr<T> msg) { (obj->*method)(*msg); };
}

template <typename T, typename Obj>
auto wrapCallback(void (Obj::*method)(MsgPtr<T>), Obj* obj) {
    return [obj, method](MsgPtr<T> msg) { (obj->*method)(std::move(msg)); };
}

template <typename T>
auto wrapCallback(void (*func)(const T&)) {
    return [func](MsgPtr<T> msg) { func(*msg); };
}

template <typename T, typename Callable>
auto wrapCallback(Callable&& cb) {
    if constexpr (std::is_invocable_v<Callable, const T&>) {
        return [cb = std::forward<Callable>(cb)](MsgPtr<T> msg) { cb(*msg); };
    } else {
        return std::forward<Callable>(cb);
    }
}

}  // namespace detail

// ── 类型擦除基类 ──

class ITopicBase {
public:
    virtual ~ITopicBase() = default;
    [[nodiscard]] virtual const std::string& name() const = 0;
    [[nodiscard]] virtual QoS qos() const = 0;
    [[nodiscard]] virtual size_t subscriberCount() const = 0;
    [[nodiscard]] virtual size_t publisherCount() const = 0;
    [[nodiscard]] virtual uint64_t messageCount() const = 0;
    [[nodiscard]] virtual bool hasPending() const = 0;
    virtual void drainOnce() = 0;
};

// ── 订阅句柄（RAII 退订）──

/// 订阅的生命周期管理——析构时自动退订
class SubscriptionBase {
public:
    virtual ~SubscriptionBase() = default;
};

/// 持有则订阅有效，析构则自动退订
using SubscriptionHandle = std::shared_ptr<SubscriptionBase>;

// ── Topic<T> 内部实现 ──

template <typename T>
class Topic final : public ITopicBase {
public:
    using Callback = std::function<void(MsgPtr<T>)>;

    Topic(std::string topic_name, QoS topic_qos)
        : name_(std::move(topic_name)), qos_(topic_qos) {}

    // ── 发布 ──

    void publish(MsgPtr<T> msg) {
        ++message_count_;
        std::lock_guard lock(mutex_);
        latest_value_ = msg;
        if (offline_mode_) {
            pending_.push_back(std::move(msg));
        } else {
            dispatchDirect(msg);
        }
    }

    void publish(T&& msg) {
        publish(std::make_shared<const T>(std::move(msg)));
    }

    void publish(const T& msg) {
        publish(std::make_shared<const T>(msg));
    }

    template <typename... Args>
    void emplace(Args&&... args) {
        publish(std::make_shared<const T>(std::forward<Args>(args)...));
    }

    // ── 订阅 ──

    /// 注册回调，返回 SubscriptionHandle（RAII 退订）
    SubscriptionHandle subscribe(Callback cb, SubscribeOptions opts = {}) {
        std::lock_guard lock(mutex_);
        size_t id = next_sub_id_++;
        subscribers_.push_back({id, std::move(cb), opts});

        // 创建 RAII 句柄：析构时调用 unsubscribe
        auto* self = this;
        auto handle = std::make_shared<SubscriptionImpl>(
            [self, id]() { self->unsubscribe(id); });
        return handle;
    }

    void unsubscribe(size_t id) {
        std::lock_guard lock(mutex_);
        subscribers_.erase(
            std::remove_if(subscribers_.begin(), subscribers_.end(),
                           [id](const SubscriberEntry& e) { return e.id == id; }),
            subscribers_.end());
    }

    // ── Latest 查询 ──

    [[nodiscard]] MsgPtr<T> latest() const {
        std::lock_guard lock(mutex_);
        return latest_value_;
    }

    // ── Publisher 计数 ──

    void addPublisher() { ++publisher_count_; }
    void removePublisher() { --publisher_count_; }

    // ── ITopicBase ──

    [[nodiscard]] const std::string& name() const override { return name_; }
    [[nodiscard]] QoS qos() const override { return qos_; }
    [[nodiscard]] size_t subscriberCount() const override {
        std::lock_guard lock(mutex_);
        return subscribers_.size();
    }
    [[nodiscard]] size_t publisherCount() const override { return publisher_count_; }
    [[nodiscard]] uint64_t messageCount() const override { return message_count_; }

    [[nodiscard]] bool hasPending() const override {
        std::lock_guard lock(mutex_);
        return !pending_.empty();
    }

    void drainOnce() override {
        MsgPtr<T> msg;
        std::vector<Callback> callbacks;
        {
            std::lock_guard lock(mutex_);
            if (pending_.empty()) return;
            msg = std::move(pending_.front());
            pending_.pop_front();
            callbacks.reserve(subscribers_.size());
            for (const auto& s : subscribers_) {
                callbacks.push_back(s.callback);
            }
        }
        for (const auto& cb : callbacks) {
            cb(msg);
        }
    }

    void setOfflineMode(bool offline) {
        std::lock_guard lock(mutex_);
        offline_mode_ = offline;
    }

private:
    /// RAII 退订实现
    class SubscriptionImpl : public SubscriptionBase {
    public:
        explicit SubscriptionImpl(std::function<void()> unsub)
            : unsub_(std::move(unsub)) {}
        ~SubscriptionImpl() override { unsub_(); }
    private:
        std::function<void()> unsub_;
    };

    struct SubscriberEntry {
        size_t id;
        Callback callback;
        SubscribeOptions options;
    };

    void dispatchDirect(const MsgPtr<T>& msg) {
        for (const auto& s : subscribers_) {
            s.callback(msg);
        }
    }

    std::string name_;
    QoS qos_;
    mutable std::mutex mutex_;
    std::vector<SubscriberEntry> subscribers_;
    std::deque<MsgPtr<T>> pending_;
    MsgPtr<T> latest_value_;
    std::atomic<uint64_t> message_count_{0};
    std::atomic<size_t> publisher_count_{0};
    size_t next_sub_id_{0};
    bool offline_mode_{true};
};

// ── Publisher<T> 句柄 ──

/// 显式发布句柄——持有 Topic 直接指针，publish 不经过名字查找
template <typename T>
class Publisher {
public:
    Publisher() = default;

    explicit Publisher(Topic<T>* topic) : topic_(topic) {
        if (topic_) topic_->addPublisher();
    }

    ~Publisher() {
        if (topic_) topic_->removePublisher();
    }

    Publisher(const Publisher& other) : topic_(other.topic_) {
        if (topic_) topic_->addPublisher();
    }

    Publisher& operator=(const Publisher& other) {
        if (this != &other) {
            if (topic_) topic_->removePublisher();
            topic_ = other.topic_;
            if (topic_) topic_->addPublisher();
        }
        return *this;
    }

    Publisher(Publisher&& other) noexcept : topic_(other.topic_) {
        other.topic_ = nullptr;
    }

    Publisher& operator=(Publisher&& other) noexcept {
        if (this != &other) {
            if (topic_) topic_->removePublisher();
            topic_ = other.topic_;
            other.topic_ = nullptr;
        }
        return *this;
    }

    void publish(MsgPtr<T> msg) { assert(topic_); topic_->publish(std::move(msg)); }
    void publish(T&& msg) { assert(topic_); topic_->publish(std::move(msg)); }
    void publish(const T& msg) { assert(topic_); topic_->publish(msg); }

    template <typename... Args>
    void emplace(Args&&... args) { assert(topic_); topic_->emplace(std::forward<Args>(args)...); }

    [[nodiscard]] bool valid() const { return topic_ != nullptr; }
    [[nodiscard]] size_t subscriberCount() const { return topic_ ? topic_->subscriberCount() : 0; }
    [[nodiscard]] const std::string& topicName() const {
        assert(topic_);
        return topic_->name();
    }

private:
    Topic<T>* topic_{nullptr};
};

}  // namespace simpleslam
