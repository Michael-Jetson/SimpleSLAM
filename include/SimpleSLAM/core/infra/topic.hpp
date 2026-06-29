#pragma once

/// @file topic.hpp
/// Topic<T> 进程内发布-订阅通信——Publisher/Subscriber 句柄模式
///
/// 设计原则：消息只读共享（shared_ptr<const T>），状态各自私有。
/// 类 ROS 2 话题 API，但更轻量：无 DDS、无序列化，publish 延迟 <100ns。
///
/// 三种 QoS：Event（可靠多播）、Stream（高频有损）、Latest（只保留最新）
/// 四种回调绑定：成员函数(const T&)、成员函数(MsgPtr)、自由函数、lambda

#include <algorithm>
#include <atomic>
#include <cassert>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
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

/// 投递模式（每订阅者）
enum class DeliveryMode : uint8_t {
    Sync,   ///< 同步内联：离线走 drain，在线走发布线程直调
    Async,  ///< 异步：每订阅者独立 worker 线程 + Inbox（仅在线模式生效）
};

/// 订阅选项
struct SubscribeOptions {
    size_t queue_depth = 0;                       ///< Inbox 容量（async）；0 取默认
    DropPolicy drop = DropPolicy::DropOldest;     ///< 队列满时策略
    int throttle_every = 1;                       ///< skip-frame：每 N 条处理 1（1=不跳），偷自高翔
    DeliveryMode delivery = DeliveryMode::Sync;   ///< 同步/异步投递
};

/// 订阅者异常处理钩子——默认写 stderr，可替换以路由到项目日志系统。
/// 进程内总线对回调异常做每回调隔离（蓝图法则 4）：错误经此上报，绝不静默吞掉。
using SubscriberErrorHandler =
    std::function<void(const std::string& topic, const char* what)>;

inline SubscriberErrorHandler& subscriberErrorHandler() {
    static SubscriberErrorHandler handler =
        [](const std::string& topic, const char* what) {
            std::fprintf(stderr,
                         "[SimpleSLAM] topic '%s' subscriber threw: %s\n",
                         topic.c_str(), what);
        };
    return handler;
}

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

/// 调用回调并隔离异常：单个订阅者抛出不波及兄弟订阅者、不外泄出分发循环。
template <typename Fn>
inline void invokeIsolated(Fn&& fn, const std::string& topic) {
    try {
        std::forward<Fn>(fn)();
    } catch (const std::exception& e) {
        subscriberErrorHandler()(topic, e.what());
    } catch (...) {
        subscriberErrorHandler()(topic, "unknown exception");
    }
}

}  // namespace detail

// ── Inbox<T>：有界消息队列（async worker 的缓冲，内核可插拔点）──

/// 线程安全有界队列。满时按 DropPolicy 丢弃，push 永不阻塞（非阻塞发布的关键）。
template <typename T>
class Inbox {
public:
    Inbox(size_t capacity, DropPolicy drop)
        : cap_(capacity ? capacity : 1), drop_(drop) {}

    /// 入队。满了按策略丢弃。返回是否发生了丢弃。
    bool push(MsgPtr<T> msg) {
        std::lock_guard<std::mutex> lk(m_);
        if (q_.size() >= cap_) {
            if (drop_ == DropPolicy::DropNewest) {
                return true;  // 拒绝新来的
            }
            q_.pop_front();   // DropOldest：丢最旧
            q_.push_back(std::move(msg));
            return true;
        }
        q_.push_back(std::move(msg));
        return false;
    }

    /// 批量抽走全部消息（消费侧批处理）。返回抽取条数。
    size_t drainBatch(std::vector<MsgPtr<T>>& out) {
        std::lock_guard<std::mutex> lk(m_);
        const size_t n = q_.size();
        out.reserve(out.size() + n);
        while (!q_.empty()) {
            out.push_back(std::move(q_.front()));
            q_.pop_front();
        }
        return n;
    }

    [[nodiscard]] size_t size() const {
        std::lock_guard<std::mutex> lk(m_);
        return q_.size();
    }

private:
    mutable std::mutex m_;
    std::deque<MsgPtr<T>> q_;
    size_t cap_;
    DropPolicy drop_;
};

// ── AsyncSubscriber<T>：每订阅者一个 worker 线程（照高翔 AsyncMessageProcess）──

/// 持有 Inbox + worker 线程：post() 非阻塞入队并唤醒；worker 批量抽取、串行回调、
/// 异常隔离、skip-frame 节流。析构时排空剩余并 join。
template <typename T>
class AsyncSubscriber {
public:
    using Callback = std::function<void(MsgPtr<T>)>;

    AsyncSubscriber(Callback cb, SubscribeOptions opts, std::string topic)
        : cb_(std::move(cb)),
          opts_(opts),
          topic_(std::move(topic)),
          inbox_(opts.queue_depth ? opts.queue_depth : kDefaultDepth, opts.drop) {
        worker_ = std::thread([this] { loop(); });
    }

    ~AsyncSubscriber() { stop(); }

    AsyncSubscriber(const AsyncSubscriber&) = delete;
    AsyncSubscriber& operator=(const AsyncSubscriber&) = delete;

    /// 发布侧：非阻塞入队 + 唤醒 worker
    void post(MsgPtr<T> msg) {
        inbox_.push(std::move(msg));
        {
            std::lock_guard<std::mutex> lk(wake_m_);
            updated_ = true;
        }
        cv_.notify_one();
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lk(wake_m_);
            exit_ = true;
        }
        cv_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

private:
    static constexpr size_t kDefaultDepth = 100;

    void loop() {
        std::vector<MsgPtr<T>> batch;
        for (;;) {
            bool should_exit;
            {
                std::unique_lock<std::mutex> lk(wake_m_);
                cv_.wait(lk, [this] { return updated_ || exit_; });
                updated_ = false;
                should_exit = exit_;
            }
            batch.clear();
            inbox_.drainBatch(batch);
            for (auto& msg : batch) {
                if (shouldDeliver())
                    detail::invokeIsolated([&] { cb_(msg); }, topic_);
            }
            if (should_exit && inbox_.size() == 0) return;  // 排空后退出
        }
    }

    bool shouldDeliver() {
        const int n = opts_.throttle_every;
        const bool fire = (n <= 1) || (seq_ % static_cast<size_t>(n) == 0);
        ++seq_;
        return fire;
    }

    Callback cb_;
    SubscribeOptions opts_;
    std::string topic_;
    Inbox<T> inbox_;
    std::thread worker_;
    std::mutex wake_m_;
    std::condition_variable cv_;
    bool updated_ = false;
    bool exit_ = false;
    size_t seq_ = 0;
};

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
        MsgPtr<T> backfill;
        Callback backfill_cb;
        size_t id;
        {
            std::lock_guard lock(mutex_);
            id = next_sub_id_++;
            subscribers_.push_back({id, std::move(cb), opts});
            SubscriberEntry& entry = subscribers_.back();
            // async 投递：在线模式下为该订阅者起一个 worker 线程
            // （离线模式保持单线程确定性，忽略 async）
            if (opts.delivery == DeliveryMode::Async && !offline_mode_) {
                entry.async = std::make_unique<AsyncSubscriber<T>>(
                    entry.callback, opts, name_);
            }
            // latching：QoS::Latest 话题对迟到订阅者补发当前最后值
            if (qos_ == QoS::Latest && latest_value_) {
                backfill = latest_value_;
                backfill_cb = entry.callback;
            }
        }

        // 创建 RAII 句柄：析构时调用 unsubscribe
        auto* self = this;
        auto handle = std::make_shared<SubscriptionImpl>(
            [self, id]() { self->unsubscribe(id); });

        // 补发在锁外执行（不变量②：回调不持内部锁），并做异常隔离
        if (backfill) {
            detail::invokeIsolated([&] { backfill_cb(backfill); }, name_);
        }
        return handle;
    }

    void unsubscribe(size_t id) {
        std::unique_ptr<AsyncSubscriber<T>> dying;  // worker 在锁外 join，避免死锁
        {
            std::lock_guard lock(mutex_);
            auto it = std::find_if(
                subscribers_.begin(), subscribers_.end(),
                [id](const SubscriberEntry& e) { return e.id == id; });
            if (it == subscribers_.end()) return;
            dying = std::move(it->async);
            subscribers_.erase(it);
        }
        // dying（含 worker join）在此处锁外析构：若回调内又向本话题 publish 也不会死锁
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
            for (auto& s : subscribers_) {
                if (s.shouldDeliver()) callbacks.push_back(s.callback);
            }
        }
        for (const auto& cb : callbacks) {
            detail::invokeIsolated([&] { cb(msg); }, name_);
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
        size_t seq = 0;  ///< 已收到的消息序号（用于 skip-frame 节流）

        /// 推进序号并返回本条是否应投递（每 throttle_every 条处理 1 条）
        bool shouldDeliver() {
            const int n = options.throttle_every;
            const bool fire = (n <= 1) || (seq % static_cast<size_t>(n) == 0);
            ++seq;
            return fire;
        }

        std::unique_ptr<AsyncSubscriber<T>> async{};  ///< 非空 ⟺ async 投递（在线模式）
    };

    void dispatchDirect(const MsgPtr<T>& msg) {
        for (auto& s : subscribers_) {
            if (s.async) {
                s.async->post(msg);  // 非阻塞入队，worker 异步处理（自带节流/隔离）
            } else if (s.shouldDeliver()) {
                detail::invokeIsolated([&] { s.callback(msg); }, name_);
            }
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
