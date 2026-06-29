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
#include <any>
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
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <unordered_map>
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

    AsyncSubscriber(Callback cb, SubscribeOptions opts, std::string topic, QoS qos)
        : cb_(std::move(cb)),
          opts_(opts),
          topic_(std::move(topic)),
          qos_(qos),
          inbox_(opts.queue_depth ? opts.queue_depth : kDefaultDepth, opts.drop) {
        worker_ = std::thread([this] { loop(); });
    }

    ~AsyncSubscriber() { stop(); }

    AsyncSubscriber(const AsyncSubscriber&) = delete;
    AsyncSubscriber& operator=(const AsyncSubscriber&) = delete;

    /// 发布侧：非阻塞入队 + 唤醒 worker
    void post(MsgPtr<T> msg) {
        if (inbox_.push(std::move(msg)) && qos_ == QoS::Event) {
            // QoS::Event 承诺可靠传递：async Inbox 满导致的丢弃必须上报（法则4，不静默吞）。
            // Stream/Latest 本就允许丢旧帧，不报。
            subscriberErrorHandler()(topic_, "async Event inbox 满，消息被丢弃");
        }
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
    QoS qos_;
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
        std::vector<Callback> sync_cbs;
        {
            std::lock_guard lock(mutex_);
            latest_value_ = msg;
            if (offline_mode_) {
                if (qos_ == QoS::Latest) {
                    latest_dirty_ = true;  // 合并：只标脏，不逐条入队（蓝图 §7.2.4 不重放）
                } else {
                    pending_.push_back(std::move(msg));
                }
                return;  // 离线：留待 drainOnce 投递
            }
            // 在线：async 投递非阻塞（仅入队+唤醒），留锁内安全（防并发 unsubscribe
            // 销毁 worker 致 UAF）；同步回调仅快照、移锁外执行——不变量②（回调不持内部
            // 锁），否则回调内向本话题 publish/latest 会自死锁非递归 mutex_。
            for (auto& s : subscribers_) {
                if (s.async) {
                    s.async->post(msg);
                } else if (s.shouldDeliver()) {
                    sync_cbs.push_back(s.callback);
                }
            }
        }
        for (const auto& cb : sync_cbs) {
            detail::invokeIsolated([&] { cb(msg); }, name_);
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
                SubscribeOptions aopts = opts;
                if (qos_ == QoS::Latest) {
                    aopts.queue_depth = 1;  // 在线 Latest 合并：Inbox 只留最新一条
                }
                entry.async = std::make_unique<AsyncSubscriber<T>>(
                    entry.callback, aopts, name_, qos_);
            }
            // latching：QoS::Latest 话题对迟到订阅者补发当前最后值
            // 仅当不脏时——若脏，即将到来的 drain 会投给它，避免重复
            if (qos_ == QoS::Latest && latest_value_ && !latest_dirty_) {
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
        return !pending_.empty() || latest_dirty_;
    }

    void drainOnce() override {
        MsgPtr<T> msg;
        std::vector<Callback> callbacks;
        {
            std::lock_guard lock(mutex_);
            if (qos_ == QoS::Latest && latest_dirty_) {
                msg = latest_value_;      // 只投最新值一次（合并中间帧）
                latest_dirty_ = false;
            } else if (!pending_.empty()) {
                msg = std::move(pending_.front());
                pending_.pop_front();
            } else {
                return;
            }
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

    /// 在尚无消息流动时采纳新 QoS——用于发布者纠正"订阅者首触按默认 Event 建话题"。
    /// 一旦已有消息/pending/脏值则拒绝改动（语义已生效，不可中途切换）。
    void adoptQoS(QoS q) {
        std::lock_guard lock(mutex_);
        if (message_count_ == 0 && pending_.empty() && !latest_dirty_) {
            qos_ = q;
        }
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

    std::string name_;
    QoS qos_;
    mutable std::mutex mutex_;
    std::vector<SubscriberEntry> subscribers_;
    std::deque<MsgPtr<T>> pending_;
    MsgPtr<T> latest_value_;
    bool latest_dirty_{false};  ///< QoS::Latest：有未投递的新值（合并标志）
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

// ── TopicHub：命名话题注册表 + BFS 协调分发 ──

/// 命名话题注册表。**懒加载全局单例**（免 init 直接用），也可建隔离实例供测试。
class TopicHub {
public:
    // ── 全局单例（懒加载：首次访问自动创建，无需先 init）──

    /// 显式设置全局实例模式（覆盖懒加载默认）。一般不需要——直接用即可。
    static void init(bool offline_mode = true) {
        std::lock_guard<std::mutex> lock(global_mutex_);
        default_offline_mode_ = offline_mode;
        global_instance_ = std::make_unique<TopicHub>(offline_mode);
    }
    static void shutdown() {
        std::lock_guard<std::mutex> lock(global_mutex_);
        global_instance_.reset();
    }
    /// 全局实例：未创建则按 default_offline_mode_ 懒构造（免 init）。
    static TopicHub& instance() {
        std::lock_guard<std::mutex> lock(global_mutex_);
        if (!global_instance_) {
            global_instance_ = std::make_unique<TopicHub>(default_offline_mode_);
        }
        return *global_instance_;
    }

    // ── 静态 Publisher 创建（用默认 hub）──

    template <typename T>
    static Publisher<T> createPublisher(std::string_view name,
                                       QoS qos = QoS::Event) {
        return instance().createPublisherImpl<T>(name, qos);
    }

    // ── 静态 Subscriber 创建：成员函数 ──

    template <typename T, typename Obj, typename Method>
    static SubscriptionHandle createSubscriber(std::string_view name,
                                               Method method, Obj* obj,
                                               SubscribeOptions opts = {}) {
        auto cb = detail::wrapCallback<T>(method, obj);
        return instance().subscribeImpl<T>(name, std::move(cb), opts);
    }

    // ── 静态 Subscriber 创建：lambda / 自由函数 / std::function ──

    template <typename T, typename Callable>
    static SubscriptionHandle createSubscriber(std::string_view name,
                                               Callable&& callback,
                                               SubscribeOptions opts = {}) {
        auto cb = detail::wrapCallback<T>(std::forward<Callable>(callback));
        return instance().subscribeImpl<T>(name, std::move(cb), opts);
    }

    // ── 最新值查询 ──

    template <typename T>
    static MsgPtr<T> getLatest(std::string_view name) {
        return instance().getLatestImpl<T>(name);
    }

    // ── Offline BFS 协调 ──

    /// 逐轮分发 pending 消息，回调中新 publish 进入下一轮。
    ///
    /// 关键：每轮在锁内快照"有 pending 的话题指针"，【释放 hub 锁后】再 drainOnce——
    /// 回调可安全重入 hub（getLatest/subscribe/createPublisher/listTopics），否则持
    /// 非递归 mutex_ 跑回调会自死锁。话题条目从不擦除，故快照指针在锁外仍有效。
    /// kMaxRounds 是回调发布环的无限循环兜底上限（蓝图 §7.2.4 BFS 深度约束）。
    size_t drainAll() {
        static constexpr int kMaxRounds = 1000;
        size_t total = 0;
        for (int round = 0; round < kMaxRounds; ++round) {
            std::vector<ITopicBase*> ready;
            {
                std::lock_guard lock(mutex_);
                for (auto& [_, entry] : topics_) {
                    if (entry.base->hasPending()) ready.push_back(entry.base.get());
                }
            }
            if (ready.empty()) return total;  // 到达不动点
            for (auto* t : ready) {
                t->drainOnce();
                ++total;
            }
        }
        std::fprintf(stderr,
                     "[SimpleSLAM] drainAll 超过最大轮数 %d——疑似回调发布环，已中断\n",
                     kMaxRounds);
        return total;
    }

    // ── 自省 ──

    static std::vector<std::string> listTopics() {
        auto& hub = instance();
        std::lock_guard lock(hub.mutex_);
        std::vector<std::string> names;
        names.reserve(hub.topics_.size());
        for (const auto& [name, _] : hub.topics_) {
            names.push_back(name);
        }
        return names;
    }

    struct TopicStats {
        std::string name;
        size_t publisher_count;
        size_t subscriber_count;
        uint64_t message_count;
    };
    static TopicStats stats(std::string_view name) {
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

    // ── 连通性检查（抓名字错配：只发无订 / 只订无发）──

    struct DanglingTopic {
        std::string name;
        size_t publisher_count;
        size_t subscriber_count;
    };

    /// 返回悬空话题（恰好一端缺）—— 多半是发布/订阅名字打错。
    std::vector<DanglingTopic> findDanglingTopics() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<DanglingTopic> out;
        for (const auto& [name, entry] : topics_) {
            const size_t p = entry.base->publisherCount();
            const size_t s = entry.base->subscriberCount();
            if ((p == 0) != (s == 0)) {  // 恰好一端为 0
                out.push_back({name, p, s});
            }
        }
        return out;
    }

    /// 全局实例连通性检查 + 写警告（Runner 接线后调一次）。返回悬空话题数。
    static size_t checkWiring() {
        auto dangling = instance().findDanglingTopics();
        for (const auto& d : dangling) {
            std::fprintf(
                stderr,
                "[SimpleSLAM] 话题 '%s' 悬空（pub=%zu sub=%zu）—— 疑似名字错配\n",
                d.name.c_str(), d.publisher_count, d.subscriber_count);
        }
        return dangling.size();
    }

    // ── 构造函数（隔离实例，测试用）──

    explicit TopicHub(bool offline_mode) : offline_mode_(offline_mode) {}
    ~TopicHub() = default;

    TopicHub(const TopicHub&) = delete;
    TopicHub& operator=(const TopicHub&) = delete;

    // ── 实例方法（测试/隔离用）──

    template <typename T>
    Publisher<T> createPublisherImpl(std::string_view name,
                                    QoS qos = QoS::Event) {
        auto* topic = getOrCreateTopic<T>(name, qos, /*qos_explicit=*/true);
        return Publisher<T>(topic);
    }

    /// 订阅：lambda / 自由函数 / std::function
    template <typename T, typename Callable>
    SubscriptionHandle subscribeImpl(std::string_view name,
                                     Callable&& cb,
                                     SubscribeOptions opts = {}) {
        auto wrapped = detail::wrapCallback<T>(std::forward<Callable>(cb));
        auto* topic = getOrCreateTopic<T>(name, QoS::Event, /*qos_explicit=*/false);
        return topic->subscribe(std::move(wrapped), opts);
    }

    /// 订阅：成员函数 void(Obj::*)(const T&)——自动推导 T
    template <typename T, typename Obj>
    SubscriptionHandle subscribeImpl(std::string_view name,
                                     void (Obj::*method)(const T&), Obj* obj,
                                     SubscribeOptions opts = {}) {
        auto cb = detail::wrapCallback<T>(method, obj);
        auto* topic = getOrCreateTopic<T>(name, QoS::Event, /*qos_explicit=*/false);
        return topic->subscribe(std::move(cb), opts);
    }

    /// 订阅：成员函数 void(Obj::*)(MsgPtr<T>)——自动推导 T
    template <typename T, typename Obj>
    SubscriptionHandle subscribeImpl(std::string_view name,
                                     void (Obj::*method)(MsgPtr<T>), Obj* obj,
                                     SubscribeOptions opts = {}) {
        auto cb = detail::wrapCallback<T>(method, obj);
        auto* topic = getOrCreateTopic<T>(name, QoS::Event, /*qos_explicit=*/false);
        return topic->subscribe(std::move(cb), opts);
    }

    template <typename T>
    MsgPtr<T> getLatestImpl(std::string_view name) {
        std::lock_guard lock(mutex_);
        std::string key(name);
        auto it = topics_.find(key);
        if (it == topics_.end()) return nullptr;
        auto* typed = std::any_cast<Topic<T>*>(&it->second.typed);
        if (!typed) return nullptr;
        return (*typed)->latest();
    }

    [[nodiscard]] bool hasTopic(std::string_view name) const {
        std::lock_guard lock(mutex_);
        return topics_.count(std::string(name)) > 0;
    }

private:
    struct TopicEntry {
        std::unique_ptr<ITopicBase> base;
        std::any typed;             ///< Topic<T>*
        bool qos_explicit = false;  ///< QoS 是否由发布者显式声明（订阅者默认不算）
    };

    /// @param qos_explicit 调用方是否"明确"该 QoS（发布者=是，订阅者默认 Event=否）。
    ///   防 QoS 首触者降级：订阅者先按默认 Event 建话题后，发布者显式 QoS 可纠正之；
    ///   两处显式声明冲突则抛错（与类型不匹配同样响亮）。
    template <typename T>
    Topic<T>* getOrCreateTopic(std::string_view name, QoS qos, bool qos_explicit) {
        std::lock_guard lock(mutex_);
        std::string key(name);
        auto it = topics_.find(key);
        if (it != topics_.end()) {
            auto* ptr = std::any_cast<Topic<T>*>(&it->second.typed);
            if (!ptr) {
                throw std::runtime_error(
                    "TopicHub: type mismatch for topic: " + key);
            }
            if (qos_explicit) {
                if (it->second.qos_explicit && (*ptr)->qos() != qos) {
                    throw std::runtime_error(
                        "TopicHub: QoS 冲突 for topic '" + key +
                        "'（两处显式声明了不同 QoS）");
                }
                if (!it->second.qos_explicit) {
                    (*ptr)->adoptQoS(qos);  // 采纳发布者意图，纠正订阅者首触的默认 Event
                }
                it->second.qos_explicit = true;
            }
            return *ptr;
        }
        auto topic = std::make_unique<Topic<T>>(key, qos);
        topic->setOfflineMode(offline_mode_);
        auto* raw = topic.get();
        TopicEntry entry;
        entry.base = std::move(topic);
        entry.typed = raw;
        entry.qos_explicit = qos_explicit;
        topics_[key] = std::move(entry);
        return raw;
    }

    mutable std::mutex mutex_;
    std::unordered_map<std::string, TopicEntry> topics_;
    bool offline_mode_{true};

    inline static std::unique_ptr<TopicHub> global_instance_;
    inline static std::mutex global_mutex_;
    inline static bool default_offline_mode_{true};
};

}  // namespace simpleslam
