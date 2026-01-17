#pragma once

#include <eventpp/eventdispatcher.h>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace SimpleSLAM {

// =========================
// Internal minimal thread pool
// =========================
class _TBThreadPool {
public:
  explicit _TBThreadPool(std::size_t n) : stop_(false) {
    if (n == 0) n = 1;
    workers_.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      workers_.emplace_back([this] { workerLoop(); });
    }
  }

  ~_TBThreadPool() { shutdown(); }

  _TBThreadPool(const _TBThreadPool&) = delete;
  _TBThreadPool& operator=(const _TBThreadPool&) = delete;

  void shutdown() {
    {
      std::lock_guard<std::mutex> lk(m_);
      if (stop_) return;
      stop_ = true;
    }
    cv_.notify_all();
    for (auto &t : workers_) {
      if (t.joinable()) t.join();
    }
    workers_.clear();
  }

  template <class F>
  void post(F&& f) {
    {
      std::lock_guard<std::mutex> lk(m_);
      if (stop_) return;
      tasks_.emplace_back(std::forward<F>(f));
    }
    cv_.notify_one();
  }

private:
  void workerLoop() {
    for (;;) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&] { return stop_ || !tasks_.empty(); });
        if (stop_ && tasks_.empty()) return;
        task = std::move(tasks_.front());
        tasks_.pop_front();
      }
      task();
    }
  }

  std::mutex m_;
  std::condition_variable cv_;
  std::deque<std::function<void()>> tasks_;
  bool stop_;
  std::vector<std::thread> workers_;
};

// =========================
// Internal envelope
// =========================
struct _TBEnvelope {
  std::string topic;
  std::type_index type{typeid(void)};
  std::shared_ptr<const void> payload;
  std::uint64_t seq{0};
};

class _TBRuntime;

// =========================
// TopicBus (public)
// =========================
class TopicBus {
public:
  // ---------- Init ----------
  struct InitOptions {
    std::size_t callback_threads = std::max<std::size_t>(1, std::thread::hardware_concurrency());
  };

  // ---------- Named-arg style option tags ----------
  // 默认值（你不传这些选项时使用）：
  //   async_callback        = true
  //   max_pending_callbacks = 2
  struct _SubParsed {
    bool async_callback = true;
    std::size_t max_pending_callbacks = 2;
  };

  struct _OptAsync { bool value; };
  struct _OptMaxPending { std::size_t value; };

  // 让你可以写：TopicBus::async_callback = false
  struct AsyncKey {
    _OptAsync operator=(bool v) noexcept { return _OptAsync{v}; }
  };
  // 让你可以写：TopicBus::max_pending_callbacks = 4
  struct MaxPendingKey {
    _OptMaxPending operator=(std::size_t v) noexcept { return _OptMaxPending{v}; }
  };

  // 这两个“键对象”就是你写 max_pending_callbacks = 4 的来源
  inline static thread_local AsyncKey async_callback{};
  inline static thread_local MaxPendingKey max_pending_callbacks{};

  // ---------- Handles ----------
  class Subscriber {
  public:
    Subscriber() = default;
    ~Subscriber() { reset(); }

    Subscriber(const Subscriber&) = delete;
    Subscriber& operator=(const Subscriber&) = delete;

    Subscriber(Subscriber&& other) noexcept { *this = std::move(other); }
    Subscriber& operator=(Subscriber&& other) noexcept {
      if (this == &other) return *this;
      reset();
      rt_ = std::move(other.rt_);
      topic_ = std::move(other.topic_);
      handle_ = other.handle_;
      valid_ = other.valid_;
      other.valid_ = false;
      return *this;
    }

    void reset();
    explicit operator bool() const noexcept { return valid_; }

  private:
    friend class _TBRuntime;
    using Handle = eventpp::EventDispatcher<std::string, void(const _TBEnvelope&)>::Handle;

    Subscriber(std::weak_ptr<_TBRuntime> rt, std::string topic, Handle h)
        : rt_(std::move(rt)), topic_(std::move(topic)), handle_(h), valid_(true) {}

    std::weak_ptr<_TBRuntime> rt_;
    std::string topic_;
    Handle handle_{};
    bool valid_{false};
  };

  template <class T>
  class Publisher {
  public:
    Publisher() = default;

    // 非阻塞：topic 队列满则返回 false
    bool Publish(std::shared_ptr<const T> msg) const;

    bool Publish(std::shared_ptr<T> msg) const {
      return Publish(std::shared_ptr<const T>(std::move(msg)));
    }
    bool Publish(const T& msg) const {
      return Publish(std::make_shared<T>(msg));
    }
    bool Publish(T&& msg) const {
      return Publish(std::make_shared<T>(std::move(msg)));
    }

    explicit operator bool() const noexcept { return !rt_.expired(); }

  private:
    friend class _TBRuntime;
    Publisher(std::weak_ptr<_TBRuntime> rt, std::string topic)
        : rt_(std::move(rt)), topic_(std::move(topic)) {}

    std::weak_ptr<_TBRuntime> rt_;
    std::string topic_;
  };

public:
  // ---------- Lifecycle ----------
  static void Init(const InitOptions& opt = {});
  static void Shutdown();
  static bool IsInitialized();

  // ---------- Advertise ----------
  template <class T>
  static Publisher<T> Advertise(const std::string& topic,
                                std::size_t topic_queue_limit = 200);

  // ============================================================
  // Subscribe (shared_ptr<const T> callback)
  // 你要的形式：Subscribe<T>("/imu", 200, this, &C::OnImu, max_pending_callbacks = 4)
  // 也支持：Subscribe<T>("/imu", 200, free_func/lambda, ...)
  // ============================================================

  // member: void C::OnMsg(std::shared_ptr<const T>)
  template <class T, class C, class... Opts>
  static Subscriber Subscribe(const std::string& topic,
                              std::size_t topic_queue_limit,
                              C* obj,
                              void (C::*mf)(std::shared_ptr<const T>),
                              Opts... opts);

  // member const: void C::OnMsg(std::shared_ptr<const T>) const
  template <class T, class C, class... Opts>
  static Subscriber Subscribe(const std::string& topic,
                              std::size_t topic_queue_limit,
                              const C* obj,
                              void (C::*mf)(std::shared_ptr<const T>) const,
                              Opts... opts);

  // free/lambda: cb(std::shared_ptr<const T>)
  template <class T, class F, class... Opts>
  static Subscriber Subscribe(const std::string& topic,
                              std::size_t topic_queue_limit,
                              F&& cb,
                              Opts... opts);

  // ============================================================
  // SubscribeRef (const T& callback)
  // 仍然是异步安全：内部持有 shared_ptr，在任务执行时解引用
  // ============================================================

  // member: void C::OnMsg(const T&)
  template <class T, class C, class... Opts>
  static Subscriber SubscribeRef(const std::string& topic,
                                 std::size_t topic_queue_limit,
                                 C* obj,
                                 void (C::*mf)(const T&),
                                 Opts... opts);

  // member const: void C::OnMsg(const T&) const
  template <class T, class C, class... Opts>
  static Subscriber SubscribeRef(const std::string& topic,
                                 std::size_t topic_queue_limit,
                                 const C* obj,
                                 void (C::*mf)(const T&) const,
                                 Opts... opts);

  // free/lambda: cb(const T&)
  template <class T, class F, class... Opts>
  static Subscriber SubscribeRef(const std::string& topic,
                                 std::size_t topic_queue_limit,
                                 F&& cb,
                                 Opts... opts);

private:
  // ---------- Option parsing ----------
  template <class X> struct _False : std::false_type {};

  static void _ApplyOpt(_SubParsed& o, _OptAsync x) { o.async_callback = x.value; }
  static void _ApplyOpt(_SubParsed& o, _OptMaxPending x) { o.max_pending_callbacks = x.value; }

  template <class X>
  static void _ApplyOpt(_SubParsed&, X) {
    static_assert(_False<X>::value, "Unknown option passed to TopicBus::Subscribe/SubscribeRef");
  }

  template <class... Opts>
  static _SubParsed _Parse(Opts... opts) {
    _SubParsed o{};
    (_ApplyOpt(o, opts), ...);
    return o;
  }

  static std::shared_ptr<_TBRuntime> _GetOrCreate();
};

// =========================
// Runtime (internal)
// =========================
class _TBRuntime : public std::enable_shared_from_this<_TBRuntime> {
public:
  explicit _TBRuntime(const TopicBus::InitOptions& opt)
      : pool_(opt.callback_threads),
        stopped_(false) {
    pump_ = std::thread([this] { pumpLoop(); });
  }

  ~_TBRuntime() { shutdown(); }

  void shutdown() {
    bool expected = false;
    if (!stopped_.compare_exchange_strong(expected, true)) return;
    q_cv_.notify_all();
    if (pump_.joinable()) pump_.join();
    pool_.shutdown();
  }

  template <class T>
  TopicBus::Publisher<T> advertise(const std::string& topic, std::size_t queue_limit) {
    ensureTopicType(topic, typeid(T), queue_limit);
    return TopicBus::Publisher<T>(shared_from_this(), topic);
  }

  bool publishRaw(const std::string& topic, std::type_index type, std::shared_ptr<const void> payload) {
    {
      std::lock_guard<std::mutex> lk(topic_m_);
      auto it = topics_.find(topic);
      if (it == topics_.end()) {
        topics_.emplace(topic, TopicInfo{type, /*queue_limit*/0, /*queued*/0});
        it = topics_.find(topic);
      }
      if (it->second.type != type) {
        throw std::runtime_error("Topic type mismatch on '" + topic + "'");
      }
      if (it->second.queue_limit > 0 && it->second.queued >= it->second.queue_limit) {
        return false; // DropNewest, never block publisher
      }
      it->second.queued += 1;
    }

    {
      std::lock_guard<std::mutex> lk(q_m_);
      q_.push_back(_TBEnvelope{topic, type, std::move(payload), ++seq_});
    }
    q_cv_.notify_one();
    return true;
  }

  TopicBus::Subscriber subscribeRaw(const std::string& topic,
                                   std::type_index type,
                                   std::size_t topic_queue_limit,
                                   std::function<void(const _TBEnvelope&)> listener) {
    ensureTopicType(topic, type, topic_queue_limit);

    TopicBus::Subscriber::Handle h;
    {
      std::unique_lock<std::shared_mutex> lk(dispatch_mu_);
      h = dispatcher_.appendListener(topic, std::move(listener));
    }
    return TopicBus::Subscriber(shared_from_this(), topic, h);
  }

  void unsubscribe(const std::string& topic, TopicBus::Subscriber::Handle h) {
    std::unique_lock<std::shared_mutex> lk(dispatch_mu_);
    dispatcher_.removeListener(topic, h);
  }

  _TBThreadPool& pool() { return pool_; }

private:
  struct TopicInfo {
    std::type_index type{typeid(void)};
    std::size_t queue_limit{0}; // 0=unlimited
    std::size_t queued{0};
  };

  void ensureTopicType(const std::string& topic, std::type_index type, std::size_t queue_limit) {
    std::lock_guard<std::mutex> lk(topic_m_);
    auto it = topics_.find(topic);
    if (it == topics_.end()) {
      topics_.emplace(topic, TopicInfo{type, queue_limit, 0});
      return;
    }
    if (it->second.type != type) {
      throw std::runtime_error("Topic type mismatch on '" + topic + "'");
    }
    if (queue_limit > it->second.queue_limit) {
      it->second.queue_limit = queue_limit;
    }
  }

  void pumpLoop() {
    while (!stopped_.load(std::memory_order_relaxed)) {
      _TBEnvelope env;
      {
        std::unique_lock<std::mutex> lk(q_m_);
        q_cv_.wait(lk, [&] {
          return stopped_.load(std::memory_order_relaxed) || !q_.empty();
        });
        if (stopped_.load(std::memory_order_relaxed)) break;
        env = std::move(q_.front());
        q_.pop_front();
      }

      {
        std::lock_guard<std::mutex> lk(topic_m_);
        auto it = topics_.find(env.topic);
        if (it != topics_.end() && it->second.queued > 0) {
          it->second.queued -= 1;
        }
      }

      {
        std::shared_lock<std::shared_mutex> lk(dispatch_mu_);
        dispatcher_.dispatch(env.topic, env);
      }
    }
  }

  std::mutex topic_m_;
  std::unordered_map<std::string, TopicInfo> topics_;

  mutable std::shared_mutex dispatch_mu_;
  eventpp::EventDispatcher<std::string, void(const _TBEnvelope&)> dispatcher_;

  std::mutex q_m_;
  std::condition_variable q_cv_;
  std::deque<_TBEnvelope> q_;
  std::thread pump_;
  std::atomic<bool> stopped_;
  std::atomic<std::uint64_t> seq_{0};

  _TBThreadPool pool_;
};

// ===== Subscriber impl =====
inline void TopicBus::Subscriber::reset() {
  if (!valid_) return;
  valid_ = false;
  if (auto rt = rt_.lock()) {
    rt->unsubscribe(topic_, handle_);
  }
  topic_.clear();
}

// ===== Publisher impl =====
template <class T>
inline bool TopicBus::Publisher<T>::Publish(std::shared_ptr<const T> msg) const {
  auto rt = rt_.lock();
  if (!rt) return false;
  return rt->publishRaw(topic_, std::type_index(typeid(T)),
                        std::static_pointer_cast<const void>(std::move(msg)));
}

// ===== Global runtime storage =====
inline std::mutex& _TBGlobalMutex() {
  static std::mutex m;
  return m;
}
inline std::shared_ptr<_TBRuntime>& _TBGlobalRuntime() {
  static std::shared_ptr<_TBRuntime> rt;
  return rt;
}

inline void TopicBus::Init(const InitOptions& opt) {
  std::lock_guard<std::mutex> lk(_TBGlobalMutex());
  if (_TBGlobalRuntime()) return;
  _TBGlobalRuntime() = std::make_shared<_TBRuntime>(opt);
}

inline bool TopicBus::IsInitialized() {
  std::lock_guard<std::mutex> lk(_TBGlobalMutex());
  return static_cast<bool>(_TBGlobalRuntime());
}

inline void TopicBus::Shutdown() {
  std::shared_ptr<_TBRuntime> rt;
  {
    std::lock_guard<std::mutex> lk(_TBGlobalMutex());
    rt = _TBGlobalRuntime();
    _TBGlobalRuntime().reset();
  }
  if (rt) rt->shutdown();
}

inline std::shared_ptr<_TBRuntime> TopicBus::_GetOrCreate() {
  std::lock_guard<std::mutex> lk(_TBGlobalMutex());
  if (!_TBGlobalRuntime()) {
    _TBGlobalRuntime() = std::make_shared<_TBRuntime>(InitOptions{});
  }
  return _TBGlobalRuntime();
}

// ===== Advertise =====
template <class T>
inline typename TopicBus::template Publisher<T>
TopicBus::Advertise(const std::string& topic, std::size_t topic_queue_limit) {
  return _GetOrCreate()->advertise<T>(topic, topic_queue_limit);
}

// ===== Subscribe free/lambda (shared_ptr<const T>) =====
inline std::shared_ptr<std::atomic<std::size_t>> _TBMakePendingCounter() {
  return std::make_shared<std::atomic<std::size_t>>(0);
}

template <class T, class F, class... Opts>
inline TopicBus::Subscriber TopicBus::Subscribe(const std::string& topic,
                                                std::size_t topic_queue_limit,
                                                F&& cb,
                                                Opts... opts) {
  static_assert(std::is_invocable_v<std::decay_t<F>&, std::shared_ptr<const T>>,
                "Subscribe<T>: callback must be callable as cb(std::shared_ptr<const T>)");

  auto rt = _GetOrCreate();
  std::weak_ptr<_TBRuntime> w_rt = rt;
  auto o = _Parse(opts...);

  using CB = std::decay_t<F>;
  auto cb_ptr = std::make_shared<CB>(std::forward<F>(cb));
  auto pending = _TBMakePendingCounter();

  auto listener = [w_rt, cb_ptr, pending, o](const _TBEnvelope& env) mutable {
    if (env.type != std::type_index(typeid(T))) return;

    // hold message strongly for async safety
    auto msg = std::static_pointer_cast<const T>(env.payload);

    if (!o.async_callback) {
      (*cb_ptr)(msg);
      return;
    }

    // pending limit (non-blocking drop)
    if (o.max_pending_callbacks > 0) {
      auto old = pending->fetch_add(1, std::memory_order_relaxed);
      if (old >= o.max_pending_callbacks) {
        pending->fetch_sub(1, std::memory_order_relaxed);
        return;
      }
    } else {
      pending->fetch_add(1, std::memory_order_relaxed);
    }

    if (auto rt2 = w_rt.lock()) {
      rt2->pool().post([cb_ptr, msg, pending]() mutable {
        (*cb_ptr)(msg);
        pending->fetch_sub(1, std::memory_order_relaxed);
      });
    } else {
      pending->fetch_sub(1, std::memory_order_relaxed);
    }
  };

  return rt->subscribeRaw(topic, std::type_index(typeid(T)), topic_queue_limit, std::move(listener));
}

// ===== Subscribe member (shared_ptr<const T>) =====
template <class T, class C, class... Opts>
inline TopicBus::Subscriber TopicBus::Subscribe(const std::string& topic,
                                                std::size_t topic_queue_limit,
                                                C* obj,
                                                void (C::*mf)(std::shared_ptr<const T>),
                                                Opts... opts) {
  return Subscribe<T>(
      topic, topic_queue_limit,
      [obj, mf](std::shared_ptr<const T> m) { (obj->*mf)(std::move(m)); },
      opts...);
}

template <class T, class C, class... Opts>
inline TopicBus::Subscriber TopicBus::Subscribe(const std::string& topic,
                                                std::size_t topic_queue_limit,
                                                const C* obj,
                                                void (C::*mf)(std::shared_ptr<const T>) const,
                                                Opts... opts) {
  return Subscribe<T>(
      topic, topic_queue_limit,
      [obj, mf](std::shared_ptr<const T> m) { (obj->*mf)(std::move(m)); },
      opts...);
}

// ===== SubscribeRef free/lambda (const T&) =====
template <class T, class F, class... Opts>
inline TopicBus::Subscriber TopicBus::SubscribeRef(const std::string& topic,
                                                   std::size_t topic_queue_limit,
                                                   F&& cb,
                                                   Opts... opts) {
  static_assert(std::is_invocable_v<std::decay_t<F>&, const T&>,
                "SubscribeRef<T>: callback must be callable as cb(const T&)");

  // Still use shared_ptr under the hood; dereference inside execution => safe for async
  return Subscribe<T>(
      topic, topic_queue_limit,
      [fn = std::forward<F>(cb)](std::shared_ptr<const T> m) mutable {
        fn(*m);
      },
      opts...);
}

// ===== SubscribeRef member (const T&) =====
template <class T, class C, class... Opts>
inline TopicBus::Subscriber TopicBus::SubscribeRef(const std::string& topic,
                                                   std::size_t topic_queue_limit,
                                                   C* obj,
                                                   void (C::*mf)(const T&),
                                                   Opts... opts) {
  return SubscribeRef<T>(
      topic, topic_queue_limit,
      [obj, mf](const T& v) { (obj->*mf)(v); },
      opts...);
}

template <class T, class C, class... Opts>
inline TopicBus::Subscriber TopicBus::SubscribeRef(const std::string& topic,
                                                   std::size_t topic_queue_limit,
                                                   const C* obj,
                                                   void (C::*mf)(const T&) const,
                                                   Opts... opts) {
  return SubscribeRef<T>(
      topic, topic_queue_limit,
      [obj, mf](const T& v) { (obj->*mf)(v); },
      opts...);
}

} // namespace SimpleSLAM
