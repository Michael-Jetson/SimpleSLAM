// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file topic.hpp
/// 话题：一对多投递。实现类型无关，在 src/comm/src/topic.cpp 里；Publisher<T> 和模块的 subscribe 只是薄的类型层。
///
/// 容量只有一个整数 queue_size，语义同 roscpp：0 无界；N>0 满了丢最旧；1 就是只留最新。
/// publish 不返回结果，丢弃数记在订阅端（见 Runtime::stats）。

#include <SimpleSLAM/comm/mailbox.hpp>
#include <SimpleSLAM/comm/types.hpp>

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>

namespace simpleslam::comm {

struct SubscriptionStats {
    std::string topic;
    std::string subscriber;
    std::size_t queue_size = 0;
    std::size_t depth = 0;
    std::size_t high_water = 0;
    std::uint64_t delivered = 0;
    std::uint64_t dropped = 0;
};

/// 缓冲只管容量，顺序由收件箱决定：每接纳一条就排一格，丢最旧时不排新格，
/// 所以丢弃之后该订阅端的存活条目会继承更早的格位。
class Subscription final : public DeliveryTarget, public std::enable_shared_from_this<Subscription> {
public:
    using Callback = std::function<void(const AnyMsg&)>;

    /// backlog_warn 是积压提醒的起点，之后每翻一倍提醒一次。
    Subscription(std::string topic, const MailboxPtr& mailbox, std::size_t queue_size, Callback cb,
                 ErrorReporter reporter, std::size_t backlog_warn);

    void close();
    [[nodiscard]] bool closed() const;
    void markRequired() noexcept { required_ = true; }
    [[nodiscard]] bool required() const noexcept { return required_; }
    [[nodiscard]] const std::string& topicName() const noexcept { return topic_; }
    [[nodiscard]] std::size_t queueSize() const noexcept { return queue_size_; }
    void offer(const AnyMsg& msg);
    void deliverOne() override;
    [[nodiscard]] SubscriptionStats stats() const;

private:
    std::string topic_;
    std::string subscriber_;
    std::weak_ptr<Mailbox> mailbox_;
    std::size_t queue_size_;
    Callback callback_;
    ErrorReporter reporter_;

    mutable std::mutex mutex_;
    std::deque<AnyMsg> queue_;
    std::size_t high_water_ = 0;
    std::size_t warn_at_;
    std::uint64_t delivered_ = 0;
    std::uint64_t dropped_ = 0;
    bool closed_ = false;
    bool required_ = false;  ///< 只在装配期写
};

/// 订阅者对象。订阅由模块持有，所以默认不需要保存，丢掉它也不会退订；
/// 要提前退订或看统计时才保存。可以比 Runtime 活得久。
class Subscriber {
public:
    Subscriber() = default;
    explicit Subscriber(std::shared_ptr<Subscription> impl) : impl_(std::move(impl)) {}

    /// 退订：队列里没处理的丢弃，之后不再开始新的回调。在别的线程调用时，正在执行的那一次不受影响。
    void shutdown() const {
        if (impl_) impl_->close();
    }
    /// 声明这个输入不可缺：init() 时话题没有发布者就失败。默认是可选的。
    const Subscriber& required() const noexcept {
        if (impl_) impl_->markRequired();
        return *this;
    }
    [[nodiscard]] bool active() const { return impl_ && !impl_->closed(); }
    [[nodiscard]] SubscriptionStats stats() const { return impl_ ? impl_->stats() : SubscriptionStats{}; }

private:
    std::shared_ptr<Subscription> impl_;
};

class Topic final {
public:
    Topic(std::string name, std::type_index type, std::shared_ptr<Hub> hub, ErrorReporter reporter,
          std::size_t backlog_warn);
    Topic(const Topic&) = delete;
    Topic& operator=(const Topic&) = delete;

    [[nodiscard]] std::type_index messageType() const noexcept { return type_; }
    [[nodiscard]] std::size_t subscriberCount() const;
    [[nodiscard]] std::size_t publisherCount() const;
    [[nodiscard]] bool inputsStopped() const noexcept { return hub_->inputsStopped(); }
    void addPublisher();
    void removePublisher();

    void publish(const AnyMsg& msg, bool latch);
    /// 已有 latch 值时补发也走订阅方的收件箱。
    std::shared_ptr<Subscription> subscribe(const MailboxPtr& mailbox, std::size_t queue_size,
                                            Subscription::Callback cb);

private:
    std::string name_;
    std::type_index type_;
    std::shared_ptr<Hub> hub_;
    ErrorReporter reporter_;
    std::size_t backlog_warn_;
    mutable std::mutex mutex_;
    std::vector<std::weak_ptr<Subscription>> subscribers_;
    AnyMsg latched_;
    std::size_t publishers_ = 0;
};

struct PublishOptions {
    bool latch = false;
    /// 外部输入进入系统的入口：requestStop() 之后这个发布端不再发。派生话题不受影响，
    /// 所以已接纳的输入能把因果链走完。
    bool ingress = false;
};

template <class T>
class Publisher {
public:
    Publisher() = default;
    Publisher(std::shared_ptr<Topic> topic, PublishOptions opts) : topic_(std::move(topic)), opts_(opts) {
        if (topic_) topic_->addPublisher();
    }
    Publisher(const Publisher& other) : Publisher(other.topic_, other.opts_) {}
    Publisher(Publisher&& other) noexcept
        : topic_(std::exchange(other.topic_, nullptr)), opts_(other.opts_) {}
    Publisher& operator=(Publisher other) noexcept {
        std::swap(topic_, other.topic_);
        std::swap(opts_, other.opts_);
        return *this;
    }
    ~Publisher() {
        if (topic_) topic_->removePublisher();
    }

    [[nodiscard]] bool valid() const noexcept { return static_cast<bool>(topic_); }

    void publish(MsgPtr<T> msg) const {
        if (!topic_ || (opts_.ingress && topic_->inputsStopped())) return;
        topic_->publish(std::move(msg), opts_.latch);
    }
    void publish(T value) const { publish(std::make_shared<const T>(std::move(value))); }

    [[nodiscard]] std::size_t subscriberCount() const {
        return topic_ ? topic_->subscriberCount() : 0U;
    }

private:
    std::shared_ptr<Topic> topic_;
    PublishOptions opts_;
};

}  // namespace simpleslam::comm
