// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/comm/topic.hpp>

#include <algorithm>
#include <string>
#include <utility>

namespace simpleslam::comm {

Subscription::Subscription(std::string topic, const MailboxPtr& mailbox, std::size_t queue_size,
                           Callback cb, ErrorReporter reporter, std::size_t backlog_warn)
    : topic_(std::move(topic)),
      subscriber_(mailbox->owner()),
      mailbox_(mailbox),
      queue_size_(queue_size),
      callback_(std::move(cb)),
      reporter_(std::move(reporter)),
      warn_at_(backlog_warn) {}

void Subscription::close() {
    std::deque<AnyMsg> discarded;
    std::lock_guard lock(mutex_);
    closed_ = true;
    discarded.swap(queue_);
}

bool Subscription::closed() const {
    std::lock_guard lock(mutex_);
    return closed_;
}

SubscriptionStats Subscription::stats() const {
    std::lock_guard lock(mutex_);
    return {topic_, subscriber_, queue_size_, queue_.size(), high_water_, delivered_, dropped_};
}

void Subscription::offer(const AnyMsg& msg) {
    AnyMsg displaced;  // 锁外析构
    std::size_t backlog = 0;
    {
        std::lock_guard lock(mutex_);
        if (closed_) return;
        if (queue_size_ > 0 && queue_.size() >= queue_size_) {
            displaced = std::move(queue_.front());
            queue_.pop_front();
            queue_.push_back(msg);
            ++dropped_;
        } else {
            auto mailbox = mailbox_.lock();
            if (!mailbox) return;
            queue_.push_back(msg);
            // 排格在锁内：同一订阅端里进队列的次序与格子进收件箱的次序一致
            if (!mailbox->postSlot(shared_from_this())) {
                queue_.pop_back();
                return;
            }
            high_water_ = std::max(high_water_, queue_.size());
            if (queue_size_ == 0 && warn_at_ > 0 && queue_.size() >= warn_at_) {
                backlog = queue_.size();
                warn_at_ *= 2;
            }
        }
    }
    if (backlog > 0) {
        detail::reportSafely(reporter_, topic_, subscriber_ + " 积压 " + std::to_string(backlog) + " 条");
    }
}

void Subscription::deliverOne() {
    AnyMsg msg;
    {
        std::lock_guard lock(mutex_);
        if (closed_ || queue_.empty()) return;
        msg = std::move(queue_.front());
        queue_.pop_front();
        ++delivered_;
    }
    callback_(msg);
}

Topic::Topic(std::string name, std::type_index type, std::shared_ptr<Hub> hub, ErrorReporter reporter,
             std::size_t backlog_warn)
    : name_(std::move(name)),
      type_(type),
      hub_(std::move(hub)),
      reporter_(std::move(reporter)),
      backlog_warn_(backlog_warn) {}

std::size_t Topic::subscriberCount() const {
    std::lock_guard lock(mutex_);
    std::size_t count = 0;
    for (const auto& weak : subscribers_) {
        if (auto sub = weak.lock(); sub && !sub->closed()) ++count;
    }
    return count;
}

std::size_t Topic::publisherCount() const {
    std::lock_guard lock(mutex_);
    return publishers_;
}

void Topic::addPublisher() {
    std::lock_guard lock(mutex_);
    ++publishers_;
}

void Topic::removePublisher() {
    std::lock_guard lock(mutex_);
    --publishers_;
}

void Topic::publish(const AnyMsg& msg, bool latch) {
    std::vector<std::shared_ptr<Subscription>> snapshot;
    AnyMsg replaced;  // 锁外析构
    {
        std::lock_guard lock(mutex_);
        if (latch) {
            replaced = std::move(latched_);
            latched_ = msg;
        }
        snapshot.reserve(subscribers_.size());
        for (auto& weak : subscribers_) {
            if (auto sub = weak.lock()) snapshot.push_back(std::move(sub));
        }
    }
    for (auto& sub : snapshot) sub->offer(msg);
}

std::shared_ptr<Subscription> Topic::subscribe(const MailboxPtr& mailbox, std::size_t queue_size,
                                               Subscription::Callback cb) {
    auto sub = std::make_shared<Subscription>(name_, mailbox, queue_size, std::move(cb), reporter_, backlog_warn_);
    AnyMsg latched;
    {
        std::lock_guard lock(mutex_);
        subscribers_.emplace_back(sub);
        latched = latched_;
    }
    if (latched) sub->offer(latched);
    return sub;
}

}  // namespace simpleslam::comm
