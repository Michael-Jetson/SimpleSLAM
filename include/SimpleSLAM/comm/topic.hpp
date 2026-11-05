// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file topic.hpp
/// 话题：类型化的一对多投递。
///
/// 四条硬规则：
/// 1. 容量约束在进入执行器队列之前执行——先接纳后投递，不把无界任务积压当队列。
/// 2. latch 补发也必须走订阅方的执行上下文，不能在 subscribe 的调用线程上执行。
/// 3. 回调与满载上报一律在内部锁之外进行，且上报路径本身有兜底。回调的异常
///    由收件箱的执行边界统一接住并计数（mailbox.hpp），订阅端不各自吞。
/// 4. 已接纳的消息排进模块收件箱，而不是各订阅端自持队列各自整批处理——
///    后者只保证"同一订阅 FIFO"，跨话题会乱序（mailbox.hpp）。

#include <SimpleSLAM/comm/execution.hpp>
#include <SimpleSLAM/comm/mailbox.hpp>
#include <SimpleSLAM/comm/types.hpp>

#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>

namespace simpleslam::comm {

/// 订阅句柄的类型无关基类。模块持有它，析构即退订。
class SubscriptionBase : public DeliveryTarget {
public:
    ~SubscriptionBase() override = default;

    /// 停止接收。已接纳但未处理的消息按 close 策略丢弃，不再有新回调。
    virtual void close() = 0;
    [[nodiscard]] virtual const std::string& topicName() const noexcept = 0;
    /// 本订阅端的交付策略。装配期检查"谁是主消费者"要用它。
    [[nodiscard]] virtual Delivery delivery() const noexcept = 0;

protected:
    SubscriptionBase() = default;
};

/// 一个订阅端：自己的有界缓冲 + 模块收件箱。
///
/// 缓冲只管容量（深度与满载策略），顺序由收件箱决定：每接纳一条就往收件箱排一格，
/// 收件箱按到达顺序回头调用 deliverOne() 取走一条。两者的条数恒等，
/// 因此 kLatest 的覆盖不排新格（待处理条数没变），kDropOldest 的挤出也不排新格。
///
/// ## 可丢档的越序范围
///
/// 格子指向的是**订阅端**而不是某一条消息，所以丢弃/覆盖之后，该订阅端里
/// **仍然存活**的条目会继承更早的格位，整体前移到其他话题的条目之前。
/// 容量 2、DropOldest，依次发布 `A1 B2 A3 A4`：格子是 `[A B A]`，A 的队列丢掉
/// A1 之后是 `[A3 A4]`，于是实际投递顺序是 `A3 B2 A4`。A3 与 B2 既没被丢也没被
/// 覆盖，相对顺序却变了。
///
/// 因此准确的承诺是：**可靠档**订阅端之间按接纳顺序投递；可丢/可合并档一旦发生
/// 丢弃或覆盖，该订阅端此后待投递的条目相对其他话题的顺序即不再受保证。要判断
/// "这一帧当时用的是哪份校正、哪代参数"，读帧产物自带的字段，不要从投递次序推断
/// （不变量 16）。
///
/// 保持存活条目顺序并非做不到——挤出时把该订阅端最旧的那个格子从收件箱摘掉、
/// 在尾部补一个，格数仍恒等于待处理条数。那需要可 O(1) 摘除的侵入式格链表。
/// 暂不做的理由是实现与维护成本，不是不可能（这一条推翻了 ADR-0015 的原论据）。
template <class T>
class Subscription final : public SubscriptionBase,
                           public std::enable_shared_from_this<Subscription<T>> {
public:
    using Callback = std::function<void(const MsgPtr<T>&)>;

    /// 收件箱用弱引用持有。
    ///
    /// 排进收件箱的一格指向订阅端本身（`shared_ptr<DeliveryTarget>`），订阅端若再
    /// 强持有收件箱，两者就互相咬住：只要还有一格没被处理，这一对谁也放不掉。
    /// 模块正常关闭时 close() 会清空格子解开它，但"忘了关"不该变成泄漏。
    /// 收件箱的所有权始终在模块手里，弱引用取不到就说明模块已经没了，此时拒收即可。
    Subscription(std::string topic, const MailboxPtr& mailbox, Callback cb,
                 SubscribeOptions opts, ErrorReporter reporter)
        : topic_(std::move(topic)),
          mailbox_(mailbox),
          callback_(std::move(cb)),
          opts_(opts),
          reporter_(std::move(reporter)) {
        if (opts_.queue_depth == 0) opts_.queue_depth = 1;
    }

    ~Subscription() override = default;

    void close() override {
        std::deque<MsgPtr<T>> discarded;
        MsgPtr<T> latest;
        {
            std::lock_guard lock(mutex_);
            closed_ = true;
            discarded.swap(queue_);
            latest = std::move(latest_);
            latest_.reset();
        }
        // 在锁外析构：丢弃的消息可能持有大块点云
    }

    [[nodiscard]] const std::string& topicName() const noexcept override { return topic_; }
    [[nodiscard]] Delivery delivery() const noexcept override { return opts_.delivery; }

    /// 接纳一条消息。返回接纳结果；待处理条数增加时才往收件箱排一格。
    ///
    /// **接纳的判据是"排进了收件箱"**，不是"进了订阅端自己的队列"：两者之间收件箱
    /// 可能已经关闭，那时消息躺在订阅端里、却没有一格会来取它，发布方看到的
    /// "已接纳"就是假的。所以先确认收件箱还开着、再改队列；万一排格时收件箱恰好
    /// 关了，把刚放进去的那条撤回来，如实返回 kClosed。
    ///
    /// **线性化点**：排格在订阅端的锁内完成。于是同一订阅端里，消息进队列的次序与
    /// 格子进收件箱的次序一致；跨订阅端的全局次序就是各格进收件箱的次序
    /// （收件箱自己的锁定序）。可靠档之间"按接纳顺序投递"说的正是这个次序。
    Admit offer(const MsgPtr<T>& msg) {
        Admit result = Admit::kAccepted;
        MsgPtr<T> displaced;  ///< 被覆盖/挤出的消息，锁外析构
        {
            std::lock_guard lock(mutex_);
            if (closed_) return Admit::kClosed;
            auto mailbox = mailbox_.lock();
            if (!mailbox || mailbox->closed()) return Admit::kClosed;  // 没人会来取

            bool need_slot = false;
            bool pushed_back = false;
            if (opts_.delivery == Delivery::kLatest) {
                if (latest_) {
                    result = Admit::kCoalesced;
                    displaced = std::move(latest_);
                } else {
                    need_slot = true;
                }
                latest_ = msg;
            } else if (queue_.size() < opts_.queue_depth) {
                queue_.push_back(msg);
                need_slot = true;
                pushed_back = true;
            } else if (opts_.delivery == Delivery::kDropOldest) {
                displaced = std::move(queue_.front());
                queue_.pop_front();
                queue_.push_back(msg);
                result = Admit::kDroppedOldest;
            } else {
                return Admit::kRejectedFull;  // kReliable 满载：拒新并告知发布方
            }

            if (need_slot && !mailbox->postMessage(this->shared_from_this())) {
                // 检查之后、排格之前收件箱关了：撤回刚放进去的那条
                if (pushed_back) {
                    queue_.pop_back();
                } else {
                    latest_.reset();
                }
                return Admit::kClosed;
            }
        }
        displaced.reset();
        if (result == Admit::kDroppedOldest) {
            detail::reportSafely(reporter_, topic_, "订阅队列已满，丢弃最旧一条");
        }
        return result;
    }

    /// 由收件箱在模块执行上下文里回调：取走恰好一条已接纳的消息。
    void deliverOne() override {
        MsgPtr<T> msg;
        {
            std::lock_guard lock(mutex_);
            if (closed_) return;
            if (latest_) {
                msg = std::move(latest_);
                latest_.reset();
            } else if (!queue_.empty()) {
                msg = std::move(queue_.front());
                queue_.pop_front();
            } else {
                return;  // 已被 close 清空：这一格作废，不是错误
            }
        }
        callback_(msg);  // 锁已释放；异常交给收件箱的执行边界
    }

    [[nodiscard]] std::size_t queuedForTest() const {
        std::lock_guard lock(mutex_);
        return queue_.size() + (latest_ ? 1U : 0U);
    }

private:
    std::string topic_;
    std::weak_ptr<Mailbox> mailbox_;
    Callback callback_;
    SubscribeOptions opts_;
    ErrorReporter reporter_;

    mutable std::mutex mutex_;
    std::deque<MsgPtr<T>> queue_;
    MsgPtr<T> latest_;
    bool closed_ = false;
};

/// 话题的类型无关基类，供注册表按名字存放并做类型校验。
class TopicBase {
public:
    virtual ~TopicBase() = default;
    TopicBase(const TopicBase&) = delete;
    TopicBase& operator=(const TopicBase&) = delete;

    [[nodiscard]] virtual std::type_index messageType() const noexcept = 0;
    [[nodiscard]] virtual const std::string& name() const noexcept = 0;
    [[nodiscard]] virtual std::size_t subscriberCount() const = 0;
    /// 可靠档订阅者数。可靠档是唯一会"拒收"的档位，因此
    /// "发布被拒 ⇔ 主消费者拒收"只在它恰好为 1 时成立。
    [[nodiscard]] virtual std::size_t reliableSubscriberCount() const = 0;
    [[nodiscard]] virtual std::size_t publisherCount() const = 0;

    /// 停止接纳新发布。已接纳的消息仍会被处理完——停止输入与销毁是两件事。
    virtual void close() = 0;
    [[nodiscard]] virtual bool closed() const = 0;

    /// 外部输入的入口话题。停止时先关它，派生话题等因果链排空后再关。
    virtual void markIngress() = 0;
    [[nodiscard]] virtual bool ingress() const = 0;

protected:
    TopicBase() = default;
};

template <class T>
class Topic final : public TopicBase {
public:
    Topic(std::string name, ErrorReporter reporter)
        : name_(std::move(name)), reporter_(std::move(reporter)) {}

    [[nodiscard]] std::type_index messageType() const noexcept override { return typeid(T); }
    [[nodiscard]] const std::string& name() const noexcept override { return name_; }

    [[nodiscard]] std::size_t subscriberCount() const override {
        std::lock_guard lock(mutex_);
        return subscribers_.size();
    }
    [[nodiscard]] std::size_t reliableSubscriberCount() const override {
        std::lock_guard lock(mutex_);
        std::size_t count = 0;
        for (const auto& weak : subscribers_) {
            if (auto sub = weak.lock(); sub && sub->delivery() == Delivery::kReliable) ++count;
        }
        return count;
    }
    [[nodiscard]] std::size_t publisherCount() const override {
        std::lock_guard lock(mutex_);
        return publishers_;
    }

    void close() override {
        std::lock_guard lock(mutex_);
        closed_ = true;
    }
    [[nodiscard]] bool closed() const override {
        std::lock_guard lock(mutex_);
        return closed_;
    }

    void markIngress() override {
        std::lock_guard lock(mutex_);
        ingress_ = true;
    }
    [[nodiscard]] bool ingress() const override {
        std::lock_guard lock(mutex_);
        return ingress_;
    }

    void addPublisher() {
        std::lock_guard lock(mutex_);
        ++publishers_;
    }
    void removePublisher() {
        std::lock_guard lock(mutex_);
        if (publishers_ > 0) --publishers_;
    }

    /// 发布。先在锁外拿到订阅快照，再逐个接纳；latch 值在本话题保留。
    ///
    /// 被新 latch 值顶掉的旧值在锁外析构：消息的析构可能是用户代码，它若再调回本话题，
    /// 在锁内析构就会自锁。
    PublishResult publish(const MsgPtr<T>& msg, bool latch) {
        std::vector<std::shared_ptr<Subscription<T>>> snapshot;
        MsgPtr<T> replaced;
        {
            std::lock_guard lock(mutex_);
            if (closed_) {
                // 话题已关：零订阅者时所有计数都是 0，单靠计数区分不出
                // "停止中"与"接线成功但没人订阅"，所以显式置位。
                PublishResult stopped;
                stopped.subscribers = subscribers_.size();
                stopped.closed = subscribers_.size();
                stopped.topic_closed = true;
                return stopped;
            }
            if (latch) {
                replaced = std::move(latched_);
                latched_ = msg;
            }
            snapshot.reserve(subscribers_.size());
            for (auto& weak : subscribers_) {
                if (auto sub = weak.lock()) snapshot.push_back(std::move(sub));
            }
        }

        PublishResult result;
        result.subscribers = snapshot.size();
        for (auto& sub : snapshot) {
            const bool reliable = sub->delivery() == Delivery::kReliable;
            if (reliable) ++result.reliable_subscribers;
            switch (sub->offer(msg)) {
                case Admit::kAccepted:
                    ++result.accepted;
                    if (reliable) ++result.reliable_accepted;
                    break;
                case Admit::kCoalesced: ++result.coalesced; break;
                case Admit::kDroppedOldest: ++result.dropped_oldest; break;
                case Admit::kRejectedFull: ++result.rejected_full; break;
                case Admit::kClosed: ++result.closed; break;
            }
        }
        if (result.rejected_full > 0) {
            detail::reportSafely(reporter_, name_, "可靠订阅端满载，本次发布被拒收");
        }
        return result;
    }

    /// 订阅。若已有 latch 值，补发任务投递到订阅方的执行上下文，不在此处执行。
    std::shared_ptr<Subscription<T>> subscribe(MailboxPtr mailbox,
                                               typename Subscription<T>::Callback cb,
                                               SubscribeOptions opts) {
        auto sub = std::make_shared<Subscription<T>>(name_, std::move(mailbox), std::move(cb),
                                                     opts, reporter_);
        MsgPtr<T> latched;
        {
            std::lock_guard lock(mutex_);
            subscribers_.emplace_back(sub);
            pruneLocked();
            latched = latched_;
        }
        if (latched) sub->offer(latched);
        return sub;
    }

private:
    void pruneLocked() {
        std::erase_if(subscribers_, [](const std::weak_ptr<Subscription<T>>& w) {
            return w.expired();
        });
    }

    std::string name_;
    ErrorReporter reporter_;
    mutable std::mutex mutex_;
    std::vector<std::weak_ptr<Subscription<T>>> subscribers_;
    MsgPtr<T> latched_;
    std::size_t publishers_ = 0;
    bool closed_ = false;
    bool ingress_ = false;
};

/// 已解析的发布端点。模块停止后继续使用会返回 closed 结果，不是未定义行为。
template <class T>
class Publisher {
public:
    Publisher() = default;
    Publisher(std::shared_ptr<Topic<T>> topic, PublishOptions opts)
        : topic_(std::move(topic)), opts_(opts) {
        if (topic_) {
            topic_->addPublisher();
            // 任何一个发布端声明自己是入口，这个话题就按入口处理
            if (opts_.ingress) topic_->markIngress();
        }
    }

    Publisher(const Publisher& other) : topic_(other.topic_), opts_(other.opts_) {
        if (topic_) topic_->addPublisher();
    }
    Publisher& operator=(const Publisher& other) {
        if (this != &other) {
            reset();
            topic_ = other.topic_;
            opts_ = other.opts_;
            if (topic_) topic_->addPublisher();
        }
        return *this;
    }
    Publisher(Publisher&& other) noexcept
        : topic_(std::move(other.topic_)), opts_(other.opts_) {
        other.topic_.reset();
    }
    Publisher& operator=(Publisher&& other) noexcept {
        if (this != &other) {
            reset();
            topic_ = std::move(other.topic_);
            opts_ = other.opts_;
            other.topic_.reset();
        }
        return *this;
    }
    ~Publisher() { reset(); }

    [[nodiscard]] bool valid() const noexcept { return static_cast<bool>(topic_); }

    PublishResult publish(const MsgPtr<T>& msg) {
        if (!topic_) {
            // 没有绑定话题的发布端：不能报"成功"，否则 ok() 会把"根本没发"说成"发了"
            PublishResult r;
            r.closed = 1;
            r.topic_closed = true;
            return r;
        }
        return topic_->publish(msg, opts_.latch);
    }

    PublishResult publish(T value) {
        return publish(std::make_shared<const T>(std::move(value)));
    }

    [[nodiscard]] std::size_t subscriberCount() const {
        return topic_ ? topic_->subscriberCount() : 0U;
    }

private:
    void reset() {
        if (topic_) {
            topic_->removePublisher();
            topic_.reset();
        }
    }

    std::shared_ptr<Topic<T>> topic_;
    PublishOptions opts_;
};

}  // namespace simpleslam::comm
