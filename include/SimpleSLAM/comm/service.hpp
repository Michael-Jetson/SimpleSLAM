// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file service.hpp
/// 服务：一请求一响应，同名只有一个服务端。
///
/// 规则：服务端处理在其属主模块的收件箱里排队；响应回到调用方的收件箱；
/// 超时不撤销服务端已产生的副作用。阻塞式 call 只给运行时之外的控制线程用，
/// 在运行时线程内调用会被入口直接拒绝。
///
/// **一次异步调用的完整生命周期**：
///
/// - 受理（asyncCall 返回 kOk）之前的一切拒绝都**同步**返回，不回调：调用方已关闭、
///   在飞调用已达上限、没有服务端。不受理，也就不欠一个终态。
/// - 受理之后恰好**结案**一次：服务端给出结果、超时、或服务端拒收，先到的那一路赢。
///   结案时立即撤掉超时定时器，与回调送不送得到无关。
/// - 服务端收下请求之后，**结案权随任务走**：任务执行了就按处理结果结案；任务没执行
///   就被销毁（服务端收件箱关闭时被丢弃、或根本没排进去），析构时以 kStopped / kRejected
///   结案并归还服务端名额。受理之后"恰好结案一次"因此不依赖任务一定会跑，
///   无超时的调用与阻塞 call 也不会永远等下去。
/// - 结案之后把完成回调投到调用方的收件箱，**至多**执行一次：调用方模块已经关闭时
///   投不进去，就计入 undeliveredCompletions()，不在错误的线程上回调一个可能已经
///   销毁的模块。
/// - **在飞名额从受理占到终态送达（或确定送不到）为止**。只占到结案是不够的：服务端
///   对超限请求是同步拒绝的，名额立刻归还，而完成通知还堆在调用方的收件箱里——
///   不驱动调用方时照样能无限发起调用。归还由随完成任务一起走的令牌负责：任务执行完、
///   投递失败、或收件箱关闭时被丢弃，令牌析构即归还，送不到的同时计数。
///
/// 服务处理函数的异常变成 kHandlerError 响应——调用方要的是一个终态，不是一条日志。
/// 完成回调是调用方自己的代码，它的异常交给调用方收件箱的执行边界：接住、上报、计数。

#include <SimpleSLAM/comm/clock.hpp>
#include <SimpleSLAM/comm/execution.hpp>
#include <SimpleSLAM/comm/mailbox.hpp>
#include <SimpleSLAM/comm/topic.hpp>
#include <SimpleSLAM/comm/types.hpp>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <typeindex>
#include <utility>

namespace simpleslam::comm {

/// 服务端点的类型无关基类。
class ServiceBase {
public:
    virtual ~ServiceBase() = default;
    ServiceBase(const ServiceBase&) = delete;
    ServiceBase& operator=(const ServiceBase&) = delete;

    [[nodiscard]] virtual std::type_index requestType() const noexcept = 0;
    [[nodiscard]] virtual std::type_index responseType() const noexcept = 0;
    [[nodiscard]] virtual const std::string& name() const noexcept = 0;
    virtual void close() = 0;

protected:
    ServiceBase() = default;
};

/// 服务端。max_in_flight 是准入上限：超过即拒绝，而不是无限堆任务。
template <class Request, class Response>
class Service final : public ServiceBase,
                      public std::enable_shared_from_this<Service<Request, Response>> {
public:
    using Handler = std::function<Response(const Request&)>;
    using Done = std::function<void(ServiceStatus, std::optional<Response>)>;

    Service(std::string name, MailboxPtr owner, Handler handler, std::size_t max_in_flight,
            ErrorReporter reporter)
        : name_(std::move(name)),
          owner_(std::move(owner)),
          handler_(std::move(handler)),
          max_in_flight_(max_in_flight == 0 ? 1 : max_in_flight),
          reporter_(std::move(reporter)) {}

    [[nodiscard]] std::type_index requestType() const noexcept override { return typeid(Request); }
    [[nodiscard]] std::type_index responseType() const noexcept override {
        return typeid(Response);
    }
    [[nodiscard]] const std::string& name() const noexcept override { return name_; }

    void close() override { closed_.store(true, std::memory_order_release); }

    /// 准入检查在投递之前完成。done 在服务端上下文里被调用，调用方负责转回自己的上下文。
    void invoke(MsgPtr<Request> request, Done done) {
        if (closed_.load(std::memory_order_acquire)) {
            done(ServiceStatus::kStopped, std::nullopt);
            return;
        }
        const std::size_t current = in_flight_.fetch_add(1, std::memory_order_acq_rel);
        if (current >= max_in_flight_) {
            in_flight_.fetch_sub(1, std::memory_order_acq_rel);
            detail::reportSafely(reporter_, name_, "服务端在飞请求已达上限，本次拒绝");
            done(ServiceStatus::kRejected, std::nullopt);
            return;
        }

        // 从这里起名额与结案权都在 admitted 手里：任务跑了按结果结案，没跑就析构时结案
        auto admitted = std::make_shared<Admitted>(this->shared_from_this(), std::move(done));
        const bool queued = owner_->postTask([admitted, request = std::move(request)] {
            Service& self = *admitted->service;
            if (self.closed_.load(std::memory_order_acquire)) {
                admitted->finish(ServiceStatus::kStopped, std::nullopt);
                return;
            }
            ServiceStatus status = ServiceStatus::kOk;
            std::optional<Response> value;
            try {
                value = self.handler_(*request);
            } catch (const std::exception& e) {
                status = ServiceStatus::kHandlerError;
                detail::reportSafely(self.reporter_, self.name_, e.what());
            } catch (...) {
                status = ServiceStatus::kHandlerError;
                detail::reportSafely(self.reporter_, self.name_, "服务处理函数抛出未知异常");
            }
            admitted->finish(status, std::move(value));
        });
        if (!queued) {
            // 服务端收件箱已关或控制类工作满载：任务那一份已随投递失败析构，
            // 这里这一份放手时以对应的状态结案、归还名额
            admitted->dropped_status = closed_.load(std::memory_order_acquire)
                                           ? ServiceStatus::kStopped
                                           : ServiceStatus::kRejected;
            detail::reportSafely(reporter_, name_, "服务端收件箱拒收，本次请求未受理");
        }
    }

    [[nodiscard]] std::size_t inFlight() const noexcept {
        return in_flight_.load(std::memory_order_acquire);
    }

private:
    /// 服务端收下、尚未执行的一次请求，持有它的结案权与在飞名额。
    ///
    /// 任务执行时由 finish() 按处理结果结案；任务没执行就被销毁时，析构以
    /// dropped_status 结案。收件箱在锁外析构被丢弃的任务，所以析构里投递终态不会
    /// 与收件箱自己的锁相撞。
    struct Admitted {
        Admitted(std::shared_ptr<Service> owner, Done callback)
            : service(std::move(owner)), done(std::move(callback)) {}
        Admitted(const Admitted&) = delete;
        Admitted& operator=(const Admitted&) = delete;

        void finish(ServiceStatus status, std::optional<Response> value) {
            finished = true;
            service->in_flight_.fetch_sub(1, std::memory_order_acq_rel);
            done(status, std::move(value));
        }
        ~Admitted() {
            if (finished) return;
            service->in_flight_.fetch_sub(1, std::memory_order_acq_rel);
            try {
                done(dropped_status, std::nullopt);
            } catch (...) {
                detail::reportSafely(service->reporter_, service->name_, "服务终态投递失败");
            }
        }

        std::shared_ptr<Service> service;
        Done done;
        ServiceStatus dropped_status = ServiceStatus::kStopped;
        bool finished = false;
    };

    std::string name_;
    MailboxPtr owner_;
    Handler handler_;
    std::size_t max_in_flight_;
    ErrorReporter reporter_;
    std::atomic<std::size_t> in_flight_{0};
    std::atomic<bool> closed_{false};
};

/// 调用端点。保存已解析的服务端，不在每次调用时按字符串查找。
template <class Request, class Response>
class ServiceClient {
public:
    using Completion = std::function<void(ServiceResult<Response>)>;

    /// 端点按需解析：装配顺序因此不重要，服务端晚于客户端登记也能找到。
    using Resolver = std::function<std::shared_ptr<Service<Request, Response>>()>;

    ServiceClient() = default;
    ServiceClient(std::string name, Resolver resolver, MailboxPtr caller,
                  std::weak_ptr<TimerQueue> timers, IClock* clock, ErrorReporter reporter = {})
        : name_(std::move(name)),
          resolver_(std::move(resolver)),
          caller_(std::move(caller)),
          timers_(timers),
          clock_(clock),
          reporter_(std::move(reporter)) {}

    [[nodiscard]] bool valid() const { return resolver_ && resolver_() != nullptr; }
    [[nodiscard]] const std::string& name() const noexcept { return name_; }

    /// 在飞异步调用的默认上限：从受理到结案都占一个名额。
    static constexpr std::size_t kDefaultMaxPending = 256;

    /// 异步调用。返回的是**受理结果**，不是终态。
    ///
    /// 返回 kOk 表示已受理：此后恰好结案一次，完成回调至多执行一次。
    /// 其余返回值都表示**未受理**，不会有回调：kStopped（调用方已关闭）、
    /// kRejected（在飞调用已达上限）、kNoProvider（没有服务端）。
    [[nodiscard]] ServiceStatus asyncCall(Request request, Duration timeout,
                                          Completion completion) {
        if (!caller_ || caller_->closed()) {
            detail::reportSafely(reporter_, name_, "调用方模块已关闭，服务调用在入口拒绝");
            return ServiceStatus::kStopped;
        }
        auto service = resolver_ ? resolver_() : nullptr;
        if (!service) return ServiceStatus::kNoProvider;

        // 名额从受理占到结案：满了就在入口拒绝，不让在飞调用与它们的定时器无界增长
        if (budget_->pending.fetch_add(1, std::memory_order_acq_rel) >= budget_->max_pending) {
            budget_->pending.fetch_sub(1, std::memory_order_acq_rel);
            detail::reportSafely(reporter_, name_, "在飞服务调用已达上限，本次在入口拒绝");
            return ServiceStatus::kRejected;
        }

        auto pending = std::make_shared<Pending>();
        pending->completion = std::move(completion);
        pending->caller = caller_;
        pending->reporter = reporter_;
        pending->where = name_;
        pending->budget = budget_;

        // 调用方还开着就说明运行时活着：时钟可用。定时器队列仍按弱引用取
        auto timers = timers_.lock();
        if (timeout.count() > 0 && timers && clock_ != nullptr) {
            pending->timers = timers_;
            pending->timer_id.store(
                timers->schedule(clock_->now() + timeout, Duration{0},
                                 [pending] { pending->settle(ServiceStatus::kTimeout, {}); }),
                std::memory_order_release);
        }

        service->invoke(std::make_shared<const Request>(std::move(request)),
                        [pending](ServiceStatus status, std::optional<Response> value) {
                            pending->settle(status, std::move(value));
                        });
        return ServiceStatus::kOk;
    }

    /// 已受理、尚未结案的调用数。
    [[nodiscard]] std::size_t pendingCalls() const noexcept {
        return budget_->pending.load(std::memory_order_acquire);
    }
    /// 已结案、但完成回调没能送到调用方的次数（调用方模块已关闭）。
    [[nodiscard]] std::size_t undeliveredCompletions() const noexcept {
        return budget_->undelivered.load(std::memory_order_acquire);
    }

    /// 阻塞调用。只允许运行时线程之外的控制线程使用，否则会等待自己推进的响应。
    ServiceResult<Response> call(Request request, Duration timeout) {
        if (inRuntimeThread()) {
            return {ServiceStatus::kWouldDeadlock, std::nullopt};
        }
        // 与异步调用同一条关闭协议：调用方已关（运行时已关闭或已销毁）就不再发起
        if (!caller_ || caller_->closed()) return {ServiceStatus::kStopped, std::nullopt};
        auto service = resolver_ ? resolver_() : nullptr;
        if (!service) return {ServiceStatus::kNoProvider, std::nullopt};

        auto state = std::make_shared<BlockingState>();
        service->invoke(std::make_shared<const Request>(std::move(request)),
                        [state](ServiceStatus status, std::optional<Response> value) {
                            {
                                std::lock_guard lock(state->mutex);
                                if (state->done) return;
                                state->done = true;
                                state->result.status = status;
                                state->result.value = std::move(value);
                            }
                            state->cv.notify_all();
                        });

        std::unique_lock lock(state->mutex);
        const bool signalled =
            timeout.count() > 0
                ? state->cv.wait_for(lock, timeout, [&] { return state->done; })
                : (state->cv.wait(lock, [&] { return state->done; }), true);
        if (!signalled) {
            state->done = true;  // 迟到响应此后被丢弃
            return {ServiceStatus::kTimeout, std::nullopt};
        }
        return state->result;
    }

private:
    /// 客户端的在飞预算。拷贝 ServiceClient 共享同一份：名额属于调用方，不属于某个拷贝。
    struct Budget {
        std::atomic<std::size_t> pending{0};
        std::atomic<std::size_t> undelivered{0};
        std::size_t max_pending = kDefaultMaxPending;
    };

    /// 一次已受理的调用。结案（settle）只发生一次，可能来自服务端的执行上下文、
    /// 也可能来自定时器线程，所以用原子标志裁决"先到的那一路"。
    struct Pending {
        Completion completion;
        MailboxPtr caller;
        std::weak_ptr<TimerQueue> timers;
        std::atomic<TimerQueue::Id> timer_id{TimerQueue::kInvalidId};
        ErrorReporter reporter;
        std::string where;
        std::shared_ptr<Budget> budget;
        std::atomic<bool> settled{false};

        /// 结案：撤掉定时器，再把完成回调投到调用方。定时器的清理不依赖回调送达；
        /// 名额随完成任务里的令牌一起走，送达或丢弃时归还。
        void settle(ServiceStatus status, std::optional<Response> value) {
            if (settled.exchange(true, std::memory_order_acq_rel)) return;  // 迟到的那一路
            const auto id = timer_id.load(std::memory_order_acquire);
            if (id != TimerQueue::kInvalidId) {
                if (auto queue = timers.lock()) queue->cancel(id);
            }

            auto token = std::make_shared<Release>(budget);
            const bool queued = caller->postTask(
                [token, callback = std::move(completion), status,
                 value = std::move(value)]() mutable {
                    token->delivered = true;
                    // 完成回调是调用方的代码：异常交给调用方收件箱的执行边界
                    if (callback) callback(ServiceResult<Response>{status, std::move(value)});
                });
            if (!queued) detail::reportSafely(reporter, where, "服务终态未能送达调用方");
        }
    };

    /// 在飞名额的归还令牌。完成任务执行完、投递失败、或随收件箱关闭被丢弃时析构——
    /// 三种结局都会归还名额，没执行过回调的记为送不到。
    struct Release {
        explicit Release(std::shared_ptr<Budget> owner) : budget(std::move(owner)) {}
        Release(const Release&) = delete;
        Release& operator=(const Release&) = delete;
        ~Release() {
            budget->pending.fetch_sub(1, std::memory_order_acq_rel);
            if (!delivered) budget->undelivered.fetch_add(1, std::memory_order_acq_rel);
        }
        std::shared_ptr<Budget> budget;
        bool delivered = false;
    };

    struct BlockingState {
        std::mutex mutex;
        std::condition_variable cv;
        bool done = false;
        ServiceResult<Response> result;
    };

    std::string name_;
    Resolver resolver_;
    MailboxPtr caller_;
    std::weak_ptr<TimerQueue> timers_;
    IClock* clock_ = nullptr;
    ErrorReporter reporter_;
    std::shared_ptr<Budget> budget_ = std::make_shared<Budget>();
};

}  // namespace simpleslam::comm
