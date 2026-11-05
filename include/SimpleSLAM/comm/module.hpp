// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file module.hpp
/// 模块基类：用法接近 ROS1，但没有 NodeHandle，订阅与服务端由模块自动管理。
///
/// 模块名是绝对名（如 /slam/odometry）。相对名解析到所在命名空间（/slam），
/// 私有名 ~x 解析到模块自身。每个模块恰有一个收件箱（mailbox.hpp），
/// 订阅消息、参数换代、定时器到期、服务响应都排进它——模块级 FIFO 由此成立。

#include <SimpleSLAM/comm/core.hpp>

#include <atomic>
#include <cassert>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace simpleslam::comm {

/// 模块定时器的句柄。默认不需要保存；需要提前停掉时才用。
///
/// 取消有两层：队列里的条目（不再产生新的到期），以及已经排进模块收件箱、还没执行的
/// 那一格（执行前检查 active 标志，被取消就跳过）。于是在模块自己的执行上下文里调用
/// cancel() 之后，回调一次也不会再跑——不存在"取消了但收件箱里还躺着一格"的尾巴。
///
/// 句柄只弱引用定时器队列：它可以比 Runtime 活得久，Runtime 销毁之后 cancel() 是空操作。
class TimerHandle {
public:
    TimerHandle() = default;
    TimerHandle(std::weak_ptr<TimerQueue> timers, TimerQueue::Id id,
                std::shared_ptr<std::atomic<bool>> active)
        : timers_(std::move(timers)), id_(id), active_(std::move(active)) {}

    void cancel() {
        if (active_) active_->store(false, std::memory_order_release);
        if (id_ == TimerQueue::kInvalidId) return;
        if (auto timers = timers_.lock()) timers->cancel(id_);
        id_ = TimerQueue::kInvalidId;
    }
    [[nodiscard]] bool valid() const noexcept { return id_ != TimerQueue::kInvalidId; }

private:
    std::weak_ptr<TimerQueue> timers_;
    TimerQueue::Id id_ = TimerQueue::kInvalidId;
    std::shared_ptr<std::atomic<bool>> active_;
};

class Module {
public:
    virtual ~Module() = default;
    Module(const Module&) = delete;
    Module& operator=(const Module&) = delete;

    [[nodiscard]] const std::string& name() const noexcept { return name_; }
    [[nodiscard]] const std::string& ns() const noexcept { return namespace_; }
    [[nodiscard]] SerialExecutor& executor() { return mailbox_->executor(); }
    [[nodiscard]] const MailboxPtr& mailbox() const noexcept { return mailbox_; }

    // ── 以下由 Runtime 调用，不属于用户接口 ──

    void attach(RuntimeCore& core, std::string resolved_name) {
        if (!isValidResolvedName(resolved_name)) {
            throw WiringError("模块名必须是合法绝对名: " + resolved_name);
        }
        core_ = &core;
        name_ = std::move(resolved_name);
        const auto slash = name_.rfind('/');
        namespace_ = (slash == 0) ? "/" : name_.substr(0, slash);
        mailbox_ = Mailbox::create(core.io(), name_, core.reporter());
        params_ = core.params().snapshot();
        publishAppliedRevision();
    }

    /// 全部模块装配完成后调用。先刷新参数快照：装配之后、init 之前加载的配置
    /// （命令行、YAML）必须对 onInit 里的 param() 可见，否则声明会读到过期快照。
    void initialize() {
        params_ = core_->params().snapshot();
        publishAppliedRevision();
        onInit();
        initialized_ = true;
    }

    /// 本模块当前手里那份参数快照的版本——"已生效"的证据。
    ///
    /// 可以从任何线程查：它读的是一个单独发布的原子版本号，不是模块执行上下文里
    /// 那个会被换掉的快照指针。快照内容不可变，并不使**同一个 shared_ptr 对象**
    /// 的并发读写安全。
    [[nodiscard]] std::uint64_t appliedParamRevision() const noexcept {
        return applied_param_revision_.load(std::memory_order_acquire);
    }

    /// 换参数快照。Runtime 在提交后投递到各模块上下文，因此一次回调内快照不变。
    ///
    /// 版本只许前进：投递路径已经串行化，这里是兜底——真出现回退就说明有人绕过了
    /// setParams()，宁可保留当前一代并上报，也不要让后面的帧悄悄用回旧参数。
    void adoptParams(ParamSnapshotPtr snapshot) {
        if (!snapshot) return;
        if (params_ && snapshot->revision < params_->revision) {
            ++stale_param_adoptions_;
            detail::reportSafely(core_ != nullptr ? core_->reporter() : ErrorReporter{}, name_,
                                 "参数快照版本回退，已忽略");
            return;
        }
        params_ = std::move(snapshot);
        publishAppliedRevision();
    }

    /// 被忽略的过期参数快照数。恒为零是正常的；非零表示发布路径被绕过了。
    [[nodiscard]] std::uint64_t staleParamAdoptions() const noexcept {
        return stale_param_adoptions_;
    }

    /// 完成已接纳工作的最后一步。由 Runtime 在关闭时、第一次排空之后调用，此时入口
    /// 话题已关、派生话题还开着，不会有回调正在执行。
    ///
    /// 模块在这里**主动核对一次补读游标**：结案不能只靠下一条业务消息——末条通知丢了、
    /// 之后又没有新的输入时，靠通知驱动的补读永远不会发生。这里发出的消息还会被排空。
    void finish() {
        if (finished_ || stopped_) return;
        finished_ = true;
        runHook([this] { onFinish(); });
    }

    /// 模块级停止。由 Runtime 在系统静止之后调用，此时不会有回调正在执行。
    /// 顺序：断输入 → 收尾 → 关收件箱。收件箱最后关，否则 onShutdown 无处投递。
    /// 收尾钩子是用户代码：它抛出的异常就地接住、上报并计数，收件箱照样要关——
    /// 关闭协议不能被一个模块的收尾中断。
    void shutdown() {
        if (stopped_) return;
        stopped_ = true;
        for (auto& sub : subscriptions_) sub->close();
        for (auto& svc : services_) svc->close();
        for (auto& timer : timers_) timer.cancel();
        runHook([this] { onShutdown(); });
        if (mailbox_) mailbox_->close();
    }

    [[nodiscard]] bool stopped() const noexcept { return stopped_; }

    /// 关闭时被丢弃的控制类工作数：没能送达的服务终态、参数换代等。
    /// 恒为零表示关闭时没有欠下任何"说好会回但没回"的东西。
    [[nodiscard]] std::size_t undeliveredTasks() const noexcept {
        return mailbox_ ? mailbox_->undeliveredTasks() : 0;
    }
    /// 因为收件箱控制类工作满载而被拒的累计数。
    [[nodiscard]] std::size_t droppedTasks() const noexcept {
        return mailbox_ ? mailbox_->droppedTasks() : 0;
    }
    /// 本模块的回调与收尾钩子里被接住的异常数。每一次都是一份没做完的工作。
    [[nodiscard]] std::uint64_t isolatedFailures() const noexcept {
        return (mailbox_ ? mailbox_->isolatedFailures() : 0)
               + hook_failures_.load(std::memory_order_acquire);
    }

protected:
    Module() = default;

    /// 装配登记。此时不要做业务，业务回调在全部模块装配完成后才开始。
    virtual void onInit() {}
    /// 关闭时完成已接纳工作：核对补读游标、结案能结案的操作。见 finish()。
    virtual void onFinish() {}
    virtual void onShutdown() {}

    [[nodiscard]] std::string resolve(std::string_view name) const {
        return resolveName(name, name_);
    }
    [[nodiscard]] IClock& clock() { return core_->clock(); }
    [[nodiscard]] RuntimeCore& core() { return *core_; }

    // ── 话题 ──

    template <class T>
    Publisher<T> advertise(std::string_view name, PublishOptions opts = {}) {
        return Publisher<T>(core_->topic<T>(resolve(name)), opts);
    }

    /// 成员函数回调，取 const T&。
    template <class T, class Self>
    void subscribe(std::string_view name, void (Self::*cb)(const T&), SubscribeOptions opts = {}) {
        auto* self = static_cast<Self*>(this);
        subscribeImpl<T>(name, [self, cb](const MsgPtr<T>& m) { (self->*cb)(*m); }, opts);
    }

    /// 成员函数回调，取 MsgPtr<T>（需要延长消息寿命时用）。
    template <class T, class Self>
    void subscribe(std::string_view name, void (Self::*cb)(const MsgPtr<T>&),
                   SubscribeOptions opts = {}) {
        auto* self = static_cast<Self*>(this);
        subscribeImpl<T>(name, [self, cb](const MsgPtr<T>& m) { (self->*cb)(m); }, opts);
    }

    /// 可调用对象回调，消息类型显式给出。
    template <class T, class F>
        requires(!std::is_member_pointer_v<std::decay_t<F>>)
    void subscribe(std::string_view name, F&& f, SubscribeOptions opts = {}) {
        auto fn = std::forward<F>(f);
        subscribeImpl<T>(name, [fn = std::move(fn)](const MsgPtr<T>& m) { fn(*m); }, opts);
    }

    // ── 服务 ──

    template <class Request, class Response, class Self>
    void advertiseService(std::string_view name, Response (Self::*handler)(const Request&),
                          std::size_t max_in_flight = 8) {
        auto* self = static_cast<Self*>(this);
        auto service = core_->registerService<Request, Response>(
            resolve(name), mailbox_,
            [self, handler](const Request& req) { return (self->*handler)(req); }, max_in_flight);
        services_.push_back(std::move(service));
    }

    /// 服务调用端。它可以比 Runtime 活得久：端点解析与定时器都只弱引用运行时，
    /// Runtime 关闭之后调用返回 kStopped，销毁之后同样安全。
    template <class Request, class Response>
    ServiceClient<Request, Response> serviceClient(std::string_view name) {
        const std::string resolved = resolve(name);
        std::weak_ptr<RuntimeCore> core = core_->weak_from_this();
        return ServiceClient<Request, Response>(
            resolved,
            [core, resolved]() -> std::shared_ptr<Service<Request, Response>> {
                auto alive = core.lock();
                return alive ? alive->findService<Request, Response>(resolved).lock() : nullptr;
            },
            mailbox_, core_->timerLink(), &core_->clock(), core_->reporter());
    }

    // ── 定时器 ──

    template <class Self>
    TimerHandle createTimer(Duration period, void (Self::*cb)()) {
        auto* self = static_cast<Self*>(this);
        return createTimer(period, [self, cb] { (self->*cb)(); });
    }

    /// 周期定时器。
    ///
    /// 到期只往收件箱排一格，而不是直接 post——定时器与消息之间也要有确定的先后。
    /// 这一格是**可合并**的：同一个定时器至多有一格待执行。唤醒的语义是
    /// "去看一眼当前状态"，模块长时间没被驱动时积压一千次唤醒没有意义，只会把
    /// 收件箱撑大。需要"每一拍都算数"的周期工作，自己在回调里按时钟补算。
    ///
    /// 取消的边界：在模块自己的执行上下文里 cancel() 之后，回调不再执行——
    /// 已经排进收件箱的那一格也会在执行前被挡住（见 TimerHandle）。
    TimerHandle createTimer(Duration period, std::function<void()> cb) {
        MailboxPtr mailbox = mailbox_;
        auto shared_cb = std::make_shared<std::function<void()>>(std::move(cb));
        auto active = std::make_shared<std::atomic<bool>>(true);
        // 合并键取回调体的地址：每个定时器唯一且稳定，且必然非零
        const auto coalesce_key = reinterpret_cast<std::uintptr_t>(shared_cb.get());
        const auto id = core_->timers().schedule(
            core_->clock().now() + period, period, [mailbox, shared_cb, active, coalesce_key] {
                // 回调的异常交给收件箱的执行边界：接住、上报、计数
                (void)mailbox->postTask(
                    [shared_cb, active] {
                        if (!active->load(std::memory_order_acquire)) return;
                        (*shared_cb)();
                    },
                    static_cast<std::uint64_t>(coalesce_key));
            });
        TimerHandle handle(core_->timerLink(), id, std::move(active));
        timers_.push_back(handle);
        return handle;
    }

    // ── 参数 ──

    /// ROS1 风格：不存在则以默认值声明，返回当前生效的值。
    ///
    /// 声明只补缺失值。装配期已由配置给定的值优先于这里的默认值，
    /// 因此声明之后必须重新取快照——模块手里的那份可能早于配置加载。
    ///
    /// **声明只在装配期（onInit 里）刷新快照**。运行期第一次碰到未声明的名字时，
    /// 若照样去取全局最新快照，就会越过收件箱里排队中的参数更新，破坏"一次回调内
    /// 快照不变"与"这一帧用的是哪一代参数"的对应。所以运行期只登记声明、返回默认值，
    /// 新名字随下一代快照正常生效。
    template <class T>
    T param(std::string_view name, T default_value) {
        const std::string resolved = resolve(name);
        if (!params_ || !params_->has(resolved)) {
            core_->params().declare(resolved, default_value);
            if (!initialized_) {
                params_ = core_->params().snapshot();
                publishAppliedRevision();
            } else {
                detail::reportSafely(core_->reporter(), name_,
                                     "运行期首次声明参数，要到下一代参数快照才生效: " + resolved);
                return default_value;
            }
        }
        if (auto v = params_->get<T>(resolved)) return *v;
        return default_value;
    }

    template <class T>
    bool getParam(std::string_view name, T& out) const {
        if (!params_) return false;
        if (auto v = params_->get<T>(resolveName(name, name_))) {
            out = *v;
            return true;
        }
        return false;
    }

    /// 本模块当前看到的参数快照。一次回调之内不会变。
    [[nodiscard]] const ParamSnapshotPtr& paramSnapshot() const noexcept { return params_; }

private:
    template <class T>
    void subscribeImpl(std::string_view name, std::function<void(const MsgPtr<T>&)> cb,
                       SubscribeOptions opts) {
        auto topic = core_->topic<T>(resolve(name));
        subscriptions_.push_back(topic->subscribe(mailbox_, std::move(cb), opts));
    }

    /// 在执行边界之外调用的用户钩子（完成、收尾）：异常接住、上报、计数。
    template <class F>
    void runHook(F&& hook) noexcept {
        detail::RuntimeThreadMark mark;  // 完成/收尾钩子同样不能同步等待本运行时退出
        try {
            hook();
        } catch (const std::exception& e) {
            hook_failures_.fetch_add(1, std::memory_order_acq_rel);
            detail::reportSafely(core_ != nullptr ? core_->reporter() : ErrorReporter{}, name_,
                                 e.what());
        } catch (...) {
            hook_failures_.fetch_add(1, std::memory_order_acq_rel);
            detail::reportSafely(core_ != nullptr ? core_->reporter() : ErrorReporter{}, name_,
                                 "模块钩子抛出未知异常");
        }
    }

    void publishAppliedRevision() noexcept {
        applied_param_revision_.store(params_ ? params_->revision : 0, std::memory_order_release);
    }

    RuntimeCore* core_ = nullptr;
    std::string name_;
    std::string namespace_;
    MailboxPtr mailbox_;
    /// 只在模块自己的执行上下文里读写；别的线程要问"生效到哪一代"，读下面的原子版本号。
    ParamSnapshotPtr params_;
    std::atomic<std::uint64_t> applied_param_revision_{0};
    std::atomic<std::uint64_t> hook_failures_{0};
    bool finished_ = false;
    bool stopped_ = false;
    bool initialized_ = false;
    std::uint64_t stale_param_adoptions_ = 0;

    std::vector<std::shared_ptr<SubscriptionBase>> subscriptions_;
    std::vector<std::shared_ptr<ServiceBase>> services_;
    std::vector<TimerHandle> timers_;
};

}  // namespace simpleslam::comm
