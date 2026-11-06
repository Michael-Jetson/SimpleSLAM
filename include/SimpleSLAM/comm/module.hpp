// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file module.hpp
/// 模块基类：相当于 ROS 里进程内的一个节点。在线时一个模块一个线程，回调严格串行。
///
///   class Talker : public comm::Module {
///       void onInit() override {
///           pub_ = advertise<Msg>("chatter");
///           subscribe("command", 0, &Talker::onCommand);   // 0 无界；N 丢最旧；1 只留最新
///           subscribe("status", 1, logStatus);             // 普通函数，消息类型从参数推导
///           subscribe<Msg>("raw", 10, [this](const Msg& m) { ... });  // lambda，消息类型显式给出
///           createTimer(100ms, &Talker::onTimer);
///           rate_ = param("~rate", 10.0);
///       }
///       ...
///   };

#include <SimpleSLAM/comm/core.hpp>

#include <atomic>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeindex>
#include <utility>
#include <vector>

namespace simpleslam::comm {

class Runtime;

class Module {
public:
    virtual ~Module() = default;
    Module(const Module&) = delete;
    Module& operator=(const Module&) = delete;

    [[nodiscard]] const std::string& name() const noexcept { return name_; }
    [[nodiscard]] const MailboxPtr& mailbox() const noexcept { return mailbox_; }
    /// 模块已经收尾。收尾之后它是只读的最终状态。
    [[nodiscard]] bool stopped() const noexcept { return stopped_; }

    // ── 装配期配置，由 main 或装配代码在 init() 之前调用 ──

    /// 本模块用到 from 这个话题或服务名时，实际接到 to。模块代码不用改。
    void remap(std::string_view from, std::string_view to);
    void setThreadOptions(ThreadOptions options) { thread_options_ = std::move(options); }

protected:
    Module() = default;

    /// 装配登记。业务回调在全部模块装配完成后才开始。
    virtual void onInit() {}
    /// 第一个回调之前调用，在线时在模块自己的线程上。绑线程的资源（GPU 上下文等）在这里建。
    virtual void onStart() {}
    /// 结束运行时，输入已停、已接纳的工作已走完。这里发出的消息还会被处理。
    virtual void onFinish() {}
    virtual void onShutdown() {}
    /// 换上新一代参数快照之后调用，在模块自己的线程上。
    virtual void onParamsChanged(const ParamSnapshot& /*snapshot*/) {}

    [[nodiscard]] std::string resolve(std::string_view name) const { return resolveName(name, name_); }
    [[nodiscard]] IClock& clock() { return core_->clock(); }
    [[nodiscard]] bool inputsStopped() const noexcept { return core_->hub()->inputsStopped(); }

    // ── 话题 ──

    template <class T>
    Publisher<T> advertise(std::string_view name, PublishOptions opts = {}) {
        advertised_.push_back(resolveEndpoint(name));
        return Publisher<T>(core_->topic<T>(advertised_.back()), opts);
    }

    template <class T>
    Publisher<T> advertise(TopicDesc<T> topic, PublishOptions opts = {}) {
        return advertise<T>(topic.name, opts);
    }

    // 回调可以是成员函数、普通函数或 lambda，都在本模块的线程上串行执行。
    // 参数取 const T& 或 const MsgPtr<T>&，后者用于延长消息寿命。

    template <class T, class Self>
    Subscriber subscribe(std::string_view name, std::size_t queue_size, void (Self::*cb)(const T&)) {
        auto* self = static_cast<Self*>(this);
        return subscribeAny<T>(name, queue_size, [self, cb](const AnyMsg& m) { (self->*cb)(view<T>(m)); });
    }

    template <class T, class Self>
    Subscriber subscribe(std::string_view name, std::size_t queue_size, void (Self::*cb)(const MsgPtr<T>&)) {
        auto* self = static_cast<Self*>(this);
        return subscribeAny<T>(name, queue_size,
                        [self, cb](const AnyMsg& m) { (self->*cb)(std::static_pointer_cast<const T>(m)); });
    }

    template <class T>
    Subscriber subscribe(std::string_view name, std::size_t queue_size, void (*cb)(const T&)) {
        return subscribeAny<T>(name, queue_size, [cb](const AnyMsg& m) { cb(view<T>(m)); });
    }

    template <class T>
    Subscriber subscribe(std::string_view name, std::size_t queue_size, void (*cb)(const MsgPtr<T>&)) {
        return subscribeAny<T>(name, queue_size, [cb](const AnyMsg& m) { cb(std::static_pointer_cast<const T>(m)); });
    }

    template <class T, class F>
        requires(!std::is_member_pointer_v<std::remove_cvref_t<F>>
                 && !std::is_function_v<std::remove_pointer_t<std::remove_cvref_t<F>>>)
    Subscriber subscribe(std::string_view name, std::size_t queue_size, F&& f) {
        return subscribeAny<T>(name, queue_size, [fn = std::forward<F>(f)](const AnyMsg& m) {
            if constexpr (std::is_invocable_v<std::remove_cvref_t<F>&, const T&>) {
                fn(view<T>(m));
            } else {
                fn(std::static_pointer_cast<const T>(m));
            }
        });
    }

    /// 经描述符订阅：消息类型取自描述符，回调的参数类型与它不符时编译不过。
    template <class T, class Callback>
    Subscriber subscribe(TopicDesc<T> topic, std::size_t queue_size, Callback&& callback) {
        return subscribe<T>(topic.name, queue_size, std::forward<Callback>(callback));
    }

    // ── 服务 ──

    template <class Request, class Response, class Self>
    ServiceServer advertiseService(std::string_view name, Response (Self::*handler)(const Request&)) {
        auto* self = static_cast<Self*>(this);
        return advertiseServiceAny<Request, Response>(name, [self, handler](const Request& r) { return (self->*handler)(r); });
    }

    template <class Request, class Response>
    ServiceServer advertiseService(std::string_view name, Response (*handler)(const Request&)) {
        return advertiseServiceAny<Request, Response>(name, handler);
    }

    template <class Request, class Response, class F>
        requires(std::is_invocable_r_v<Response, std::remove_cvref_t<F>&, const Request&>)
    ServiceServer advertiseService(std::string_view name, F&& handler) {
        return advertiseServiceAny<Request, Response>(name, std::forward<F>(handler));
    }

    template <class Request, class Response>
    ServiceClient<Request, Response> serviceClient(std::string_view name) {
        return ServiceClient<Request, Response>(
            makeCaller(*core_, resolveEndpoint(name), typeid(Request), typeid(Response), mailbox_));
    }

    // ── 定时器 ──

    template <class Self>
    TimerHandle createTimer(Duration period, void (Self::*cb)()) {
        auto* self = static_cast<Self*>(this);
        return createTimer(period, std::function<void()>([self, cb] { (self->*cb)(); }));
    }

    /// 周期定时器。到期往收件箱排一格，同一个定时器至多有一格待执行。
    TimerHandle createTimer(Duration period, std::function<void()> cb);

    /// 一次性定时器。
    TimerHandle createOneShot(Duration delay, std::function<void()> cb);
    template <class Self>
    TimerHandle createOneShot(Duration delay, void (Self::*cb)()) {
        auto* self = static_cast<Self*>(this);
        return createOneShot(delay, std::function<void()>([self, cb] { (self->*cb)(); }));
    }

    // ── 参数 ──

    /// 不存在则以默认值声明，返回当前生效的值。配置已给定的值优先于默认值。
    template <class T>
    T param(std::string_view name, T default_value) {
        const std::string resolved = resolve(name);
        if (!params_ || !params_->has(resolved)) {
            core_->params().declare(resolved, default_value);
            // 运行期首次碰到的名字随下一代快照生效，不越过收件箱里排队的更新
            if (initialized_) return default_value;
            params_ = core_->params().snapshot();
        }
        if (auto v = params_->get<T>(resolved)) return *v;
        return default_value;
    }

    template <class T>
    bool getParam(std::string_view name, T& out) const {
        if (!params_) return false;
        if (auto v = params_->get<T>(resolve(name))) {
            out = *v;
            return true;
        }
        return false;
    }

    /// 写入一个已声明的参数。自己也是在下一代快照才看到新值。
    template <class T>
    ParamCommitResult setParam(std::string_view name, T value) {
        return core_->setParams({ParamChange{resolve(name), ParamValue{std::move(value)}}});
    }

    /// 本模块当前的参数快照。一次回调之内不会变。
    [[nodiscard]] const ParamSnapshotPtr& paramSnapshot() const noexcept { return params_; }

    // ── 日志 ──

    /// 本模块的日志对象：`log_.info(…)`、`log_.warn(…)`、`log_.every(1s).warn(…)`。
    /// 接进运行时就带上模块名；本模块的插件挂在它下面，处理每条输入之前设上当前帧。
    logging::Logger log_;  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes)

    // ── 异常 ──

    /// 本模块回调抛出没人处理的异常时怎么办。默认用 RuntimeOptions::on_exception。
    void setExceptionPolicy(ExceptionPolicy policy) noexcept { mailbox_->setExceptionPolicy(policy); }

private:
    friend class Runtime;

    void attach(RuntimeCore& core, std::string resolved_name);
    void initialize();
    void finish();
    void shutdown();
    void start();
    void adoptParams(ParamSnapshotPtr snapshot);
    [[nodiscard]] std::string resolveEndpoint(std::string_view name) const;
    TimerHandle scheduleTimer(Duration delay, Duration period, std::function<void()> cb);
    static ServiceCaller makeCaller(RuntimeCore& core, std::string resolved, std::type_index request,
                                    std::type_index response, MailboxPtr caller);

    template <class F>
    void runHook(F&& hook) noexcept;

    template <class T>
    [[nodiscard]] static const T& view(const AnyMsg& m) noexcept {
        return *static_cast<const T*>(m.get());
    }

    template <class Request, class Response, class F>
    ServiceServer advertiseServiceAny(std::string_view name, F&& handler) {
        services_.push_back(resolveEndpoint(name));
        return ServiceServer(core_->addService(services_.back(), typeid(Request), typeid(Response), mailbox_,
                                               [fn = std::forward<F>(handler)](const AnyMsg& r) -> AnyMsg {
                                                   return std::make_shared<const Response>(fn(view<Request>(r)));
                                               }));
    }

    template <class T>
    Subscriber subscribeAny(std::string_view name, std::size_t queue_size, Subscription::Callback cb) {
        const std::string resolved = resolveEndpoint(name);
        subscriptions_.push_back(core_->topic<T>(resolved)->subscribe(mailbox_, queue_size, std::move(cb)));
        return Subscriber(subscriptions_.back());
    }

    RuntimeCore* core_ = nullptr;
    std::string name_;
    MailboxPtr mailbox_;
    ParamSnapshotPtr params_;
    std::atomic<std::uint64_t> hook_failures_{0};
    bool finished_ = false;
    bool stopped_ = false;
    bool initialized_ = false;
    bool started_ = false;
    ThreadOptions thread_options_;
    std::map<std::string, std::string> remaps_;

    std::vector<std::shared_ptr<Subscription>> subscriptions_;
    std::vector<TimerHandle> timers_;
    std::vector<std::string> advertised_;
    std::vector<std::string> services_;
};

}  // namespace simpleslam::comm
