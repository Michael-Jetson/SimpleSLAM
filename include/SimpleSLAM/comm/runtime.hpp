// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file runtime.hpp
/// 运行时：整个 SimpleSLAM 进程的装配根。对外相当于 ROS 的一个节点，内部一个模块一个线程。
///
/// 线程约定：装配、驱动（start / spin / spinOnce / spinFor）和 shutdown 由一个控制线程调用。
/// spin() 阻塞着控制线程时，别的线程可以调 shutdown()。
/// requestStop、setParams、publish、stats、阻塞 call 可以从任何线程调用。

#include <SimpleSLAM/comm/core.hpp>
#include <SimpleSLAM/comm/module.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace simpleslam::comm {

enum class RuntimeState : std::uint8_t { kBuilding, kReady, kRunning, kStopped, kDone };

enum class ShutdownMode : std::uint8_t {
    kFinish,  ///< 处理完已接纳的工作，让模块收尾
    kCancel,  ///< 丢弃未开始的工作，未决调用结案为 kStopped
};

struct RuntimeOptions {
    /// 离线：虚拟时钟，单线程步进。在线：单调时钟，一个模块一个线程。
    bool offline = false;
    /// 离线步进时打乱模块之间的先后，用来验证结果不依赖调度顺序。0 表示不打乱。
    std::uint64_t schedule_seed = 0;
    /// 无界订阅积压到这么多条时提醒一次，之后每翻一倍再提醒。0 表示不提醒。
    std::size_t backlog_warn = 256;
    /// 模块没有单独设置时用的异常策略。
    ExceptionPolicy on_exception = ExceptionPolicy::kIsolate;
    /// 日志组件。空表示自建一个同步的：写终端，不另开线程（单独用通信层时就是这样）。
    std::shared_ptr<logging::Router> log_router{};
};

struct ModuleStats {
    std::string name;
    MailboxStats mailbox;
};

struct RuntimeStats {
    std::vector<ModuleStats> modules;
    std::vector<SubscriptionStats> subscriptions;
};

/// 运行时自带的节点，给写不了模块的地方用：main()、工具、测试、外部桥接。
/// 用法和模块内部一样；回调同样串行，在线时在它自己的线程上，离线时由步进驱动。
class Node final : public Module {
public:
    using Module::advertise;
    using Module::advertiseService;
    using Module::createTimer;
    using Module::getParam;
    using Module::param;
    using Module::serviceClient;
    using Module::setParam;
    using Module::subscribe;
};

class Runtime {
public:
    explicit Runtime(RuntimeOptions options = {});
    ~Runtime();
    Runtime(const Runtime&) = delete;
    Runtime& operator=(const Runtime&) = delete;

    template <class M, class... Args>
    M& addModule(std::string name, Args&&... args) {
        static_assert(std::is_base_of_v<Module, M>, "模块必须派生自 comm::Module");
        if (state_.load(std::memory_order_acquire) != RuntimeState::kBuilding) {
            throw WiringError("只能在 init 之前添加模块: " + name);
        }
        if (name.empty() || name.front() != '/') name.insert(name.begin(), '/');
        auto module = std::make_unique<M>(std::forward<Args>(args)...);
        M& ref = *module;
        static_cast<Module&>(ref).attach(*core_, std::move(name));
        modules_.push_back(std::move(module));
        return ref;
    }

    void init();

    // ── 离线驱动 ──

    /// 触发到期定时器并处理就绪的收件箱，最多 budget 批。返回实际批数。
    std::size_t spinOnce(std::size_t budget = 1024);
    /// 虚拟时钟下推进一段时间，按定时器到期点逐段推进。
    std::size_t spinFor(Duration span, std::size_t budget = 1024);

    // ── 在线驱动 ──

    /// 给每个模块起一个线程，立刻返回，调用线程可以去做别的（比如 ros::spin）。通信层自身不占线程。
    void start();
    /// start() 之后阻塞到 requestStop() 且已接纳的工作走完。
    void spin();

    /// 停止输入：入口发布端不再发，服务和参数写入不再受理。已接纳的工作继续走完。
    void requestStop();

    /// requestStop() 是否已经调用过（异常策略为停机时也会触发）。
    [[nodiscard]] bool stopRequested() const noexcept { return core_->hub()->inputsStopped(); }

    /// 等系统静止：没有排队也没有执行中的工作。离线时由本线程步进，在线时等待。
    bool waitIdle(Duration timeout = Duration{0});

    /// 结束运行。模块保留为只读的最终状态，析构时才销毁。
    void shutdown(ShutdownMode mode = ShutdownMode::kFinish);

    // ── 对外的门 ──

    /// 外部线程（ROS 回调、驱动）往系统里送数据用。requestStop() 之后不再发。
    template <class T>
    Publisher<T> inputPublisher(std::string_view name) {
        return Publisher<T>(core_->topic<T>(resolveName(name, "/")), PublishOptions{false, true});
    }

    /// 运行时自带的节点，第一次用时才建。要在 init() 之前第一次调用；不用就不占线程。
    [[nodiscard]] Node& node() {
        if (node_ == nullptr) node_ = &addModule<Node>("/runtime");
        return *node_;
    }

    /// 外部线程调用服务用，只能用阻塞 call()。不需要节点。
    template <class Request, class Response>
    ServiceClient<Request, Response> serviceClient(std::string_view name) {
        return ServiceClient<Request, Response>(Module::makeCaller(
            *core_, resolveName(name, "/"), typeid(Request), typeid(Response), nullptr));
    }

    ParamCommitResult setParams(const std::vector<ParamChange>& changes) {
        return core_->setParams(changes);
    }
    [[nodiscard]] ParameterStore& params() noexcept { return core_->params(); }

    // ── 观测 ──

    [[nodiscard]] RuntimeStats stats() const;
    /// 统计表，给人看的。
    [[nodiscard]] std::string statsTable() const;
    [[nodiscard]] std::uint64_t isolatedFailures() const;
    /// 第一个被接住的异常，不论策略。
    [[nodiscard]] std::optional<ModuleFailure> firstFailure() const { return core_->hub()->firstFailure(); }
    [[nodiscard]] bool shutdownDrained() const noexcept { return drained_; }
    [[nodiscard]] bool cancelled() const noexcept { return cancelled_; }
    [[nodiscard]] RuntimeState state() const noexcept { return state_.load(std::memory_order_acquire); }

    [[nodiscard]] RuntimeCore& core() noexcept { return *core_; }
    [[nodiscard]] IClock& clock() noexcept { return core_->clock(); }
    [[nodiscard]] VirtualClock* virtualClock() noexcept { return virtual_clock_; }
    /// 离线确定时钟原点时用：全部模块的定时器后移 offset。
    void shiftTimers(Duration offset);
    void setErrorReporter(ErrorReporter reporter) { sink_->set(std::move(reporter)); }
    /// 日志组件：模块的日志对象从它来。
    [[nodiscard]] logging::Router& logRouter() noexcept { return core_->logRouter(); }

private:
    std::size_t step(std::size_t budget, bool fire_timers = true);
    bool drainRemaining();
    void finishRun();
    [[nodiscard]] std::optional<Time> nextTimerDue() const;
    void rethrowFatal();

    std::shared_ptr<detail::ReportSink> sink_ = std::make_shared<detail::ReportSink>();
    VirtualClock* virtual_clock_ = nullptr;
    std::shared_ptr<RuntimeCore> core_;
    std::vector<std::unique_ptr<Module>> modules_;
    Node* node_ = nullptr;

    std::atomic<RuntimeState> state_{RuntimeState::kBuilding};
    std::mutex control_mutex_;
    std::condition_variable control_cv_;
    bool stop_requested_ = false;
    bool finishing_ = false;
    bool modules_started_ = false;  ///< 离线：只在控制线程上读写
    std::vector<std::thread> threads_;
    std::vector<TimerQueue*> timer_queues_;  ///< init() 时定下

    bool drained_ = true;
    bool cancelled_ = false;
};

}  // namespace simpleslam::comm
