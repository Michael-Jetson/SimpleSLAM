// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file runtime.hpp
/// 运行时：显式持有的装配根，没有进程级单例。
///
/// 同一进程可以有两个互不干扰的 Runtime，使用相同话题名也不串。
/// 在线用 spin() 驱动，离线用 spinOnce() / spinFor() 步进同一套模块代码；
/// 离线的定时器由虚拟时钟推进，不依赖墙钟等待。
///
/// ## 生命周期协议
///
/// 停止、静止、销毁是三件事，必须按序发生，不能合并成一个 shutdown()：
///
///   kBuilding → kInitialized → kRunning → kStopping → kQuiesced → kTornDown
///
/// - **停止**（requestStop）：关闭全部话题的接纳口与服务端，让 spin() 返回。
///   已接纳的消息仍会被处理完——"不再接新输入"不等于"丢掉手上的活"。
/// - **静止**（quiesce）：先**封住外部执行入口**，再等待已有参与者退出。
///   两步顺序不能反，也不能分开：等待已有参与者退出 ≠ 禁止新参与者进入。
///   只做后者的话，一个刚好在静止之后调用 spinOnce() 的线程会拿到执行权，
///   而关闭线程已经在析构模块了。因此"查状态 + 登记参与者"
///   是同一个受锁保护的操作。参与者包括工作线程、定时器线程，以及调用
///   spin()/spinOnce() 的那个外部线程——strand 保证回调不并发，不保证回调已经退出。
/// - **收尾**：静止之后把已接纳的工作排空、让模块收尾。收尾完成后模块**保留为
///   只读的最终状态**：执行入口已封、话题与服务已关、收件箱已关、定时器已撤，
///   不会再有任何业务回调，读它们的结果不再变化，也没有寿命问题。
/// - **销毁**：Runtime 析构时才逆序销毁模块，且不再重复收尾。在静止之前析构模块，
///   就是在仍在执行的回调脚下抽掉对象；在收尾时当场销毁，则让"关闭之后读最终结果"
///   变成读已释放的内存。
///
/// shutdown() 不得在运行时线程内调用：它要等待的正是调用者自己。

#include <SimpleSLAM/comm/core.hpp>
#include <SimpleSLAM/comm/module.hpp>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace simpleslam::comm {

/// 运行时所处的阶段。对外可读，便于模块与测试断言"现在允许做什么"。
enum class RuntimeState : std::uint8_t {
    kBuilding,     ///< 正在装配模块，业务回调还没开始
    kInitialized,  ///< init() 已完成，可以驱动
    kRunning,      ///< spin() 正在执行
    kStopping,     ///< 已停输入，正在排空已接纳的工作
    kQuiesced,     ///< 所有参与线程已退出
    kTornDown,     ///< 已排空并收尾：模块保留为只读的最终状态，析构时才销毁
};

/// 两种结束方式。
enum class ShutdownMode : std::uint8_t {
    /// 完成已接纳工作：排空；让模块主动核对补读游标、结案能结案的操作（Module::finish）；
    /// 再排空，然后收尾。正常结束一次运行用它。
    kFinish,
    /// 取消运行：停止新工作，不再排空。收件箱关闭时丢弃的未决操作随之明确结案
    /// （服务调用结案为"已停止"），结果如实标为取消，不伪装成完成。
    kCancel,
};

struct RuntimeOptions {
    /// 在线工作线程数。spin() 会额外借用调用线程，所以总并发是 worker_threads。
    std::size_t worker_threads = 2;
    /// 用虚拟时钟（离线步进）还是单调时钟（在线）。
    bool virtual_clock = false;
};

class Runtime {
public:
    /// 等待静止时每隔多久上报一次"还没静止"。不设上限：宁可挂住并被看见，
    /// 也不要超时之后照常析构——后者把可诊断的停顿换成不可诊断的内存错误。
    static constexpr Duration kQuiesceReportInterval = std::chrono::seconds(2);

    explicit Runtime(RuntimeOptions options = {}) : options_(options) {
        if (options_.virtual_clock) {
            auto vc = std::make_unique<VirtualClock>();
            virtual_clock_ = vc.get();
            clock_ = std::move(vc);
        } else {
            clock_ = std::make_unique<SteadyClock>();
        }
        // 交给内核与各个句柄的是共享的上报出口，不是捕获 this 的闭包：句柄可能比
        // Runtime 活得久，Runtime 析构时清空出口，旧句柄的上报就安全地落空
        reporter_ = [sink = report_sink_](std::string_view where, std::string_view what) {
            sink->report(where, what);
        };
        core_ = std::make_shared<RuntimeCore>(io_, *clock_, timers_, params_, reporter_);
        // 插入更早的 deadline 时叫醒在线定时器循环：不然它会一直睡到旧的 deadline。
        // waker 在队列锁外被调用。
        timers_->setWaker([this] {
            {
                std::lock_guard lock(timer_mutex_);
            }
            timer_cv_.notify_all();
        });
    }

    /// 析构：还没关闭就先按协议关闭；已经关闭则不再重复收尾，只逆序销毁模块。
    ~Runtime() {
        shutdown();
        // 模块析构是用户代码：先在锁内摘下整张表，再在锁外逆序析构
        std::vector<std::unique_ptr<Module>> doomed;
        {
            std::lock_guard lock(modules_mutex_);
            doomed.swap(modules_);
        }
        // 收件箱可能被比 Runtime 活得久的句柄引用着，strand 却不能比 io_context 活得久：
        // 先释放每个收件箱的执行底座，此后那些句柄碰到的只是一个关着的收件箱
        for (const auto& module : doomed) {
            if (module->mailbox()) module->mailbox()->executor().release();
        }
        while (!doomed.empty()) doomed.pop_back();
        // 比 Runtime 活得久的句柄从此失效：唤醒出口捕获着 this，上报出口指向用户的回调
        timers_->setWaker({});
        report_sink_->clear();
    }

    Runtime(const Runtime&) = delete;
    Runtime& operator=(const Runtime&) = delete;

    /// 装配一个模块。名字必须是绝对名，缺前导 / 时自动补上。
    ///
    /// 只在装配阶段（init() 之前）允许。运行起来之后再加模块，它的订阅会错过已经
    /// 发生的一切，而 shutdown() 遍历模块表时也不能有人在改它——入口直接拒绝，
    /// 不靠调用约定。
    template <class M, class... Args>
    M& addModule(std::string name, Args&&... args) {
        static_assert(std::is_base_of_v<Module, M>, "模块必须派生自 comm::Module");
        if (state_.load(std::memory_order_acquire) != RuntimeState::kBuilding) {
            throw WiringError("只能在装配阶段（init 之前）添加模块: " + name);
        }
        if (name.empty() || name.front() != '/') name.insert(name.begin(), '/');
        auto module = std::make_unique<M>(std::forward<Args>(args)...);
        M& ref = *module;
        ref.attach(*core_, std::move(name));
        std::lock_guard lock(modules_mutex_);
        modules_.push_back(std::move(module));
        return ref;
    }

    /// 全部模块装配完成后再开始跑业务回调：onInit 只做登记。只能调用一次。
    void init() {
        if (state_.load(std::memory_order_acquire) != RuntimeState::kBuilding) {
            throw WiringError("init() 只能在装配阶段调用一次");
        }
        for (auto& m : modules_) m->initialize();
        state_.store(RuntimeState::kInitialized, std::memory_order_release);
    }

    // ── 离线驱动 ──

    // 三种驱动方式（spin / spinOnce / spinFor）彼此互斥：同一时刻只能有一个外部线程
    // 在驱动。io_context 的 restart() 不能与尚未退出的 run()/poll() 并发，两个线程同时
    // 步进也说不清谁的定时器先触发。互斥由 step_mutex_ 保证，撞上了就在入口报错，
    // 而不是悄悄排队。

    /// 触发到期定时器并处理就绪任务，最多 budget 条。返回实际处理条数。
    std::size_t spinOnce(std::size_t budget = 1024) {
        Participation joined(*this);
        if (!joined.admitted()) return 0;  // 入口已封：关闭正在进行，不再接纳新执行者
        std::unique_lock step(step_mutex_, std::try_to_lock);
        if (!step.owns_lock()) {
            throw WiringError("spinOnce() 不可与 spin()、spinFor() 或另一个 spinOnce() 并发");
        }
        return stepLocked(budget);
    }

    /// 虚拟时钟下推进一段时间：按定时器到期点逐段推进，期间处理就绪任务。
    ///
    /// 整次调用在同一次准入之下；关闭一旦开始就停止推进并返回。每一步都在到期点
    /// 触发全部到期的定时器（一次性的被移除、周期的推到下一个到期点），所以最早到期
    /// 时刻严格前进，循环必然走到目标时刻。
    std::size_t spinFor(Duration span, std::size_t budget = 1024) {
        Participation joined(*this);
        if (!joined.admitted()) return 0;
        std::unique_lock step(step_mutex_, std::try_to_lock);
        if (!step.owns_lock()) {
            throw WiringError("spinFor() 不可与 spin()、spinOnce() 或另一个 spinFor() 并发");
        }
        std::size_t total = stepLocked(budget);
        if (virtual_clock_ == nullptr) return total;
        const Time target = clock_->now() + span;
        while (!entryClosed()) {
            const auto next = timers_->nextDue();
            if (!next || *next > target) break;
            virtual_clock_->advanceTo(*next);
            total += stepLocked(budget);
        }
        if (!entryClosed()) {
            virtual_clock_->advanceTo(target);
            total += stepLocked(budget);
        }
        return total;
    }

    // ── 在线驱动 ──

    /// 阻塞直到 requestStop()。调用线程也参与执行，因此它也是静止的等待对象。
    ///
    /// 整个调用——从准入判断到最后一个工作线程退出——都在同一次 Participation 之下。
    /// 只登记 io_.run() 那一小段是不够的：启动阶段（改状态、建 work guard、拉线程）
    /// 会裸露在协议之外，关闭可以插进"查过入口没封"与"登记参与者"之间，
    /// 于是 waitUntilQuiesced() 看到零参与者就判定静止，
    /// 而 spin() 随后还会把状态写回 kRunning 并拉起线程。
    ///
    /// 启动与停止之间由 drive_mutex_ 串行化，work_ 的建立与释放也归它管。
    void spin() {
        Participation joined(*this);  // 准入与登记是同一个受锁操作
        if (!joined.admitted()) return;
        std::unique_lock step(step_mutex_, std::try_to_lock);
        if (!step.owns_lock()) {
            throw WiringError("spin() 不可与 spinOnce()、spinFor() 或另一个 spin() 并发");
        }

        bool started = false;
        {
            std::lock_guard drive(drive_mutex_);
            if (state_.load(std::memory_order_acquire) >= RuntimeState::kStopping) return;
            bool expected = false;
            if (!spinning_.compare_exchange_strong(expected, true)) {
                throw WiringError("spin() 不可重入，也不可由两个线程同时调用");
            }
            started = true;
            // 之前用 spinOnce() 步进过的话，io_context 已经因为跑空而 stopped，
            // 不 restart 的话 io_.run() 会立刻返回零处理数
            if (io_.stopped()) io_.restart();
            stop_requested_.store(false, std::memory_order_release);
            state_.store(RuntimeState::kRunning, std::memory_order_release);
            work_.emplace(net::make_work_guard(io_));

            std::lock_guard lock(threads_mutex_);
            timer_thread_ = std::thread([this] { timerLoop(); });
            const std::size_t extra = options_.worker_threads > 1 ? options_.worker_threads - 1 : 0;
            workers_.reserve(extra);
            for (std::size_t i = 0; i < extra; ++i) {
                workers_.emplace_back([this] {
                    Participation worker_joined(*this);
                    if (!worker_joined.admitted()) return;
                    detail::RuntimeThreadMark mark;
                    io_.run();
                });
            }
        }

        try {
            detail::RuntimeThreadMark mark;
            io_.run();
        } catch (...) {
            joinThreads();
            if (started) spinning_.store(false, std::memory_order_release);
            throw;
        }
        joinThreads();
        if (started) spinning_.store(false, std::memory_order_release);
    }

    /// 停止接纳新**外部输入**并让 spin() 返回。已接纳的输入会把因果链走完。
    ///
    /// 这是"停输入"的第一段：只关入口话题与服务端，派生话题留着。于是一条已经被
    /// 里程计收下的包，仍然能产出关键帧、进图、进轨迹——最终图与轨迹是完整的。
    /// 第二段（关掉其余话题）在 shutdown() 里、因果链排空之后才做。
    ///
    /// 没有任何话题声明自己是入口时退化为一次性全关，与旧行为一致。
    void requestStop() {
        std::lock_guard drive(drive_mutex_);
        auto expected = state_.load(std::memory_order_acquire);
        while (expected < RuntimeState::kStopping
               && !state_.compare_exchange_weak(expected, RuntimeState::kStopping,
                                                std::memory_order_acq_rel)) {
        }
        stop_requested_.store(true, std::memory_order_release);
        {
            // 参数入口与发布路径同一把锁：此后到来的提交一律拒绝，已经过了入口检查的
            // 那一次整段做完（它的快照照常排进收件箱，按关闭协议被排空生效）
            std::lock_guard params(param_publish_mutex_);
            params_closed_ = true;
        }
        if (core_) {
            core_->stopInputs();
            const bool has_ingress = core_->ingressTopicCount() > 0;
            core_->closeTopics(has_ingress ? TopicCloseScope::kIngress : TopicCloseScope::kAll);
            core_->closeServices();
        }
        {
            std::lock_guard lock(timer_mutex_);
        }
        timer_cv_.notify_all();
        work_.reset();
    }

    /// 等待所有参与执行的线程退出运行时。返回是否已静止。
    ///
    /// 参与者 = 工作线程 + 调用 spin()/spinOnce() 的外部线程。
    /// timeout 为零表示一直等；期间按固定间隔上报，便于定位是谁没退出。
    bool waitUntilQuiesced(Duration timeout = Duration{0}) {
        if (inRuntimeThread()) {
            detail::reportSafely(reporter_, "Runtime",
                                 "不得在运行时线程内等待静止：等待的正是调用者自己");
            return false;
        }
        const auto wait_span =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(kQuiesceReportInterval);
        const auto deadline =
            std::chrono::steady_clock::now()
            + std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeout);
        std::unique_lock lock(quiesce_mutex_);
        for (;;) {
            const auto idle = [this] {
                return participants_.load(std::memory_order_acquire) == 0;
            };
            if (idle()) return true;
            if (quiesce_cv_.wait_for(lock, wait_span, idle)) return true;
            if (timeout.count() > 0 && std::chrono::steady_clock::now() >= deadline) return false;
            detail::reportSafely(reporter_, "Runtime",
                                 "等待静止超过上报间隔：仍有线程在运行时内执行");
        }
    }

    /// 关闭协议：停输入 → 等静止 →（完成已接纳工作）→ 让模块收尾。模块在析构时才销毁。
    ///
    /// 不在运行时线程内调用——那会变成等待自己退出。重复调用是空操作。
    void shutdown(ShutdownMode mode = ShutdownMode::kFinish) {
        if (state_.load(std::memory_order_acquire) == RuntimeState::kTornDown) return;
        if (inRuntimeThread()) {
            detail::reportSafely(reporter_, "Runtime",
                                 "不得在运行时线程内调用 shutdown()：已改为只停输入");
            requestStop();
            return;
        }
        std::lock_guard shutdown_lock(shutdown_mutex_);
        if (state_.load(std::memory_order_acquire) == RuntimeState::kTornDown) return;

        requestStop();     // 第一段：只关入口话题，派生链路留着
        if (mode == ShutdownMode::kCancel) {
            // 必须在等待在线 run() 退出之前拒绝未开始的工作，否则 run() 会先把它们跑完。
            // 收件箱关闭与取出下一格用同一把锁：已取得执行权的一格可结束，其余丢弃结案。
            std::vector<MailboxPtr> mailboxes;
            {
                std::lock_guard lock(modules_mutex_);
                for (const auto& m : modules_) mailboxes.push_back(m->mailbox());
            }
            // 被丢弃任务的析构可能重入 Runtime；不持模块表锁，并禁止同步等待自己。
            detail::RuntimeThreadMark mark;
            for (const auto& mailbox : mailboxes) mailbox->close();
        }
        closeEntry();      // 先封执行入口，再等已有参与者——顺序不能反
        waitUntilQuiesced();
        joinThreads();
        state_.store(RuntimeState::kQuiesced, std::memory_order_release);

        // 此刻只有本线程在跑。先把已接纳输入的整条因果链走完（派生话题还开着），
        // 再做第二段关闭，最后让模块收尾。顺序反过来的话，关键帧会因为下游话题
        // 已关而被拒，最终图和轨迹静悄悄地少一截。
        //
        // 排空有轮数上限（防止自我续命的任务把关闭挂死），但用完了预算必须可见：
        // "承诺把因果链走完"而实际没走完，下游的结论就建立在残缺的产物上。
        std::lock_guard step(step_mutex_);
        bool drained = true;
        if (mode == ShutdownMode::kFinish) {
            drained = drainRemaining();
            // 完成已接纳工作：模块主动核对补读游标，它们发出的消息接着排空。
            // 结案不能只靠下一条业务消息——末条通知丢了、之后又没有输入时它不会来
            for (auto& m : modules_) m->finish();
            drained = drainRemaining() && drained;
        }
        if (core_) core_->closeTopics(TopicCloseScope::kAll);
        if (mode == ShutdownMode::kFinish) drained = drainRemaining() && drained;
        for (auto& m : modules_) m->shutdown();
        // 取消时收件箱已关，这一步只是让排队的空任务退出，不执行任何业务
        const bool flushed = drainRemaining();
        if (mode == ShutdownMode::kFinish) {
            drained = flushed && drained;
        } else {
            drained = false;  // 取消就是没有排空：未决工作被明确放弃了
            cancelled_.store(true, std::memory_order_release);
        }
        shutdown_drained_.store(drained, std::memory_order_release);
        if (!drained && mode == ShutdownMode::kFinish) {
            detail::reportSafely(reporter_, "Runtime", "关闭时排空预算用尽：仍有未处理的工作");
        }

        state_.store(RuntimeState::kTornDown, std::memory_order_release);
        // 模块不在这里销毁：它们是这次运行的最终状态，关闭之后还要被读。
        // 执行入口已封、收件箱已关，此后没有任何回调会再改它们；析构时才销毁。
    }

    /// 上一次 shutdown() 是否把已接纳的工作全部排空。false 表示排空预算用尽或运行被取消，
    /// 最终产物可能不完整。
    [[nodiscard]] bool shutdownDrained() const noexcept {
        return shutdown_drained_.load(std::memory_order_acquire);
    }
    /// 运行是否以取消结束。
    [[nodiscard]] bool cancelled() const noexcept {
        return cancelled_.load(std::memory_order_acquire);
    }

    // ── 参数 ──

    [[nodiscard]] ParameterStore& params() noexcept { return params_; }

    /// 提交参数事务，成功后把新快照排进每个模块的收件箱（与消息同一条 FIFO，
    /// 因此"这一帧看到的是哪一代参数"是确定的）。
    ///
    /// 整条发布路径由 param_publish_mutex_ 串行化，分发的是 commit() 带回的**那一份**
    /// 快照而不是事后再取的最新快照：两者不是一回事，并发提交可以挤在中间，
    /// 于是返回的版本号与模块实际收到的快照对不上，甚至先投新版再投旧版
    /// 。模块侧另有版本单调检查兜底。
    ///
    /// 送不到的模块记在 result.undelivered 里——参数已进真源但那些模块还用着旧一代，
    /// 这件事必须可见。
    ///
    /// **停止即拒绝**：requestStop() 生效起，新的提交一律返回 kRejectedStopped，参数
    /// 存储不变——参数与服务同属控制输入，停输入就一起停。入口检查与发布路径在同一把
    /// 锁下：已经过了检查的那一次提交整段做完，它的快照照常排进收件箱，按关闭协议
    /// 在排空时生效；不存在"检查时没停、提交时已停"的中间态。
    ParamCommitResult setParams(const std::vector<ParamChange>& changes,
                                std::optional<std::uint64_t> expected_revision = std::nullopt) {
        const auto rejected = [this] {
            ParamCommitResult stopped;
            stopped.status = ParamCommit::kRejectedStopped;
            stopped.revision = params_.revision();
            stopped.detail = "运行时已停止输入，不再接受参数写入";
            return stopped;
        };
        Participation joined(*this);
        if (!joined.admitted()) return rejected();
        std::lock_guard publish_lock(param_publish_mutex_);
        if (params_closed_) return rejected();
        auto result = params_.commit(changes, expected_revision);
        if (!result.ok()) return result;
        auto snapshot = result.snapshot;
        {
            std::lock_guard modules(modules_mutex_);
            for (auto& m : modules_) {
                Module* raw = m.get();
                const bool queued =
                    raw->mailbox()->postTask([raw, snapshot] { raw->adoptParams(snapshot); });
                if (!queued) result.undelivered.push_back(raw->name());
            }
        }
        paramUpdatePublisher().publish(ParamUpdate{result.revision});
        return result;
    }

    /// 某一版参数是否已经**生效**：每个模块都已在帧边界换上了不早于它的快照。
    ///
    /// 与 ParamCommitResult::fullyQueued() 不同——那只说明新快照排进了收件箱。
    /// 可以从任何线程查：读的是各模块单独发布的原子版本号；模块表由锁保护，
    /// 且模块在 Runtime 析构之前不会销毁。
    [[nodiscard]] bool paramsApplied(std::uint64_t revision) const {
        std::lock_guard lock(modules_mutex_);
        for (const auto& m : modules_) {
            if (m->appliedParamRevision() < revision) return false;
        }
        return true;
    }

    /// 全部模块的回调里被执行边界接住的异常总数。每一次都是一份没做完的工作，
    /// 运行结局据此判断处理是否完整。任何线程可查，关闭之后仍可查。
    [[nodiscard]] std::uint64_t isolatedFailures() const {
        std::lock_guard lock(modules_mutex_);
        std::uint64_t total = 0;
        for (const auto& m : modules_) total += m->isolatedFailures();
        return total;
    }

    // ── 访问器 ──

    [[nodiscard]] RuntimeCore& core() noexcept { return *core_; }
    [[nodiscard]] IClock& clock() noexcept { return *clock_; }
    [[nodiscard]] VirtualClock* virtualClock() noexcept { return virtual_clock_; }
    [[nodiscard]] TimerQueue& timers() noexcept { return *timers_; }
    [[nodiscard]] std::size_t moduleCount() const {
        std::lock_guard lock(modules_mutex_);
        return modules_.size();
    }
    [[nodiscard]] RuntimeState state() const noexcept {
        return state_.load(std::memory_order_acquire);
    }
    [[nodiscard]] bool initialized() const noexcept {
        return state() >= RuntimeState::kInitialized;
    }
    [[nodiscard]] std::size_t participants() const noexcept {
        return participants_.load(std::memory_order_acquire);
    }
    /// 外部执行入口是否已封（关闭进行中）。
    [[nodiscard]] bool executionClosed() const { return entryClosed(); }

    void setErrorReporter(ErrorReporter reporter) { report_sink_->set(std::move(reporter)); }

    [[nodiscard]] std::vector<std::string> danglingTopics() const { return core_->danglingTopics(); }

private:
    /// 参与执行的 RAII 准入标记。
    ///
    /// "查入口是否已封 + 登记参与者"必须是同一个受锁保护的操作：分成两步，
    /// 就存在"查的时候没封、登记完才封"的窗口，关闭线程会在这个窗口里
    /// 认为已经静止并开始析构模块。
    class Participation {
    public:
        explicit Participation(Runtime& runtime) {
            std::lock_guard lock(runtime.quiesce_mutex_);
            if (runtime.entry_closed_) return;  // 未获准入，runtime_ 保持空
            runtime_ = &runtime;
            runtime_->participants_.fetch_add(1, std::memory_order_acq_rel);
        }
        ~Participation() {
            if (runtime_ == nullptr) return;
            bool last = false;
            {
                std::lock_guard lock(runtime_->quiesce_mutex_);
                last = runtime_->participants_.fetch_sub(1, std::memory_order_acq_rel) == 1;
            }
            if (last) runtime_->quiesce_cv_.notify_all();
        }
        [[nodiscard]] bool admitted() const noexcept { return runtime_ != nullptr; }

        Participation(const Participation&) = delete;
        Participation& operator=(const Participation&) = delete;

    private:
        Runtime* runtime_ = nullptr;
    };

    void closeEntry() {
        std::lock_guard lock(quiesce_mutex_);
        entry_closed_ = true;
    }
    [[nodiscard]] bool entryClosed() const {
        std::lock_guard lock(quiesce_mutex_);
        return entry_closed_;
    }

    Publisher<ParamUpdate>& paramUpdatePublisher() {
        if (!param_pub_.valid()) {
            param_pub_ = Publisher<ParamUpdate>(core_->topic<ParamUpdate>("/parameter_updates"),
                                                PublishOptions{true});
        }
        return param_pub_;
    }

    /// 只在已静止之后调用：把已接纳的工作跑完，不再接纳新的。
    /// 返回是否真的排空了（最后一轮没有任何工作）。预算用尽返回 false。
    bool drainRemaining(std::size_t max_rounds = 256) {
        detail::RuntimeThreadMark mark;
        for (std::size_t i = 0; i < max_rounds; ++i) {
            if (io_.stopped()) io_.restart();
            std::size_t done = 0;
            while (done < 4096) {
                const std::size_t n = io_.poll_one();
                if (n == 0) break;
                done += n;
            }
            if (done == 0) return true;
        }
        return false;
    }

    /// 一步离线推进的本体：调用方已持有准入与 step_mutex_。
    std::size_t stepLocked(std::size_t budget) {
        detail::RuntimeThreadMark mark;
        if (io_.stopped()) io_.restart();
        timers_->fireDue(clock_->now());
        std::size_t done = 0;
        while (done < budget) {
            const std::size_t n = io_.poll_one();
            if (n == 0) break;
            done += n;
        }
        return done;
    }

    void timerLoop() {
        Participation joined(*this);
        if (!joined.admitted()) return;  // 与其他执行入口服从同一个准入协议
        std::unique_lock lock(timer_mutex_);
        while (!stop_requested_.load(std::memory_order_acquire)) {
            const auto next = timers_->nextDue();
            const Time now = clock_->now();
            if (!next) {
                timer_cv_.wait_for(lock, std::chrono::milliseconds(20));
                continue;
            }
            if (*next <= now) {
                lock.unlock();
                timers_->fireDue(now);
                lock.lock();
                continue;
            }
            timer_cv_.wait_for(lock, *next - now);
        }
    }

    void joinThreads() {
        std::lock_guard lock(threads_mutex_);
        if (timer_thread_.joinable()) {
            stop_requested_.store(true, std::memory_order_release);
            timer_cv_.notify_all();
            timer_thread_.join();
        }
        for (auto& t : workers_) {
            if (t.joinable()) t.join();
        }
        workers_.clear();
    }

    RuntimeOptions options_;
    IoContext io_;
    std::unique_ptr<IClock> clock_;
    VirtualClock* virtual_clock_ = nullptr;
    /// 共享持有：TimerHandle 与服务调用端只弱引用它，Runtime 销毁之后它们安全地失效。
    std::shared_ptr<TimerQueue> timers_ = std::make_shared<TimerQueue>();
    ParameterStore params_;
    std::shared_ptr<detail::ReportSink> report_sink_ = std::make_shared<detail::ReportSink>();
    /// 运行时自己用的上报出口：经共享出口转给用户设置的回调。
    ErrorReporter reporter_;
    std::shared_ptr<RuntimeCore> core_;
    /// 模块表。装配期追加，Runtime 析构时逆序销毁；其间结构不变。锁只保护
    /// "追加 / 销毁"与跨线程遍历之间的并发，不在锁内调用模块的业务代码。
    std::vector<std::unique_ptr<Module>> modules_;
    mutable std::mutex modules_mutex_;
    Publisher<ParamUpdate> param_pub_;

    std::optional<net::executor_work_guard<IoContext::executor_type>> work_;
    std::vector<std::thread> workers_;
    std::thread timer_thread_;
    std::mutex threads_mutex_;
    std::mutex shutdown_mutex_;
    std::mutex timer_mutex_;
    std::condition_variable timer_cv_;

    /// 串行化"启动驱动"与"请求停止"：两者都要动 state_ / work_ / 线程表。
    std::mutex drive_mutex_;
    /// 驱动方式互斥：spin / spinOnce / spinFor 与关闭时的排空，同一时刻只能有一个。
    std::mutex step_mutex_;
    std::atomic<bool> shutdown_drained_{true};
    std::atomic<bool> cancelled_{false};
    /// 串行化参数发布路径：入口检查、提交、取本次快照、逐模块分发必须是一串。
    std::mutex param_publish_mutex_;
    /// requestStop() 之后不再接受参数写入。由 param_publish_mutex_ 保护。
    bool params_closed_ = false;
    mutable std::mutex quiesce_mutex_;
    std::condition_variable quiesce_cv_;
    std::atomic<std::size_t> participants_{0};
    /// 外部执行入口是否已封。只有 shutdown() 会置它——停输入与停执行是两件事。
    bool entry_closed_ = false;

    std::atomic<RuntimeState> state_{RuntimeState::kBuilding};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> spinning_{false};
};

}  // namespace simpleslam::comm
