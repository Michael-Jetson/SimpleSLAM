// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/comm/runtime.hpp>

#include <algorithm>
#include <cstdio>
#include <thread>
#include <utility>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace simpleslam::comm {

namespace {

/// 线程名用模块名的最后一段，在 top -H 里看得出是哪个模块。
void setUpThread(const std::string& module, const ThreadOptions& options, const ErrorReporter& reporter) {
#if defined(__linux__)
    const auto slash = module.rfind('/');
    pthread_setname_np(pthread_self(), module.substr(slash == std::string::npos ? 0 : slash + 1, 15).c_str());
    if (options.nice != 0 && setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), options.nice) != 0) {
        detail::reportSafely(reporter, module, "设置线程优先级失败");
    }
    if (!options.cpus.empty()) {
        cpu_set_t set;
        CPU_ZERO(&set);
        for (const int cpu : options.cpus) CPU_SET(cpu, &set);
        if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
            detail::reportSafely(reporter, module, "绑定线程到核失败");
        }
    }
#else
    (void)module;
    (void)options;
    (void)reporter;
#endif
}

}  // namespace

Runtime::Runtime(RuntimeOptions options) {
    std::unique_ptr<IClock> clock;
    if (options.offline) {
        auto vc = std::make_unique<VirtualClock>();
        virtual_clock_ = vc.get();
        clock = std::move(vc);
    } else {
        clock = std::make_unique<SteadyClock>();
    }
    ErrorReporter reporter = [sink = sink_](std::string_view where, std::string_view what) {
        sink->report(where, what);
    };
    core_ = std::make_shared<RuntimeCore>(std::move(clock), std::move(reporter), options.backlog_warn,
                                          options.on_exception, options.log_router);
    core_->hub()->setStopHook([this] { requestStop(); });
    core_->hub()->setScheduleSeed(options.schedule_seed);
    core_->setParamBroadcast([this](const ParamSnapshotPtr& snapshot) {
        for (const auto& m : modules_) {
            m->mailbox()->post([raw = m.get(), snapshot] { raw->adoptParams(snapshot); });
        }
    });
}

Runtime::~Runtime() {
    shutdown();
    core_->setParamBroadcast({});
    core_->hub()->setStopHook({});
    core_->hub()->clearReady();
    while (!modules_.empty()) modules_.pop_back();
    sink_->set({});
}

void Runtime::init() {
    if (state() != RuntimeState::kBuilding) throw WiringError("init() 只能调用一次");
    for (auto& m : modules_) timer_queues_.push_back(&m->mailbox()->timers());
    for (auto& m : modules_) m->initialize();
    // 接线校验：一个进程里全部模块都已登记，这里就能查出来，不用等运行时才发现收不到数据
    std::string missing;
    for (const auto& m : modules_) {
        for (const auto& sub : m->subscriptions_) {
            if (sub->required() && core_->publisherCount(sub->topicName()) == 0) {
                missing += "\n  " + m->name() + " 需要话题 " + sub->topicName() + "，但没有发布者";
            }
        }
    }
    if (!missing.empty()) throw WiringError("接线不完整：" + missing);
    state_.store(RuntimeState::kReady, std::memory_order_release);
}

std::size_t Runtime::step(std::size_t budget, bool fire_timers) {
    detail::RuntimeThreadMark mark;
    if (!modules_started_ && state() != RuntimeState::kBuilding) {
        modules_started_ = true;
        for (const auto& m : modules_) m->start();
    }
    if (fire_timers) fireDueTimers(timer_queues_, core_->clock().now());
    std::size_t done = 0;
    while (done < budget) {
        auto box = core_->hub()->popReady();
        if (!box) break;
        box->drainSome();
        ++done;
    }
    return done;
}

std::size_t Runtime::spinOnce(std::size_t budget) {
    if (state() >= RuntimeState::kDone) return 0;
    if (state() == RuntimeState::kRunning) throw WiringError("spin() 运行期间不能步进");
    const std::size_t done = step(budget);
    rethrowFatal();
    return done;
}

std::size_t Runtime::spinFor(Duration span, std::size_t budget) {
    std::size_t total = spinOnce(budget);
    if (virtual_clock_ == nullptr || state() >= RuntimeState::kDone) return total;
    const Time target = virtual_clock_->now() + span;
    for (auto next = nextTimerDue(); next && *next <= target; next = nextTimerDue()) {
        virtual_clock_->advanceTo(*next);
        total += step(budget);
        rethrowFatal();
    }
    virtual_clock_->advanceTo(target);
    total += step(budget);
    rethrowFatal();
    return total;
}

std::optional<Time> Runtime::nextTimerDue() const {
    std::optional<Time> earliest;
    for (auto* queue : timer_queues_) {
        const auto next = queue->nextDue();
        if (next && (!earliest || *next < *earliest)) earliest = next;
    }
    return earliest;
}

void Runtime::shiftTimers(Duration offset) {
    for (auto* queue : timer_queues_) queue->shiftAll(offset);
}

void Runtime::start() {
    // 起线程也在锁内：spin() 阻塞着控制线程时，别的线程可能马上 shutdown()，它要等线程都起完才能回收
    std::lock_guard lock(control_mutex_);
    if (stop_requested_) return;
    if (state() != RuntimeState::kReady) throw WiringError("start() 要在 init() 之后调用，且只能调用一次");
    state_.store(RuntimeState::kRunning, std::memory_order_release);
    core_->hub()->clearReady();
    threads_.reserve(modules_.size());
    for (const auto& m : modules_) {
        threads_.emplace_back([module = m.get(), box = m->mailbox(), core = core_] {
            setUpThread(box->owner(), module->thread_options_, core->reporter());
            detail::RuntimeThreadMark mark;
            module->start();
            box->threadLoop(core->clock());
        });
    }
}

void Runtime::spin() {
    start();
    finishRun();
    rethrowFatal();
}

/// 等停止，等已接纳的工作走完，收回模块线程。可以由 spin() 或 shutdown() 进入，只做一次。
void Runtime::finishRun() {
    {
        std::unique_lock lock(control_mutex_);
        if (state() != RuntimeState::kRunning) return;
        control_cv_.wait(lock, [this] { return stop_requested_; });
        if (finishing_) {
            control_cv_.wait(lock, [this] { return state() != RuntimeState::kRunning; });
            return;
        }
        finishing_ = true;
    }
    // 输入已停、定时器不再触发，等已接纳的工作把因果链走完。不设上限：宁可挂住并被看见
    while (!core_->hub()->waitIdle(std::chrono::seconds(2))) {
        detail::reportSafely(core_->reporter(), "Runtime", "等待静止超过 2 秒：仍有模块在处理");
    }
    for (const auto& m : modules_) m->mailbox()->stopThread();
    for (auto& t : threads_) t.join();
    threads_.clear();
    {
        std::lock_guard lock(control_mutex_);
        state_.store(RuntimeState::kStopped, std::memory_order_release);
    }
    control_cv_.notify_all();
}

void Runtime::rethrowFatal() {
    if (auto cause = core_->hub()->takeFatal()) std::rethrow_exception(cause);
}

void Runtime::requestStop() {
    core_->hub()->stopInputs();
    {
        std::lock_guard lock(control_mutex_);
        stop_requested_ = true;
    }
    control_cv_.notify_all();
}

bool Runtime::waitIdle(Duration timeout) {
    if (inRuntimeThread()) return false;
    if (virtual_clock_ == nullptr) return core_->hub()->waitIdle(timeout);
    while (step(4096) != 0) {
    }
    rethrowFatal();
    return core_->hub()->idle();
}

bool Runtime::drainRemaining() {
    for (std::size_t i = 0; i < 256; ++i) {
        if (step(4096, false) == 0) return true;  // 收尾时定时器不再触发
    }
    return false;
}

void Runtime::shutdown(ShutdownMode mode) {
    if (state() == RuntimeState::kDone) return;
    if (inRuntimeThread()) {
        detail::reportSafely(core_->reporter(), "Runtime", "不得在运行时线程内调用 shutdown()：已改为只停输入");
        requestStop();
        return;
    }
    requestStop();
    if (mode == ShutdownMode::kCancel) {
        cancelled_ = true;
        for (const auto& m : modules_) m->mailbox()->closeAdmission();
        for (const auto& m : modules_) m->mailbox()->releaseDiscarded();
    }
    finishRun();

    // 此刻只有本线程在跑
    bool drained = !cancelled_;
    if (mode == ShutdownMode::kFinish) {
        drained = drainRemaining();
        for (auto& m : modules_) m->finish();
        drained = drainRemaining() && drained;
    }
    for (auto& m : modules_) m->shutdown();
    core_->hub()->clearReady();
    drained_ = drained;
    if (!drained && mode == ShutdownMode::kFinish) {
        detail::reportSafely(core_->reporter(), "Runtime", "关闭时排空预算用尽：仍有未处理的工作");
    }
    state_.store(RuntimeState::kDone, std::memory_order_release);
}

RuntimeStats Runtime::stats() const {
    RuntimeStats out;
    for (const auto& m : modules_) {
        out.modules.push_back({m->name(), m->mailbox()->stats()});
        for (const auto& sub : m->subscriptions_) out.subscriptions.push_back(sub->stats());
    }
    return out;
}

std::string Runtime::graph() const {
    std::string out = "digraph simpleslam {\n  rankdir=LR;\n";
    const auto quote = [](const std::string& s) { return '"' + s + '"'; };
    for (const auto& m : modules_) {
        out += "  " + quote(m->name()) + " [shape=box];\n";
        for (const auto& t : m->advertised_) {
            out += "  " + quote(t) + " [shape=ellipse];\n  " + quote(m->name()) + " -> " + quote(t) + ";\n";
        }
        for (const auto& sub : m->subscriptions_) {
            out += "  " + quote(sub->topicName()) + " [shape=ellipse];\n  " + quote(sub->topicName()) + " -> "
                   + quote(m->name()) + " [label=" + quote(std::to_string(sub->queueSize())) + "];\n";
        }
        for (const auto& s : m->services_) {
            out += "  " + quote(s) + " [shape=diamond];\n  " + quote(m->name()) + " -> " + quote(s) + " [style=dashed];\n";
        }
    }
    return out + "}\n";
}

std::string Runtime::statsTable() const {
    std::string out = "模块                      回调数    忙碌(ms)  最长(ms)  异常  排队\n";
    char line[256];
    const auto report = stats();
    for (const auto& m : report.modules) {
        std::snprintf(line, sizeof(line), "%-24s %8llu %10.1f %9.1f %5llu %5zu\n", m.name.c_str(),
                      static_cast<unsigned long long>(m.mailbox.callbacks), static_cast<double>(m.mailbox.busy_ns) * 1e-6,
                      static_cast<double>(m.mailbox.max_callback_ns) * 1e-6,
                      static_cast<unsigned long long>(m.mailbox.failures), m.mailbox.depth);
        out += line;
    }
    out += "话题 -> 订阅者                                  队列  峰值    送达    丢弃\n";
    for (const auto& s : report.subscriptions) {
        const std::string edge = s.topic + " -> " + s.subscriber;
        std::snprintf(line, sizeof(line), "%-46s %5zu %5zu %7llu %7llu\n", edge.c_str(), s.queue_size, s.high_water,
                      static_cast<unsigned long long>(s.delivered), static_cast<unsigned long long>(s.dropped));
        out += line;
    }
    return out;
}

std::uint64_t Runtime::isolatedFailures() const {
    std::uint64_t total = 0;
    for (const auto& m : modules_) {
        total += m->mailbox()->stats().failures + m->hook_failures_.load(std::memory_order_relaxed);
    }
    return total;
}

}  // namespace simpleslam::comm
