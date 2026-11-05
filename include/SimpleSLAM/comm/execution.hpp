// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file execution.hpp
/// 执行底座的唯一出口。Asio 只在这个头文件里出现，项目其余部分不直接用它。
///
/// 一个模块 = 一个 SerialExecutor（strand）。所有进入模块的入口——普通订阅、
/// latch 补发、服务请求、服务响应、定时器、参数通知——都经这里投递，
/// 因此模块内的处理天然串行，跨模块投递也不会在发布者的调用栈上执行。

#if defined(SIMPLESLAM_COMM_STANDALONE_ASIO)
#include <asio/executor_work_guard.hpp>
#include <asio/io_context.hpp>
#include <asio/post.hpp>
#include <asio/strand.hpp>
#else
#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/strand.hpp>
#endif

#include <cstddef>
#include <mutex>
#include <optional>
#include <utility>

namespace simpleslam::comm {

#if defined(SIMPLESLAM_COMM_STANDALONE_ASIO)
namespace net = asio;
#else
namespace net = boost::asio;
#endif

using IoContext = net::io_context;
using StrandType = net::strand<IoContext::executor_type>;

namespace detail {

/// 标记"当前线程正在推进某个运行时"。用于在入口处拒绝会自锁的阻塞调用。
[[nodiscard]] inline int& runtimeDepthTls() noexcept {
    static thread_local int depth = 0;
    return depth;
}

/// RAII 地进入/离开运行时线程标记。
class RuntimeThreadMark {
public:
    RuntimeThreadMark() noexcept { ++runtimeDepthTls(); }
    ~RuntimeThreadMark() { --runtimeDepthTls(); }
    RuntimeThreadMark(const RuntimeThreadMark&) = delete;
    RuntimeThreadMark& operator=(const RuntimeThreadMark&) = delete;
};

}  // namespace detail

/// 当前线程是否正在推进运行时（run / spinOnce 之内）。
[[nodiscard]] inline bool inRuntimeThread() noexcept { return detail::runtimeDepthTls() > 0; }

/// 串行执行上下文。不拥有 IoContext。
///
/// asio 的 strand 不能比它的 io_context 活得久：strand 析构时要回到 io_context 的服务里
/// 摘掉自己。收件箱却可能被比 Runtime 活得久的句柄引用着（服务调用端、在飞的调用），
/// 所以 strand 的寿命不跟收件箱走：Runtime 在销毁 io_context 之前调用 release()，
/// 此后 post() 返回 false，那些句柄碰到的只是一个关着的收件箱。
class SerialExecutor {
public:
    explicit SerialExecutor(IoContext& io) : strand_(net::make_strand(io)) {}

    SerialExecutor(const SerialExecutor&) = delete;
    SerialExecutor& operator=(const SerialExecutor&) = delete;
    SerialExecutor(SerialExecutor&&) = delete;
    SerialExecutor& operator=(SerialExecutor&&) = delete;
    ~SerialExecutor() = default;

    /// 投递一个任务。任意线程可调用，不阻塞，不在调用栈内执行。
    /// 返回 false 表示执行底座已经释放，任务没有排进去。
    template <class F>
    bool post(F&& f) {
        std::lock_guard lock(mutex_);
        if (!strand_) return false;
        net::post(*strand_, std::forward<F>(f));
        return true;
    }

    /// 当前线程是否正在这个上下文里执行任务。用于契约断言。
    [[nodiscard]] bool runningInThisThread() const {
        std::lock_guard lock(mutex_);
        return strand_ && strand_->running_in_this_thread();
    }

    /// 释放 strand。由 Runtime 在销毁 io_context 之前调用，此时已没有线程在推进运行时。
    void release() {
        std::optional<StrandType> doomed;
        {
            std::lock_guard lock(mutex_);
            doomed.swap(strand_);
        }
    }

private:
    mutable std::mutex mutex_;
    std::optional<StrandType> strand_;
};

}  // namespace simpleslam::comm
