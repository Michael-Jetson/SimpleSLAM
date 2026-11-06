// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file types.hpp
/// 通信层基础类型：话题描述符、消息指针、上报出口、服务终态、名称解析。

#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace simpleslam::comm {

/// 话题描述符：把话题名和消息类型绑在一起。经它发布、订阅时，消息类型取自描述符，
/// 名字对、类型错在编译期就报错，而不是装配时才发现。运行期才生成的话题名用字符串入口。
template <class T>
struct TopicDesc {
    using Message = T;
    std::string_view name;
};

/// 异常与积压的上报出口。
using ErrorReporter = std::function<void(std::string_view where, std::string_view what)>;

namespace detail {

inline void reportSafely(const ErrorReporter& reporter, std::string_view where,
                         std::string_view what) noexcept {
    if (!reporter) return;
    try {
        reporter(where, what);
    } catch (...) {  // NOLINT(bugprone-empty-catch)
    }
}

/// 句柄可能比 Runtime 活得久，所以上报出口是一个共享的小对象。
class ReportSink {
public:
    void set(ErrorReporter reporter) {
        std::lock_guard lock(mutex_);
        reporter_ = std::move(reporter);
    }
    void report(std::string_view where, std::string_view what) const {
        ErrorReporter reporter;
        {
            std::lock_guard lock(mutex_);
            reporter = reporter_;
        }
        if (reporter) reporter(where, what);
    }

private:
    mutable std::mutex mutex_;
    ErrorReporter reporter_;
};

[[nodiscard]] inline int& runtimeDepthTls() noexcept {
    static thread_local int depth = 0;
    return depth;
}

/// 标记当前线程正在推进运行时。阻塞调用靠它在入口拒绝自锁。
class RuntimeThreadMark {
public:
    RuntimeThreadMark() noexcept { ++runtimeDepthTls(); }
    ~RuntimeThreadMark() { --runtimeDepthTls(); }
    RuntimeThreadMark(const RuntimeThreadMark&) = delete;
    RuntimeThreadMark& operator=(const RuntimeThreadMark&) = delete;
};

}  // namespace detail

[[nodiscard]] inline bool inRuntimeThread() noexcept { return detail::runtimeDepthTls() > 0; }

/// 模块的线程属性。只影响在线运行，设置失败只上报，不影响运行。
struct ThreadOptions {
    int nice = 0;           ///< 正数让出 CPU；负数需要权限
    std::vector<int> cpus;  ///< 绑定的核，空表示不绑
};

/// 跨线程共享的不可变消息。
template <class T>
using MsgPtr = std::shared_ptr<const T>;
/// 通信层内部不认识消息类型，只在模块和发布端的薄类型层转换。
using AnyMsg = std::shared_ptr<const void>;

/// 装配期错误：名字非法、类型不符、服务重名。
class WiringError : public std::runtime_error {
public:
    explicit WiringError(const std::string& what) : std::runtime_error(what) {}
};

enum class ServiceStatus : std::uint8_t {
    kOk,
    kNoProvider,     ///< 该名字没有服务端
    kTimeout,        ///< 超时；不代表服务端没有执行
    kHandlerError,   ///< 服务端处理函数抛出异常
    kStopped,        ///< 运行时已停止
    kWouldDeadlock,  ///< 在运行时线程内做阻塞调用
};

[[nodiscard]] constexpr const char* toString(ServiceStatus s) noexcept {
    switch (s) {
        case ServiceStatus::kOk: return "Ok";
        case ServiceStatus::kNoProvider: return "NoProvider";
        case ServiceStatus::kTimeout: return "Timeout";
        case ServiceStatus::kHandlerError: return "HandlerError";
        case ServiceStatus::kStopped: return "Stopped";
        case ServiceStatus::kWouldDeadlock: return "WouldDeadlock";
    }
    return "Unknown";
}

/// 服务调用失败时由 ServiceResult::get() 抛出。
class ServiceError : public std::runtime_error {
public:
    ServiceError(ServiceStatus status, const std::string& what)
        : std::runtime_error(what.empty() ? toString(status) : what), status_(status) {}
    [[nodiscard]] ServiceStatus status() const noexcept { return status_; }

private:
    ServiceStatus status_;
};

template <class Response>
struct ServiceResult {
    ServiceStatus status = ServiceStatus::kNoProvider;
    MsgPtr<Response> value;  ///< 只有 kOk 时有效
    std::string error;       ///< kHandlerError 时是处理函数抛出的原因

    [[nodiscard]] bool ok() const noexcept { return status == ServiceStatus::kOk; }
    explicit operator bool() const noexcept { return ok(); }
    /// 成功时返回响应，否则抛 ServiceError。
    [[nodiscard]] const Response& get() const {
        if (!ok()) throw ServiceError(status, error);
        return *value;
    }
};

/// 回调抛出没人处理的异常时怎么办。有领域语义的失败不走这里：算法的异常是那一包的失败终态，
/// 服务处理函数的异常是 kHandlerError。
enum class ExceptionPolicy : std::uint8_t {
    kIsolate,  ///< 接住、上报、计数，继续处理下一条
    kStop,     ///< 同时停止输入，已接纳的工作走完，第一个异常从驱动调用重新抛出
};

/// 一次被接住的异常。
struct ModuleFailure {
    std::string module;
    std::string what;
    std::exception_ptr cause;
};

/// 绝对名 /a/b 原样保留；私有名 ~x 解析到模块自身；相对名 x 解析到模块所在的命名空间。
[[nodiscard]] std::string resolveName(std::string_view name, std::string_view module_name);
[[nodiscard]] bool isValidResolvedName(std::string_view name) noexcept;

}  // namespace simpleslam::comm
