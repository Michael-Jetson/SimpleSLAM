// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file types.hpp
/// 通信层基础类型：消息指针、交付策略、接纳结果、服务终态、名称解析。
///
/// 设计依据见 docs/proposals/2026-09-20-ROS风格进程内通信提案.md：
/// latch 与 Latest 是两件事；发布必须返回接纳结果而不是静默丢弃；
/// 服务的终态是一组明确枚举，不是"成功或超时"两分。

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

namespace simpleslam::comm {

/// 异常与满载的上报出口。实现必须自己吞掉异常；框架另有兜底。
using ErrorReporter = std::function<void(std::string_view where, std::string_view what)>;

namespace detail {

/// 在锁外安全调用上报出口：出口自己抛出时不得逃逸。
inline void reportSafely(const ErrorReporter& reporter, std::string_view where,
                         std::string_view what) noexcept {
    if (!reporter) return;
    try {
        reporter(where, what);
    } catch (...) {  // NOLINT(bugprone-empty-catch)
        // 上报路径不得再抛。这里没有更高层可以通知，只能就地丢弃。
    }
}

/// 运行时的上报出口。
///
/// 句柄可能比 Runtime 活得久（ServiceClient、Publisher 被拷到了别处），所以交给句柄的
/// 上报出口是一个共享的小对象，而不是捕获 Runtime 自身的闭包——后者在 Runtime 析构之后
/// 就指向已释放的内存。Runtime 析构时清空它，此后经旧句柄的上报安全地落空。
/// 用户的出口在锁外调用：它可能再调回运行时。
class ReportSink {
public:
    void set(ErrorReporter reporter) {
        std::lock_guard lock(mutex_);
        reporter_ = std::move(reporter);
    }
    void clear() { set({}); }
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

}  // namespace detail

/// 跨执行上下文共享的不可变消息。发布后不得经其他可变别名修改。
template <class T>
using MsgPtr = std::shared_ptr<const T>;

/// 订阅端的交付策略。消息语义（可替代状态 / 可丢观测 / 关键事件）决定选哪一种。
enum class Delivery : std::uint8_t {
    kReliable,    ///< 有界 FIFO，满则拒新并告知发布方（关键事件用，配合补读）
    kDropOldest,  ///< 有界 FIFO，满则丢最旧（可丢的观测帧用）
    kLatest,      ///< 覆盖槽 + 合并唤醒，只保留最新值（可替代的状态用）
};

/// 发布端选项。latch 只决定"新订阅者是否补发最后一个值"，与 Delivery 无关。
struct PublishOptions {
    bool latch = false;
    /// 这是外部输入进入系统的入口话题吗？
    ///
    /// 停止分两段：先关入口话题，让已接纳的输入把派生链路走完，再关其余话题。
    /// 只做"一次性全关"的话，一条已经被里程计处理完的包，其关键帧会因为
    /// 下游话题也被关掉而丢失，最终图与轨迹静悄悄地少一截——而精度门禁正是
    /// 拿它们做结论的。
    bool ingress = false;
};

/// 订阅端选项。queue_depth 对 kLatest 无意义（覆盖槽恒为 1）。
struct SubscribeOptions {
    std::size_t queue_depth = 10;
    Delivery delivery = Delivery::kDropOldest;
};

/// 单个订阅端的接纳结果。
enum class Admit : std::uint8_t {
    kAccepted,      ///< 入队成功
    kCoalesced,     ///< 覆盖了尚未处理的旧值（kLatest）
    kDroppedOldest, ///< 入队成功，但挤掉了最旧的一条
    kRejectedFull,  ///< 队列已满且策略为拒新
    kClosed,        ///< 订阅方已停止接收
};

/// 一次发布的汇总结果。accepted 只表示被接纳，不表示已被处理，更不表示已持久化。
struct PublishResult {
    std::size_t subscribers = 0;
    std::size_t accepted = 0;
    std::size_t coalesced = 0;
    std::size_t dropped_oldest = 0;
    std::size_t rejected_full = 0;
    std::size_t closed = 0;
    /// 可靠档订阅端的数量与其中接纳成功的数量。
    ///
    /// 分开记是因为"发布被拒 ⇔ 主消费者拒收"只在可靠订阅端恰好一个时成立。
    /// 关键数据链路要的是"指定的那个消费者确实收下了"，而不是"没人满载"。
    std::size_t reliable_subscribers = 0;
    std::size_t reliable_accepted = 0;
    /// 话题的接纳口已关闭（停止进行中）。
    ///
    /// 与"没有订阅者"不是一回事：零订阅者时两者都表现为"谁也没收到"，
    /// 但前者是系统在停，后者是接线错了。汇总计数区分不了，所以单列一个标志——
    /// 关闭且零订阅时 closed 计数恰好是 0，旧的 ok() 会把它当成成功
    /// 。
    bool topic_closed = false;

    /// 没有任何订阅端因为满载而拒收，且话题接纳口仍开着。
    ///
    /// 它**不**表示"主消费者收到了"：零订阅者同样满足。关键扇出请用
    /// deliveredTo()，状态广播才用 ok()。
    [[nodiscard]] bool ok() const noexcept { return rejected_full == 0 && !topic_closed; }

    /// 关键扇出的判据：恰好 expected 个可靠订阅端，且它们全部接纳。
    [[nodiscard]] bool deliveredTo(std::size_t expected) const noexcept {
        return !topic_closed && reliable_subscribers == expected
               && reliable_accepted == expected;
    }

    /// 存在部分接纳：关键扇出遇到这种情况要走补读或明确失败，不能当作成功。
    [[nodiscard]] bool partial() const noexcept {
        return (rejected_full + closed) > 0 && (accepted + coalesced + dropped_oldest) > 0;
    }
};

/// 服务调用的终态。
///
/// 承诺是**至多一个**终态，不是恰好一个：调用方模块一旦关闭，收件箱不再接受
/// 业务回调，此后产生的终态无处投递。
/// 关闭之后再发起的调用在入口处同步拒绝为 kStopped，这一步是可观察的；
/// 关闭时已在飞的调用，其完成回调被抑制并计入模块的"未送达完成数"。
/// 迟到响应一律丢弃。
enum class ServiceStatus : std::uint8_t {
    kOk,
    kNoProvider,     ///< 该名字没有服务端
    kRejected,       ///< 服务端准入失败（满载或拒绝）
    kTimeout,        ///< 超时；注意超时不代表服务端没有执行
    kCancelled,      ///< 调用方取消；取消不回滚副作用
    kHandlerError,   ///< 服务端处理函数抛出异常
    kStopped,        ///< 运行时或模块已停止
    kWouldDeadlock,  ///< 在运行时线程内做阻塞调用，入口处直接拒绝
};

[[nodiscard]] constexpr std::string_view toString(ServiceStatus s) noexcept {
    switch (s) {
        case ServiceStatus::kOk: return "Ok";
        case ServiceStatus::kNoProvider: return "NoProvider";
        case ServiceStatus::kRejected: return "Rejected";
        case ServiceStatus::kTimeout: return "Timeout";
        case ServiceStatus::kCancelled: return "Cancelled";
        case ServiceStatus::kHandlerError: return "HandlerError";
        case ServiceStatus::kStopped: return "Stopped";
        case ServiceStatus::kWouldDeadlock: return "WouldDeadlock";
    }
    return "Unknown";
}

/// 服务结果。只有 kOk 时 value 有效。
template <class Response>
struct ServiceResult {
    ServiceStatus status = ServiceStatus::kNoProvider;
    std::optional<Response> value;

    [[nodiscard]] bool ok() const noexcept { return status == ServiceStatus::kOk; }
    explicit operator bool() const noexcept { return ok(); }
};

/// 一次参数写入的提交结果。
///
/// 参数从写入到生效有三步，各有各的证据，不能互相顶替：
/// **Stored**（这里的 kStored：存储已校验并提交新版本）→ **Queued**（新快照已排进
/// 每个模块的收件箱，见 ParamCommitResult::fullyQueued）→ **Applied**（模块在帧边界
/// 换上了这份快照，见 Runtime::paramsApplied）。
enum class ParamCommit : std::uint8_t {
    kStored,          ///< 参数存储已校验并提交新版本
    kRejectedType,    ///< 类型不符
    kRejectedValue,   ///< 跨字段校验失败
    kRejectedStale,   ///< expected_revision 与当前版本不符
    kUnknownName,     ///< 未声明的参数名
    kRejectedStopped, ///< 运行时已进入关闭，不再接受参数写入
};

[[nodiscard]] constexpr std::string_view toString(ParamCommit c) noexcept {
    switch (c) {
        case ParamCommit::kStored: return "Stored";
        case ParamCommit::kRejectedType: return "RejectedType";
        case ParamCommit::kRejectedValue: return "RejectedValue";
        case ParamCommit::kRejectedStale: return "RejectedStale";
        case ParamCommit::kUnknownName: return "UnknownName";
        case ParamCommit::kRejectedStopped: return "RejectedStopped";
    }
    return "Unknown";
}

/// 名称解析：绝对名 /a/b 原样保留；私有名 ~x 解析到模块自身命名空间；
/// 相对名 x 解析到模块所在的命名空间。模块名必须是绝对名。
///
/// 例：模块名 /slam/odometry 时，pose → /slam/pose，~voxel_size → /slam/odometry/voxel_size。
[[nodiscard]] std::string resolveName(std::string_view name, std::string_view module_name);

/// 名称合法性：非空、以 / 开头、只含 [A-Za-z0-9_/]、无空段、不以 / 结尾（根除外）。
[[nodiscard]] bool isValidResolvedName(std::string_view name) noexcept;

}  // namespace simpleslam::comm
