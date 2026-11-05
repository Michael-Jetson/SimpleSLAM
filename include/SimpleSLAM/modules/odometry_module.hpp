// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file odometry_module.hpp
/// 里程计模块：把里程计算法接到总线与资源上。
///
/// 算法本体只实现 plugin::OdometryBase::track()，本模块负责其余全部工程事务：
/// 订阅数据包、写真源（先入库再通知，断号可补读）、更新当前状态、
/// 在帧边界应用后端校正、报告处理进度、发布诊断。
///
/// 两条与数学接口有关的纪律：
/// - 算法声明的信息矩阵来源原样入库，不由"矩阵是否为零"反推；
/// - 跨关键帧的相对约束只有算法能给，框架不拿单帧配准量冒充（keyframe.hpp）。
///
/// **每个送到这里的包恰好有一个终态**：跟踪了、按政策等待支撑、按政策拒绝、违反契约、
/// 算法失败。算法抛出的异常在这里变成那一包的"失败"终态并计数——它有领域语义，
/// 不交给通信层去当一条日志吞掉。终态之和等于收到的包数，运行结局据此核对
/// "送达的包是不是都有了下落"。
///
/// 失败之后按算法声明的恢复协议处理（默认重新初始化）：重新初始化会把关键帧链断开，
/// 之后是新的一段；当前状态随之复位，不把旧里程计系的校正套到新的一段上。
///
/// **校正先准备、后生效**：算法的准备（重建地图）成功之后才写当前状态、才算生效；
/// 准备失败时校正不生效，并按同一协议恢复——不会有帧顶着新校正的身份输出，而地图还是旧的。

#include <SimpleSLAM/comm/module.hpp>
#include <SimpleSLAM/framework/diagnostics.hpp>
#include <SimpleSLAM/plugins/odometry.hpp>
#include <SimpleSLAM/framework/solution.hpp>
#include <SimpleSLAM/modules/topics.hpp>

#include <cstdint>
#include <memory>
#include <optional>

namespace simpleslam::modules {

class OdometryModule final : public comm::Module {
public:
    /// 取走关键帧库与当前状态的写令牌：这两份资源从此只有本模块能写。
    OdometryModule(std::unique_ptr<plugin::OdometryBase> algorithm, SlamResources& resources,
                   fw::DiagnosticSink* diagnostics = nullptr);

    [[nodiscard]] plugin::OdometryBase& algorithm() noexcept { return *algorithm_; }
    /// 跟踪了的帧数（含按缺支撑降级的）。
    [[nodiscard]] std::uint64_t processedFrames() const noexcept { return processed_; }
    [[nodiscard]] std::uint64_t keyframeCount() const noexcept { return keyframes_; }
    /// 因为违反包契约而没被处理的帧数。非零说明有输入没进估计。
    [[nodiscard]] std::uint64_t contractErrors() const noexcept { return contract_errors_; }
    /// 算法抛出异常、没能产出结果的帧数。
    [[nodiscard]] std::uint64_t failedFrames() const noexcept { return failed_; }
    /// 缺支撑、按算法政策等待初始化而没有跟踪的帧数。
    [[nodiscard]] std::uint64_t awaitingSupportFrames() const noexcept { return awaiting_; }
    /// 缺支撑、按算法政策拒绝的帧数。
    [[nodiscard]] std::uint64_t rejectedForGapFrames() const noexcept { return rejected_for_gap_; }
    /// 缺支撑但照常跟踪、结果被降级的帧数。
    [[nodiscard]] std::uint64_t degradedForGapFrames() const noexcept { return degraded_for_gap_; }
    /// 收到的包数。等于各终态之和。
    [[nodiscard]] std::uint64_t receivedPackets() const noexcept { return received_; }
    /// 算法报告状态不能再用的帧数（计入失败帧）。
    [[nodiscard]] std::uint64_t stateLosses() const noexcept { return state_losses_; }
    /// 准备失败、没有生效的校正数。
    [[nodiscard]] std::uint64_t failedCorrections() const noexcept { return failed_corrections_; }
    /// 按恢复协议重新初始化的次数。每一次都让轨迹与关键帧链断开一段。
    [[nodiscard]] std::uint64_t reinitializations() const noexcept { return reinitializations_; }
    /// 算法状态推进到的时刻与下一包的传播起点接不上的次数：缺失区间进入帧上下文，
    /// 与来源缺口一起执行算法声明的缺支撑政策。
    [[nodiscard]] std::uint64_t propagationGaps() const noexcept { return propagation_gaps_; }
    /// 发布出去的里程计结果数。
    [[nodiscard]] std::uint64_t publishedResults() const noexcept { return published_results_; }
    /// 真正生效了的最新一份解的版本。准备失败的解不算。
    [[nodiscard]] std::uint64_t appliedSolution() const noexcept { return applied_solution_; }
    /// 处理过的最新一份解的版本：生效了的，或准备失败、不再尝试的。更旧的解直接跳过。
    [[nodiscard]] std::uint64_t handledSolution() const noexcept { return handled_solution_; }

protected:
    void onInit() override;

private:
    void onPacket(const comm::MsgPtr<fw::SensorPacket>& packet);
    /// 这一包有了终态：报告处理进度。累计的，按序处理，序号即进度。
    void acknowledge(const fw::SensorPacket& packet);
    /// 算法抛出异常：这一包是"失败"终态，再按算法声明的恢复协议处理。
    void failFrame(const fw::SensorPacket& packet, const char* what);
    /// 重新初始化：算法复位，关键帧链与当前状态随之断开。
    ///
    /// 复位之后算法的里程计系是新的一段（结果的段号加一）：下一个关键帧不接在旧链上
    /// （图从这里开新的一段），当前生效的校正清空；已处理版本水位继续保留。
    void reinitialize();
    fw::KeyframePtr commitKeyframe(const fw::OdometryResult& result,
                                   const comm::MsgPtr<fw::SensorPacket>& packet,
                                   const fw::FrameContext& context);
    void onCorrection(const fw::CorrectionEvent& event);
    void correctionFailed(const fw::SolutionIdentity& identity, const char* what);

    void report(fw::DiagnosticClass level, const char* key, const char* detail);
    void report(fw::DiagnosticClass level, const char* key, double value);
    /// 记录绑定到当前这一包；校正生效、重新初始化这类不针对某一包的记录，绑定的是
    /// 最近处理的那一包，表示发生在它之后的帧边界。
    [[nodiscard]] fw::DiagnosticRecord recordHeader(fw::DiagnosticClass level) const;

    /// 正在处理（或最近处理）的那一包。
    struct PacketIdentity {
        std::uint64_t sequence = 0;
        fw::Timestamp time{0};
        std::uint64_t parameter_revision = 0;
    };

    std::unique_ptr<plugin::OdometryBase> algorithm_;
    SlamResources* resources_;
    fw::WriteToken<fw::KeyframeStoreData> keyframe_writer_;
    fw::WriteToken<fw::CurrentStateData> state_writer_;
    fw::DiagnosticSink* diagnostics_;

    comm::Publisher<fw::OdometryResult> odometry_pub_;
    comm::Publisher<fw::KeyframeEvent> keyframe_pub_;
    comm::Publisher<fw::TrackingLostEvent> lost_pub_;
    comm::Publisher<fw::PacketConsumed> consumed_pub_;

    fw::KeyframeId last_keyframe_{};
    PacketIdentity current_;
    std::uint64_t received_ = 0;
    std::uint64_t state_losses_ = 0;
    std::uint64_t failed_corrections_ = 0;
    std::uint64_t reinitializations_ = 0;
    std::uint64_t propagation_gaps_ = 0;
    /// 算法状态推进到的时刻。重新初始化之后没有状态，为空。
    std::optional<fw::Timestamp> last_state_time_;
    std::uint64_t processed_ = 0;
    std::uint64_t contract_errors_ = 0;
    std::uint64_t failed_ = 0;
    std::uint64_t awaiting_ = 0;
    std::uint64_t rejected_for_gap_ = 0;
    std::uint64_t degraded_for_gap_ = 0;
    std::uint64_t published_results_ = 0;
    std::uint64_t keyframes_ = 0;
    std::uint64_t applied_solution_ = 0;
    std::uint64_t handled_solution_ = 0;
};

}  // namespace simpleslam::modules
