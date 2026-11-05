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

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>

namespace simpleslam::modules {

class OdometryModule final : public comm::Module {
public:
    OdometryModule(std::unique_ptr<plugin::OdometryBase> algorithm, SlamResources& resources,
                   fw::DiagnosticSink* diagnostics = nullptr)
        : algorithm_(std::move(algorithm)),
          resources_(&resources),
          keyframe_writer_(resources.keyframes.acquireWriteToken()),
          state_writer_(resources.state.acquireWriteToken()),
          diagnostics_(diagnostics) {}

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
    void onInit() override {
        algorithm_->reset(resources_->session);
        odometry_pub_ = advertise<fw::OdometryResult>(topics::kOdometry);
        keyframe_pub_ = advertise<fw::KeyframeEvent>(topics::kKeyframe);
        lost_pub_ = advertise<fw::TrackingLostEvent>(topics::kTrackingLost);
        consumed_pub_ = advertise<fw::PacketConsumed>(topics::kPacketConsumed);

        // 包不能在这里丢：丢包等于丢它带的 IMU，状态传播就接不上了。过载由入口丢触发帧
        subscribe(topics::kSensorPacket, 0, &OdometryModule::onPacket);
        // 校正事件自带解快照；取最新一份即可，旧解不需要逐个应用
        subscribe(topics::kCorrection, 1, &OdometryModule::onCorrection);
    }

private:
    void onPacket(const comm::MsgPtr<fw::SensorPacket>& packet) {
        // 帧边界上取一次上下文：标定快照与参数版本在本帧内恒定，
        // 算法用的和产物标的因此是同一份
        fw::FrameContext context;
        context.calibration = resources_->calibrationSnapshot();
        context.params = paramSnapshot();
        context.parameter_revision = context.params ? context.params->revision : 0;
        // 这一包的终态与诊断都绑定到它的身份，不绑定到"已经跟踪了几帧"
        current_ = PacketIdentity{packet->sequence, packet->reference_time,
                                  context.parameter_revision};

        ++received_;
        // 算法的状态停在哪一刻，与这一包的传播起点对一下：接不上就是一段没进这一包的输入
        if (last_state_time_ && *last_state_time_ < packet->propagation.begin) {
            context.missing_propagation = fw::TimeRange{*last_state_time_, packet->propagation.begin};
            ++propagation_gaps_;
            report(fw::DiagnosticClass::kControl, "odometry.propagation_gap", "");
        }
        plugin::FrameOutcome outcome;
        try {
            outcome = algorithm_->process(*packet, context);
        } catch (const plugin::OdometryContractError& e) {
            ++contract_errors_;
            report(fw::DiagnosticClass::kControl, "odometry.contract_error", e.what());
            if (e.stage() == plugin::ContractStage::kOutput) reinitialize();
            // 每个终态都要报进度：否则入口以为下游一直在忙
            acknowledge(*packet);
            return;
        } catch (const std::exception& e) {
            failFrame(*packet, e.what());
            return;
        } catch (...) {
            failFrame(*packet, "算法抛出未知异常");
            return;
        }
        switch (outcome.disposition) {
            case plugin::FrameDisposition::kTracked:
                break;
            case plugin::FrameDisposition::kStateUnusable:
                // 算法自己说状态不能再用：不论声明的恢复协议是什么，都只能重新初始化
                ++failed_;
                ++state_losses_;
                report(fw::DiagnosticClass::kControl, "odometry.state_unusable", "");
                reinitialize();
                acknowledge(*packet);
                return;
            case plugin::FrameDisposition::kAwaitingSupport:
            case plugin::FrameDisposition::kRejectedForGap:
                if (outcome.disposition == plugin::FrameDisposition::kAwaitingSupport) {
                    ++awaiting_;
                } else {
                    ++rejected_for_gap_;
                }
                report(fw::DiagnosticClass::kControl, "odometry.support_gap",
                       plugin::toString(outcome.disposition));
                acknowledge(*packet);
                return;
        }
        ++processed_;
        if (outcome.support_gap) ++degraded_for_gap_;
        auto& result = outcome.result;
        last_state_time_ = result.state_time;

        state_writer_.modify([&](fw::CurrentStateData& state) {
            state.time = result.time;
            state.T_odom_body = result.T_odom_body;
            state.status = result.status;
        });

        if (result.is_keyframe) {
            // 先在真源提交，再发通知：通知丢了也能凭序号补读，位姿图不会断链
            const auto frame = commitKeyframe(result, packet, context);
            fw::KeyframeEvent event;
            // 通知里的序号是关键帧库的序号，不是帧序号：消费者按它发现断号、补读
            event.sequence = frame->sequence;
            event.id = frame->id;
            event.time = result.time;
            event.T_odom_body = result.T_odom_body;
            keyframe_pub_.publish(event);
            ++keyframes_;
        }

        odometry_pub_.publish(result);
        ++published_results_;

        if (result.status == fw::TrackingStatus::kLost) {
            fw::TrackingLostEvent lost;
            lost.time = result.time;
            lost.last_good_T_odom_body = result.T_odom_body;
            lost_pub_.publish(lost);
            report(fw::DiagnosticClass::kControl, "odometry.tracking_lost", "");
        }

        report(fw::DiagnosticClass::kMetric, "odometry.inliers",
               static_cast<double>(result.inliers));

        acknowledge(*packet);
    }

    /// 这一包有了终态：报告处理进度。累计的，按序处理，序号即进度。
    void acknowledge(const fw::SensorPacket& packet) {
        consumed_pub_.publish(fw::PacketConsumed{packet.sequence});
    }

    /// 算法抛出异常：这一包是"失败"终态，再按算法声明的恢复协议处理。
    void failFrame(const fw::SensorPacket& packet, const char* what) {
        ++failed_;
        report(fw::DiagnosticClass::kControl, "odometry.frame_failed", what);
        if (algorithm_->failureRecovery() == plugin::FailureRecovery::kReinitialize) {
            reinitialize();
        }
        acknowledge(packet);
    }

    /// 重新初始化：算法复位，关键帧链与当前状态随之断开。
    ///
    /// 复位之后算法的里程计系是新的一段（结果的段号加一）：下一个关键帧不接在旧链上
    /// （图从这里开新的一段），当前生效的校正清空；已处理版本水位继续保留。
    void reinitialize() {
        last_keyframe_ = fw::KeyframeId{};
        last_state_time_.reset();
        applied_solution_ = 0;
        state_writer_.modify([](fw::CurrentStateData& state) {
            state = fw::CurrentStateData{};
        });
        ++reinitializations_;
        try {
            algorithm_->reinitialize();
        } catch (const std::exception& e) {
            report(fw::DiagnosticClass::kControl, "odometry.reinitialize_failed", e.what());
            return;  // 基类保留不可用状态；本包仍须报告进度
        } catch (...) {
            report(fw::DiagnosticClass::kControl, "odometry.reinitialize_failed", "复位抛出未知异常");
            return;
        }
        report(fw::DiagnosticClass::kControl, "odometry.reinitialized", "");
    }

    fw::KeyframePtr commitKeyframe(const fw::OdometryResult& result,
                                   const comm::MsgPtr<fw::SensorPacket>& packet,
                                   const fw::FrameContext& context) {
        fw::KeyframePtr committed;
        keyframe_writer_.modify([&](fw::KeyframeStoreData& store) {
            auto frame = std::make_shared<fw::Keyframe>();
            frame->id = fw::KeyframeId{store.last_id.value + 1};
            frame->sequence = store.next_sequence++;
            frame->odometry_segment = result.odometry_segment;
            frame->time = result.time;
            frame->T_odom_body = result.T_odom_body;
            frame->packet = packet;  // 整包原始数据进真源
            // 继承本帧实际用的那一份：帧处理中标定可能已经换代，现读最新版本会贴错身份。
            // 内容一并留下——版本号代替不了内容，按当时条件重建这一帧要用它
            frame->calibration_revision = result.calibration_revision;
            frame->calibration = context.calibration;
            frame->previous = last_keyframe_;
            // 算法给出的跨关键帧约束原样保存；只有端点对得上才认
            if (result.constraint_to_last_keyframe && last_keyframe_.valid()) {
                frame->constraint_from_previous = result.constraint_to_last_keyframe;
            }
            frame->registration_information = result.registration_information;
            frame->registration_information_source = result.registration_information_source;
            committed = frame;
            store.append(std::move(frame));
        });
        last_keyframe_ = committed->id;
        return committed;
    }

    void onCorrection(const fw::CorrectionEvent& event) {
        if (!event.valid()) {
            report(fw::DiagnosticClass::kControl, "odometry.correction_empty", "");
            return;
        }
        // 身份与数据同源：都来自事件携带的那份不可变快照
        const auto& solution = *event.solution;
        const fw::SolutionIdentity identity = solution.identity;

        // 身份失效（换会话、标定不兼容）的结果直接拒；版本稍旧但兼容的仍然接受
        if (!identity.compatibleWith(resources_->session, resources_->calibrationRevision())) {
            report(fw::DiagnosticClass::kControl, "odometry.correction_rejected", "身份失效");
            return;
        }
        if (identity.solution_revision <= handled_solution_) return;

        const auto keyframes = resources_->keyframes.snapshot();
        const auto correction = fw::worldFromOdom(solution, *keyframes, algorithm_->odometrySegment());
        if (!correction) return;

        const SE3d T_world_odom = *correction;
        // 帧边界：本回调与 onPacket 在同一收件箱里排队，不会并发。
        // 先让算法准备并激活，成功之后才写当前状态、才算生效；准备失败时校正不生效，
        // 算法状态可能已被部分修改，按恢复协议重新初始化
        try {
            algorithm_->applyCorrection(T_world_odom, solution, identity);
        } catch (const std::exception& e) {
            correctionFailed(identity, e.what());
            return;
        } catch (...) {
            correctionFailed(identity, "校正准备抛出未知异常");
            return;
        }
        state_writer_.modify([&](fw::CurrentStateData& state) {
            state.T_world_odom = T_world_odom;
            state.correction_identity = identity;
        });
        applied_solution_ = identity.solution_revision;
        handled_solution_ = identity.solution_revision;
        report(fw::DiagnosticClass::kControl, "odometry.correction_applied", "");
    }

    void correctionFailed(const fw::SolutionIdentity& identity, const char* what) {
        ++failed_corrections_;
        // 这份解不会再被尝试：它面对的是失败之前的里程计系
        handled_solution_ = std::max(handled_solution_, identity.solution_revision);
        report(fw::DiagnosticClass::kControl, "odometry.correction_failed", what);
        reinitialize();
    }

    void report(fw::DiagnosticClass level, const char* key, const char* detail) {
        if (diagnostics_ == nullptr) return;
        auto record = recordHeader(level);
        record.payload = fw::EventRecord{key, detail};
        diagnostics_->submit(record);
    }
    void report(fw::DiagnosticClass level, const char* key, double value) {
        if (diagnostics_ == nullptr) return;
        auto record = recordHeader(level);
        record.payload = fw::MetricRecord{key, value, ""};
        diagnostics_->submit(record);
    }
    /// 记录绑定到当前这一包；校正生效、重新初始化这类不针对某一包的记录，绑定的是
    /// 最近处理的那一包，表示发生在它之后的帧边界。
    [[nodiscard]] fw::DiagnosticRecord recordHeader(fw::DiagnosticClass level) const {
        fw::DiagnosticRecord record;
        record.header.module = name();
        record.header.frame_sequence = current_.sequence;
        record.header.data_time = current_.time;
        record.header.parameter_revision = current_.parameter_revision;
        record.header.session = resources_->session;
        record.level = level;
        return record;
    }

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
