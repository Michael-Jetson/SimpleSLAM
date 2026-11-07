// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/modules/odometry_module.hpp>

#include <algorithm>
#include <exception>
#include <memory>
#include <utility>

namespace simpleslam::modules {

OdometryModule::OdometryModule(std::unique_ptr<plugin::OdometryBase> algorithm,
                               SlamResources& resources)
    : algorithm_(std::move(algorithm)),
      resources_(&resources),
      keyframe_writer_(resources.keyframes.acquireWriteToken()),
      state_writer_(resources.state.acquireWriteToken()) {}

void OdometryModule::onInit() {
    // 不经槽、手工构造的算法在这里接到本模块的日志下面；经槽构造的已经接好了
    logging::adopt(*algorithm_, log_, "plugin", algorithm_->name());
    algorithm_->reset(resources_->session);
    odometry_pub_ = advertise(topics::kOdometry);
    keyframe_pub_ = advertise(topics::kKeyframe);
    lost_pub_ = advertise(topics::kTrackingLost);
    consumed_pub_ = advertise(topics::kPacketConsumed);

    // 包不能在这里丢：丢包等于丢它带的 IMU，状态传播就接不上了。过载由入口丢触发帧
    subscribe(topics::kSensorPacket, 0, &OdometryModule::onPacket);
    // 校正事件自带解快照；取最新一份即可，旧解不需要逐个应用
    subscribe(topics::kCorrection, 1, &OdometryModule::onCorrection);
}

void OdometryModule::onPacket(const comm::MsgPtr<fw::SensorPacket>& packet) {
    // 帧边界上取一次上下文：标定快照与参数版本在本帧内恒定，
    // 算法用的和产物标的因此是同一份
    fw::FrameContext context;
    context.calibration = resources_->calibrationSnapshot();
    context.params = paramSnapshot();
    context.parameter_revision = context.params ? context.params->revision : 0;
    // 这一包的终态与诊断都绑定到它的身份，不绑定到"已经跟踪了几帧"
    current_ = PacketIdentity{packet->sequence, packet->reference_time,
                              context.parameter_revision};
    log_.setFrame({.sequence = packet->sequence,
                   .data_time_ns = packet->reference_time.count(),
                   .session = resources_->session.value,
                   .parameter_revision = context.parameter_revision});

    ++received_;
    // 上一次复位没成功：先重试。成功了这一包照常处理；还不行，这一包记为状态不能再用
    if (!algorithm_->stateReady()) {
        reinitialize();
        if (!algorithm_->stateReady()) {
            ++failed_;
            ++state_losses_;
            log_.event("odometry.state_unusable", "复位还没成功");
            acknowledge(*packet);
            return;
        }
    }
    // 算法的状态停在哪一刻，与这一包的传播起点对一下：接不上就是一段没进这一包的输入
    if (last_state_time_ && *last_state_time_ < packet->propagation.begin) {
        context.missing_propagation = fw::TimeRange{*last_state_time_, packet->propagation.begin};
        ++propagation_gaps_;
        log_.event("odometry.propagation_gap", "");
    }
    plugin::FrameOutcome outcome;
    try {
        outcome = algorithm_->process(*packet, context);
    } catch (const plugin::OdometryContractError& e) {
        log_.event("odometry.contract_error", e.what());
        if (e.stage() == plugin::ContractStage::kOutput) {
            ++output_contract_errors_;  // 跟踪过了：结果作废，估计状态要恢复
            reinitialize();
        } else {
            ++contract_errors_;  // 输入不合契约：没进估计
        }
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
            log_.event("odometry.state_unusable", "");
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
            log_.event("odometry.support_gap", plugin::toString(outcome.disposition));
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
        log_.event("odometry.tracking_lost", "");
    }

    log_.metric("odometry.inliers", static_cast<double>(result.inliers));

    acknowledge(*packet);
}

void OdometryModule::acknowledge(const fw::SensorPacket& packet) {
    consumed_pub_.publish(fw::PacketConsumed{packet.sequence});
}

void OdometryModule::failFrame(const fw::SensorPacket& packet, const char* what) {
    ++failed_;
    log_.event("odometry.frame_failed", what);
    if (algorithm_->failureRecovery() == plugin::FailureRecovery::kReinitialize) {
        reinitialize();
    }
    acknowledge(packet);
}

void OdometryModule::reinitialize() {
    last_keyframe_ = fw::KeyframeId{};
    last_state_time_.reset();
    applied_solution_ = 0;
    state_writer_.modify([](fw::CurrentStateData& state) { state = fw::CurrentStateData{}; });
    ++reinitializations_;
    try {
        algorithm_->reinitialize();
    } catch (const std::exception& e) {
        log_.event("odometry.reinitialize_failed", e.what());
        return;  // 基类保留不可用状态；本包仍须报告进度
    } catch (...) {
        log_.event("odometry.reinitialize_failed", "复位抛出未知异常");
        return;
    }
    log_.event("odometry.reinitialized", "");
}

fw::KeyframePtr OdometryModule::commitKeyframe(const fw::OdometryResult& result,
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

void OdometryModule::onCorrection(const fw::CorrectionEvent& event) {
    if (!event.valid()) {
        log_.event("odometry.correction_empty", "");
        return;
    }
    // 身份与数据同源：都来自事件携带的那份不可变快照
    const auto& solution = *event.solution;
    const fw::SolutionIdentity identity = solution.identity;

    // 身份失效（换会话、标定不兼容）的结果直接拒；版本稍旧但兼容的仍然接受
    if (!identity.compatibleWith(resources_->session, resources_->calibrationRevision())) {
        log_.event("odometry.correction_rejected", "身份失效");
        return;
    }
    if (identity.solution_revision <= handled_solution_) return;

    const auto keyframes = resources_->keyframes.snapshot();
    const auto correction =
        fw::worldFromOdom(solution, *keyframes, algorithm_->odometrySegment());
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
    log_.event("odometry.correction_applied", "");
}

void OdometryModule::correctionFailed(const fw::SolutionIdentity& identity, const char* what) {
    ++failed_corrections_;
    // 这份解不会再被尝试：它面对的是失败之前的里程计系
    handled_solution_ = std::max(handled_solution_, identity.solution_revision);
    log_.event("odometry.correction_failed", what);
    reinitialize();
}

}  // namespace simpleslam::modules
