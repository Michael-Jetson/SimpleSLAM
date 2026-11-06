// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/plugins/odometry.hpp>

#include <chrono>
#include <cmath>
#include <utility>

namespace simpleslam::plugin {

namespace {

[[nodiscard]] bool allFinite(const SE3d& pose) {
    const auto t = pose.translation();
    const auto q = pose.quat();
    return std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z())
           && std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y())
           && std::isfinite(q.z());
}

}  // namespace

FrameOutcome OdometryBase::process(const SensorPacket& packet, const fw::FrameContext& context) {
    if (!packet.imuCoversDeclaredRange()) {
        throw OdometryContractError("SensorPacket 的 IMU 未覆盖其声明区间", ContractStage::kInput);
    }
    context_ = context;

    FrameOutcome outcome;
    if (!state_ready_) {
        outcome.disposition = FrameDisposition::kStateUnusable;
        return outcome;  // onReset 失败后不得继续调用 track
    }
    const auto limit = maxImuGap();
    outcome.support_gap = packet.gap != fw::GapReason::kNone
                          || context.missing_propagation.has_value()
                          || (limit && packet.imu_max_gap > *limit);
    if (outcome.support_gap) {
        const SupportGapPolicy policy = supportGapPolicy();
        if (policy == SupportGapPolicy::kReject) {
            outcome.disposition = FrameDisposition::kRejectedForGap;
            return outcome;
        }
        if (policy == SupportGapPolicy::kWaitForInitialization && !initialized_) {
            outcome.disposition = FrameDisposition::kAwaitingSupport;
            return outcome;
        }
    }

    // 算法本体的耗时是逐帧的度量（墙钟）；抛出的那一帧不记
    const auto started = std::chrono::steady_clock::now();
    TrackResult tracked = track(packet);
    const std::chrono::duration<double, std::milli> elapsed =
        std::chrono::steady_clock::now() - started;
    log_.metric("track.duration", elapsed.count(), "ms");
    if (!tracked.state_usable) {
        // 算法自己说状态不能再用：这一帧的位姿不可信，不发布，交给恢复协议
        outcome.disposition = FrameDisposition::kStateUnusable;
        state_ready_ = false;
        return outcome;
    }
    // 缺支撑的包不能当完整包：算法报了"正常跟踪"，结论也只到"降级"为止
    if (outcome.support_gap && tracked.status == TrackingStatus::kTracking) {
        tracked.status = TrackingStatus::kDegraded;
    }
    if (!allFinite(tracked.T_odom_body)) {
        throw OdometryContractError("track() 返回了非有限位姿");
    }
    if (tracked.state_time && *tracked.state_time > packet.reference_time) {
        throw OdometryContractError("估计状态时刻晚于本包参考时刻");
    }
    if (tracked.constraint_to_last_keyframe) {
        const auto& constraint = *tracked.constraint_to_last_keyframe;
        if (constraint.to_time != packet.reference_time) {
            throw OdometryContractError("相对约束的终点时刻与本包参考时刻不符");
        }
        // 起点也要核：只核终点的话，一条起点指向别处的约束会被当成
        // "上一关键帧 → 本帧"接进图里，而它约束的根本不是这两个节点
        if (!has_keyframe_ || constraint.from_time != last_keyframe_time_) {
            throw OdometryContractError("相对约束的起点时刻与上一关键帧不符");
        }
        if (!allFinite(constraint.measurement)) {
            throw OdometryContractError("相对约束的测量含非有限值");
        }
        if (!fw::isValidInformation(constraint.information)) {
            throw OdometryContractError("相对约束的信息矩阵不合法（须有限、对称、半正定）");
        }
        if (constraint.source == fw::InformationSource::kUnset) {
            throw OdometryContractError("相对约束必须声明信息来源");
        }
    }

    bool keyframe = false;
    try {
        keyframe = tracked.status != TrackingStatus::kLost && selectKeyframe(tracked, packet);
    } catch (...) {
        throw OdometryContractError("关键帧判定失败，track 已执行，须恢复估计状态");
    }

    OdometryResult result;
    result.sequence = frame_sequence_ + 1;
    result.packet_sequence = packet.sequence;
    result.odometry_segment = segment_;
    result.time = packet.reference_time;
    result.status = tracked.status;
    result.T_odom_body = tracked.T_odom_body;
    result.covariance = tracked.covariance;
    result.registration_information = tracked.registration_information;
    result.registration_information_source = tracked.registration_information_source;
    result.constraint_to_last_keyframe = std::move(tracked.constraint_to_last_keyframe);
    result.is_keyframe = keyframe;
    // 结果自带它成立的前提：当时实际生效的那一份校正
    result.T_world_odom = T_world_odom_;
    result.applied_correction = correction_identity_;
    result.parameter_revision = context_.parameter_revision;
    result.calibration_revision = context_.calibrationRevision();
    result.state_time = tracked.state_time.value_or(packet.reference_time);
    result.support_gap = outcome.support_gap;
    result.iterations = tracked.iterations;
    result.inliers = tracked.inliers;

    if (keyframe) {
        try {
            onKeyframeSelected(result);
        } catch (...) {
            throw OdometryContractError("关键帧提交钩子失败，须恢复估计状态");
        }
        last_keyframe_pose_ = result.T_odom_body;
        last_keyframe_time_ = result.time;
        has_keyframe_ = true;
    }
    // 后置钩子也属于本次处理；它失败时不提交基类的序号与最后结果。
    last_ = result;
    frame_sequence_ = result.sequence;
    if (result.status == TrackingStatus::kTracking || result.status == TrackingStatus::kDegraded) {
        initialized_ = true;
    }
    outcome.result = std::move(result);
    return outcome;
}

void OdometryBase::applyCorrection(const SE3d& T_world_odom, const PoseGraphData& graph,
                                   const SolutionIdentity& identity) {
    if (!state_ready_) throw OdometryContractError("估计状态尚未恢复，不能准备校正");
    onCorrection(T_world_odom, graph);
    correction_identity_ = identity;
    T_world_odom_ = T_world_odom;
}

void OdometryBase::reset(SessionId session) {
    session_ = session;
    frame_sequence_ = 0;
    segment_ = 0;
    restartState();
}

void OdometryBase::reinitialize() {
    ++segment_;
    restartState();
}

void OdometryBase::restartState() {
    state_ready_ = false;
    last_ = OdometryResult{};
    correction_identity_ = SolutionIdentity{};
    T_world_odom_ = SE3d::Identity();
    has_keyframe_ = false;
    initialized_ = false;
    last_keyframe_pose_ = SE3d::Identity();
    last_keyframe_time_ = Timestamp{0};
    onReset();
    state_ready_ = true;
}

}  // namespace simpleslam::plugin
