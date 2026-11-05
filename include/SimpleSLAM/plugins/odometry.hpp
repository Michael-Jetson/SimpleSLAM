// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file odometry.hpp
/// 里程计算法接口。写算法的人只面对这一层。
///
/// 这是**插件**：纯算法，按帧调用，不碰总线、不碰资源、
/// 无生命周期，数据由所属模块取好传入。接到总线与资源上是模块的事
/// （`modules/odometry_module.hpp`）。目录与命名空间都独立，正是为了让
/// "这是插件还是模块"在使用处一眼可判，而不是靠记忆。
///
/// 框架职责（固定骨架）与算法本体分开：`process()` 非虚，负责计时、序号、
/// 关键帧判定、结果组装与契约检查；算法只实现 `track()`。
///
/// 两类信息矩阵不要混：
/// - `registration_information` 是"本帧相对局部地图"的配准量，逐帧都有；
/// - `constraint_to_last_keyframe` 是"上一关键帧 → 本帧"的相对约束，跨帧传播而来。
/// 图边要的是后者。只有算法知道自己跨了几帧、用没用局部地图、融没融 IMU 预积分，
/// 所以框架不会由前者推出后者；算法不给，框架就按策略补启发式权重并如实标注。
///
/// **缺支撑的包不能当完整包处理**。打包器如实标出缺口（首帧前半段没有 IMU、中间断流、
/// 数据被拒收……）；遇到这样的包怎么办，算法必须明确选择（`supportGapPolicy()`）：
/// 初始化前等待、降级处理、或拒绝。框架执行这个选择：拒绝与等待的包不进 track()，
/// 降级处理的包即使算法报了 kTracking，结果也只会是 kDegraded。算法对惯性支撑有自己的
/// 连续性上限时用 `maxImuGap()` 声明——它独立于来源推荐的阈值，超了同样按缺支撑处理。
///
/// **"包已接纳""回调结束""状态推进到本帧"是三件事**。预期内的退化、缺观测、初始化
/// 未完成用结果表达（状态、`state_time`）；算法可以明说"这一帧之后我的状态不能再用"
/// （`state_usable = false`）。意料之外的异常由框架接住，按算法声明的失败恢复协议处理
/// （`failureRecovery()`）：通用的 try-catch 只能保证异常不外传，替有状态的算法恢复不了
/// 统计上的正确性，所以默认是重新初始化。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/events.hpp>
#include <SimpleSLAM/framework/frame_context.hpp>
#include <SimpleSLAM/framework/odometry_result.hpp>
#include <SimpleSLAM/framework/pose_graph.hpp>
#include <SimpleSLAM/framework/sensor_packet.hpp>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace simpleslam::plugin {

namespace fw = simpleslam::fw;

using fw::InformationSource;
using fw::OdometryResult;
using fw::PoseGraphData;
using fw::RelativeConstraint;
using fw::SensorPacket;
using fw::SessionId;
using fw::SolutionIdentity;
using fw::Timestamp;
using fw::TrackingStatus;

/// 算法一次跟踪的产出。
struct TrackResult {
    TrackingStatus status = TrackingStatus::kTracking;
    /// 本帧位姿，里程计系。时间戳按公约取包的参考时刻（扫描终点）。
    SE3d T_odom_body = SE3d::Identity();
    /// 本帧位姿的协方差。
    Mat6d covariance = Mat6d::Identity();
    /// 本帧配准的信息矩阵（本帧相对局部地图）。退化时可以只是半正定。
    /// 它不是图边权重——见文件头。
    Mat6d registration_information = Mat6d::Zero();
    /// 上面那个矩阵的来源。由算法声明，框架原样保存，不由"是否为零"反推。
    InformationSource registration_information_source = InformationSource::kUnset;
    /// 上一关键帧 → 本帧的相对约束。本帧被选为关键帧时，它就是那条图边。
    /// 留空表示算法没有给出跨关键帧的传播量，框架按策略补启发式权重。
    std::optional<RelativeConstraint> constraint_to_last_keyframe;
    /// 算法给出的关键帧建议。最终是否成为关键帧由 selectKeyframe 决定。
    bool suggest_keyframe = false;
    /// 估计状态推进到了哪个时刻。空表示推进到了本包的参考时刻；测量更新被拒而回滚时
    /// 填回滚到的时刻——下一包的传播起点与它接不上，就是一段没有被积分的输入。
    std::optional<Timestamp> state_time;
    /// 这一帧之后，算法的状态是否还能继续用。false 时框架不发布这一帧的结果，
    /// 按失败恢复协议处理（默认重新初始化）。
    bool state_usable = true;
    std::uint32_t iterations = 0;
    std::uint32_t inliers = 0;
};

/// 契约失败发生的阶段。只有进入算法之前的检查失败能保证没有修改估计状态。
enum class ContractStage : std::uint8_t { kInput, kOutput };

/// 契约违反。插件自行抛出时保守地按执行后的失败处理。
class OdometryContractError : public std::runtime_error {
public:
    explicit OdometryContractError(const std::string& what,
                                   ContractStage stage = ContractStage::kOutput)
        : std::runtime_error(what), stage_(stage) {}
    [[nodiscard]] ContractStage stage() const noexcept { return stage_; }

private:
    ContractStage stage_;
};

/// 遇到缺支撑的包，算法选择怎么办。没有默认值：这是算法必须自己做的决定。
enum class SupportGapPolicy : std::uint8_t {
    /// 初始化之前不跟踪缺支撑的包，等第一个支撑完整的包再初始化；初始化之后按降级处理。
    kWaitForInitialization,
    /// 照常跟踪，但结果至多是 kDegraded。
    kDegrade,
    /// 不跟踪缺支撑的包。
    kReject,
};

[[nodiscard]] constexpr const char* toString(SupportGapPolicy policy) noexcept {
    switch (policy) {
        case SupportGapPolicy::kWaitForInitialization: return "wait_for_initialization";
        case SupportGapPolicy::kDegrade: return "degrade";
        case SupportGapPolicy::kReject: return "reject";
    }
    return "unknown";
}

/// 一个包在算法这一侧的结局。每个送到算法的包恰好落在其中之一。
enum class FrameDisposition : std::uint8_t {
    kTracked,           ///< 跟踪了，有结果
    kAwaitingSupport,   ///< 初始化之前缺支撑，按政策等待，没有跟踪
    kRejectedForGap,    ///< 缺支撑，按政策拒绝，没有跟踪
    kStateUnusable,     ///< 跟踪了，但算法报告状态不能再用：结果不发布，按失败恢复协议处理
};

[[nodiscard]] constexpr const char* toString(FrameDisposition disposition) noexcept {
    switch (disposition) {
        case FrameDisposition::kTracked: return "tracked";
        case FrameDisposition::kAwaitingSupport: return "awaiting_support";
        case FrameDisposition::kRejectedForGap: return "rejected_for_gap";
        case FrameDisposition::kStateUnusable: return "state_unusable";
    }
    return "unknown";
}

/// 算法抛出意料之外的异常、或报告状态不可用之后，框架怎么恢复。
enum class FailureRecovery : std::uint8_t {
    /// 默认：状态可能已被部分修改，重新初始化（`reinitialize()`）。里程计轨迹与关键帧链
    /// 从这里断开，之后是新的一段（结果的段号加一），不与之前的里程计系相连。
    kReinitialize,
    /// 算法保证 track 抛异常时状态不被改动（强异常安全）：只放弃这一帧。
    /// track 正常返回后的契约校验或关键帧钩子失败不享有这个保证，必须恢复。
    kStateIntact,
};

[[nodiscard]] constexpr const char* toString(FailureRecovery recovery) noexcept {
    return recovery == FailureRecovery::kStateIntact ? "state_intact" : "reinitialize";
}

/// process() 的产出：结局，以及跟踪了时的结果。
struct FrameOutcome {
    FrameDisposition disposition = FrameDisposition::kTracked;
    /// 这个包缺支撑（打包器标了缺口，或超出了算法自己声明的连续性上限）。
    bool support_gap = false;
    /// 仅当 disposition 为 kTracked 时有意义。
    OdometryResult result;
};

class OdometryBase {
public:
    virtual ~OdometryBase() = default;
    OdometryBase(const OdometryBase&) = delete;
    OdometryBase& operator=(const OdometryBase&) = delete;

    /// 框架入口，非虚。算法不要覆盖它。
    ///
    /// 上下文在帧边界由模块取好传入：这一帧用的标定与参数版本，算法读它、
    /// 产物标它，因此必然是同一份。算法不持有资源句柄，也就不会在一帧之内
    /// 取到两份不同的标定。
    ///
    /// 缺支撑的包按算法声明的政策处理（见文件头），结局写在返回值里。
    FrameOutcome process(const SensorPacket& packet, const fw::FrameContext& context) {
        if (!packet.imuCoversDeclaredRange()) {
            throw OdometryContractError("SensorPacket 的 IMU 未覆盖其声明区间",
                                        ContractStage::kInput);
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

        TrackResult tracked = track(packet);
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

    /// 在帧边界应用后端校正：**先准备，成功之后才激活**。框架保证不会与 process() 并发。
    ///
    /// 准备是算法的事（重建局部地图等，可能耗时、可能失败）；激活只是切换这里的两个值，
    /// 不会失败。准备抛出时校正不生效，异常原样外传——算法状态可能已被部分修改，由模块
    /// 按失败恢复协议处理。顺序反过来的话，准备失败之后的每一帧都会顶着新校正的身份输出，
    /// 而地图还是旧的。
    void applyCorrection(const SE3d& T_world_odom, const PoseGraphData& graph,
                         const SolutionIdentity& identity) {
        if (!state_ready_) throw OdometryContractError("估计状态尚未恢复，不能准备校正");
        onCorrection(T_world_odom, graph);
        correction_identity_ = identity;
        T_world_odom_ = T_world_odom;
    }

    /// 重新开始一个建图会话：帧序号与段号都从头数。
    void reset(SessionId session) {
        session_ = session;
        frame_sequence_ = 0;
        segment_ = 0;
        restartState();
    }

    /// 按恢复协议重新初始化：丢掉算法状态，下一帧起是新的一段里程计系。
    /// 会话不变，帧序号接着数（帧序号在一个会话里唯一）；段号加一，下游据此知道
    /// 前后两段的位姿不在同一个里程计系里。
    void reinitialize() {
        ++segment_;
        restartState();
    }

    /// 遇到缺支撑的包怎么办。每个算法都必须明确选择，框架按它执行。
    [[nodiscard]] virtual SupportGapPolicy supportGapPolicy() const = 0;

    /// 抛出意料之外的异常、或报告状态不可用之后怎么恢复。默认重新初始化；
    /// 只有能保证强异常安全的算法才该声明 kStateIntact。
    [[nodiscard]] virtual FailureRecovery failureRecovery() const {
        return FailureRecovery::kReinitialize;
    }

    /// 算法对惯性支撑的连续性上限：包内相邻 IMU 样本的间隔超过它，就按缺支撑处理。
    /// 它独立于来源推荐的打包阈值——那是对数据的描述，这是算法的能力。空表示不另设上限。
    [[nodiscard]] virtual std::optional<fw::Duration> maxImuGap() const { return std::nullopt; }

    /// 算法是否已经初始化：产出过至少一个正常或降级的跟踪结果。
    [[nodiscard]] bool initialized() const noexcept { return initialized_; }

    [[nodiscard]] const OdometryResult& lastResult() const noexcept { return last_; }
    [[nodiscard]] SessionId session() const noexcept { return session_; }
    [[nodiscard]] std::uint64_t odometrySegment() const noexcept { return segment_; }
    [[nodiscard]] virtual std::string name() const { return "OdometryBase"; }

protected:
    OdometryBase() = default;

    /// 算法本体：唯一必须实现的函数。
    virtual TrackResult track(const SensorPacket& packet) = 0;

    /// 关键帧判定。默认采纳算法的建议；可覆盖为按距离/角度/时间的策略。
    virtual bool selectKeyframe(const TrackResult& tracked, const SensorPacket& packet) {
        (void)packet;
        return tracked.suggest_keyframe;
    }

    /// 本帧已被选为关键帧。需要固化局部地图或重置预积分的实现在这里做。
    virtual void onKeyframeSelected(const OdometryResult& result) { (void)result; }

    /// 本帧的上下文：标定快照与参数版本。整帧用同一份，不要另行取快照。
    [[nodiscard]] const fw::FrameContext& context() const noexcept { return context_; }

    /// 取外参。没有或来源未定时返回空，算法要自己决定这时怎么办——
    /// 退回单位阵是一个可以做但必须自觉的选择，框架不替它做。
    [[nodiscard]] std::optional<SE3d> extrinsic(const fw::SensorName& sensor) const {
        return context_.T_body_sensor(sensor);
    }

    // ── 上一个关键帧。算法据此构造"上一关键帧 → 本帧"的相对约束 ──
    [[nodiscard]] bool hasKeyframe() const noexcept { return has_keyframe_; }
    [[nodiscard]] const SE3d& lastKeyframePose() const noexcept { return last_keyframe_pose_; }
    [[nodiscard]] Timestamp lastKeyframeTime() const noexcept { return last_keyframe_time_; }

    /// 收到后端校正，准备切换：需要重建局部地图的实现在这里做，代价自负；框架保证是在
    /// 帧边界调用。默认什么都不做（只由模块更新世界系偏移）。这里抛出，校正就不生效。
    virtual void onCorrection(const SE3d& T_world_odom, const PoseGraphData& graph) {
        (void)T_world_odom;
        (void)graph;
    }

    virtual void onReset() {}

    [[nodiscard]] const SolutionIdentity& correctionIdentity() const noexcept {
        return correction_identity_;
    }
    /// 当前生效的里程计系 → 世界系变换。由最近一次 applyCorrection 给定。
    [[nodiscard]] const SE3d& worldFromOdom() const noexcept { return T_world_odom_; }

private:
    /// 会话开始与重新初始化共用：估计状态、关键帧链与已生效的校正一并清掉。
    void restartState() {
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

    static bool allFinite(const SE3d& pose) {
        const auto t = pose.translation();
        const auto q = pose.quat();
        return std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z())
               && std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y())
               && std::isfinite(q.z());
    }

    SessionId session_{1};
    std::uint64_t frame_sequence_ = 0;
    std::uint64_t segment_ = 0;
    OdometryResult last_;
    SolutionIdentity correction_identity_;
    SE3d T_world_odom_ = SE3d::Identity();
    SE3d last_keyframe_pose_ = SE3d::Identity();
    Timestamp last_keyframe_time_{0};
    fw::FrameContext context_;
    bool has_keyframe_ = false;
    bool initialized_ = false;
    bool state_ready_ = true;
};

}  // namespace simpleslam::plugin
