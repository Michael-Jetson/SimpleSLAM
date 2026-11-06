// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file events.hpp
/// 事件描述事实，命令有明确的接收者。
///
/// - 事件（`*Event`）只说发生了什么，反应方式由消费者按自身能力决定；
/// - 请求（`*Request`）发给唯一的接收者，带请求代号防止旧任务覆盖新任务；
/// - 异步结果带完整的逻辑身份（SolutionIdentity），使"稍旧但兼容"与"身份失效"可分。
///
/// 相对约束（RelativeConstraint）在这里定义：它是图边的输入产物，
/// 与"本帧配准的信息矩阵"是两个不同的随机量，不能互相冒充（见 odometry.hpp）。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/ids.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <cstdint>

namespace simpleslam::fw {

/// 一个异步结果的逻辑身份。字段按组件分组使用，不要求每条消息都填满。
struct SolutionIdentity {
    SessionId session{};                  ///< 哪一次建图；换会话则旧结果一律失效
    std::uint64_t source_revision = 0;    ///< 消费的输入快照版本
    std::uint64_t solution_revision = 0;  ///< 第几次已发布的解
    std::uint64_t covered_until = 0;      ///< 数据覆盖到哪个关键帧序号
    std::uint64_t calibration_revision = 0;  ///< 所用标定与去畸变版本
    RequestGeneration request_generation{};  ///< 哪次请求触发

    /// 身份是否仍然有效：同会话且标定兼容。版本稍旧不算失效。
    [[nodiscard]] bool compatibleWith(SessionId current_session,
                                      std::uint64_t current_calibration) const noexcept {
        return session == current_session && calibration_revision == current_calibration;
    }
};

/// 里程计跟踪状态。系统级健康是另一层词汇，不复用这组值。
enum class TrackingStatus : std::uint8_t {
    kInitializing,
    kTracking,
    kDegraded,
    kLost,
};

/// 新关键帧已写入真源。事件只带 id 与小字段，点云从 KeyframeStore 按 id 读。
struct KeyframeEvent {
    std::uint64_t sequence = 0;  ///< 单调递增；断号即缺口，消费者补读
    KeyframeId id{};
    Timestamp time{0};
    SE3d T_odom_body = SE3d::Identity();
};

/// 扰动约定：6×6 的信息矩阵与雅可比是按哪一侧、哪种分块顺序定义的。
///
/// 不写清楚，两个模块对同一个矩阵的理解就会差一次 Adjoint 或一次分块转置，
/// 而这种错误在小位移下不会让结果发散，只会让权重悄悄失真。
enum class PerturbationSide : std::uint8_t {
    kRightBody,  ///< T ← T · Exp(δ)，δ 在本体系
    kLeftWorld,  ///< T ← Exp(δ) · T，δ 在世界系
};

enum class TangentOrder : std::uint8_t {
    kTranslationRotation,  ///< δ = [ρ; φ]
    kRotationTranslation,  ///< δ = [φ; ρ]
};

struct PerturbationConvention {
    PerturbationSide side = PerturbationSide::kRightBody;
    TangentOrder order = TangentOrder::kTranslationRotation;

    friend constexpr bool operator==(PerturbationConvention, PerturbationConvention) noexcept =
        default;
};

[[nodiscard]] constexpr const char* toString(PerturbationSide side) noexcept {
    switch (side) {
        case PerturbationSide::kRightBody: return "right_body";
        case PerturbationSide::kLeftWorld: return "left_world";
    }
    return "right_body";
}

[[nodiscard]] constexpr const char* toString(TangentOrder order) noexcept {
    switch (order) {
        case TangentOrder::kTranslationRotation: return "translation_rotation";
        case TangentOrder::kRotationTranslation: return "rotation_translation";
    }
    return "translation_rotation";
}

/// 信息矩阵的来源。启发式权重与协方差之逆不可混谈，
/// 更不能由"矩阵是否为零"反推——算法明确标注的启发式不该在入库后被改判。
enum class InformationSource : std::uint8_t {
    kUnset,       ///< 算法未给；由框架按策略补一条启发式权重
    kPropagated,  ///< 由估计器协方差传播而来（两端点之间，已跨过中间帧）
    kHeuristic,   ///< 工程权重：按距离、时间跨度或内点数拟定
    kFixed,       ///< 标定或外部给定的固定权重
};

[[nodiscard]] constexpr bool isHeuristic(InformationSource source) noexcept {
    return source != InformationSource::kPropagated;
}

/// 按扰动约定生成对角启发式信息矩阵。权重单位：平移 1/m²，旋转 1/rad²。
[[nodiscard]] inline Mat6d diagonalInformation(double translation_weight, double rotation_weight,
                                               PerturbationConvention convention) {
    Mat6d information = Mat6d::Zero();
    const bool translation_first = convention.order == TangentOrder::kTranslationRotation;
    for (int i = 0; i < 3; ++i) {
        information(i, i) = translation_first ? translation_weight : rotation_weight;
        information(i + 3, i + 3) = translation_first ? rotation_weight : translation_weight;
    }
    return information;
}

/// 两个端点之间的相对约束——图边的输入。
///
/// 这是"上一关键帧 → 本关键帧"这一对时刻之间的随机量，与单帧配准的信息矩阵不同：
/// 后者描述"本帧相对局部地图"。跨过若干非关键帧、用了局部地图或融合了 IMU 之后，
/// 两者一般不相等，因此框架不会拿单帧量当边权重，只会在算法没给时补启发式权重。
struct RelativeConstraint {
    Timestamp from_time{0};
    Timestamp to_time{0};
    SE3d measurement = SE3d::Identity();  ///< T_from_to
    Mat6d information = Mat6d::Identity();
    InformationSource source = InformationSource::kUnset;
    PerturbationConvention convention{};

    [[nodiscard]] bool heuristic() const noexcept { return isHeuristic(source); }
};

/// 一条回环约束。`id` 是这条边的稳定身份，诊断和图里的边都用它。
struct LoopConstraint {
    ConstraintId id{};
    KeyframeId query{};
    KeyframeId match{};
    SE3d T_match_query = SE3d::Identity();
    Mat6d information = Mat6d::Identity();
    InformationSource information_source = InformationSource::kHeuristic;
    PerturbationConvention convention{};
    double score = 0.0;
};

/// 里程计的处理进度：序号不大于 through 的包都有了终态。入口据此算出还有多少包在途，
/// 下游处理不过来时丢触发帧。它是**累计**的，订阅端只留最新一条就够。
struct PacketConsumed {
    std::uint64_t through = 0;
};

/// 跟踪丢失。由里程计发布，重定位是否发起由消费者决定。
struct TrackingLostEvent {
    Timestamp time{0};
    SE3d last_good_T_odom_body = SE3d::Identity();
};

/// 重定位请求。发给唯一接收者。
struct RelocationRequest {
    RequestGeneration generation{};
    KeyframeId query{};
    Timestamp time{0};
};

/// 重定位结果。必须带回请求代号与会话，跟踪恢复后旧结果据此被拒。
struct RelocationResult {
    RequestGeneration generation{};
    SessionId session{};
    bool success = false;
    SE3d T_map_body = SE3d::Identity();
    double score = 0.0;
};

/// 地图重建请求。范围由 covered_until 表达，追赶策略由接收者决定。
struct MapRebuildRequest {
    SolutionIdentity identity;
};

/// 系统级健康等级。与 TrackingStatus 分属两层，词汇不重叠。
enum class HealthLevel : std::uint8_t { kHealthy, kWarning, kCritical, kFatal };

/// 健康状态转移。影响算法决策，不能与可丢日志共用丢弃政策。
struct HealthEvent {
    Timestamp time{0};
    HealthLevel level = HealthLevel::kHealthy;
    std::uint64_t frame_sequence = 0;  ///< 从哪一帧起生效
    const char* reason = "";
};

}  // namespace simpleslam::fw
