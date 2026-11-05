// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file odometry_result.hpp
/// 里程计结果：发布在总线上的消息，因此属于框架契约而不是插件。
///
/// 与 `plugin::TrackResult` 的分工：后者是算法交回框架的产物（只在模块内部流转），
/// 前者是框架组装好、公布给全系统的事实。分开是为了让"哪些字段是算法说了算、
/// 哪些是框架填的"一眼可见——序号、是否关键帧由框架决定，算法填不了也不该填。
///
/// 结果**自带它成立的前提**：处理这一帧时实际生效的校正身份、由该校正得到的
/// `T_world_odom`、以及实际生效的参数与标定版本。不带这些，下游就只能按自己的
/// 接收进度去猜"这一帧当时用的是哪一份校正"——而可丢/可合并的交付策略会让这个
/// 猜测错得悄无声息：后到的校正作用到更早发布的位姿上，数字仍然像模像样。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/events.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <cstdint>
#include <optional>

namespace simpleslam::fw {

struct OdometryResult {
    /// 帧序号：本会话里跟踪了的第几帧。重新初始化不重数，所以在一个会话里唯一。
    std::uint64_t sequence = 0;
    /// 产生本结果的数据包的发布序号（`SensorPacket::sequence`）。结果据此绑定到实际的包：
    /// 帧序号只数跟踪了的帧，等待、拒绝、失败的包不占帧序号，两者不能互相推算。
    std::uint64_t packet_sequence = 0;
    /// 里程计系的段号：从 0 起，每按恢复协议重新初始化一次加一。段号不同的两个位姿
    /// 不在同一个里程计系里，不能直接相减。
    std::uint64_t odometry_segment = 0;
    Timestamp time{0};
    TrackingStatus status = TrackingStatus::kInitializing;
    SE3d T_odom_body = SE3d::Identity();
    Mat6d covariance = Mat6d::Identity();
    /// 本帧配准的信息矩阵（本帧相对局部地图）。**不是**图边权重。
    Mat6d registration_information = Mat6d::Zero();
    InformationSource registration_information_source = InformationSource::kUnset;
    /// 上一关键帧 → 本帧的相对约束。本帧被选为关键帧时，它就是那条图边。
    std::optional<RelativeConstraint> constraint_to_last_keyframe;
    bool is_keyframe = false;
    std::uint32_t iterations = 0;
    std::uint32_t inliers = 0;

    // ── 本帧成立的前提。由框架填，不由下游推断 ──
    /// 处理本帧时实际生效的 `T_world_odom`。未收到过任何校正时是单位阵。
    SE3d T_world_odom = SE3d::Identity();
    /// 产生上面那个变换的校正身份。`solution_revision == 0` 表示还没应用过校正。
    SolutionIdentity applied_correction;
    /// 处理本帧时模块看到的参数版本。"参数改了从哪一帧起生效"因此可查。
    std::uint64_t parameter_revision = 0;
    /// 处理本帧时生效的标定版本。在线标定改了之后，"这一帧用的是哪副外参"可查。
    std::uint64_t calibration_revision = 0;
    /// 估计状态推进到的时刻。通常等于 time；测量更新被拒而回滚时早于它。
    Timestamp state_time{0};
    /// 这一包缺支撑（打包器标了缺口，或超出了算法自己的连续性上限）。此时 status 至多是
    /// kDegraded——缺支撑的包不以完整包的面目出现。
    bool support_gap = false;

    /// 本帧的世界系位姿。这是当时真的输出过的值，不是事后用最新校正重算的。
    [[nodiscard]] SE3d T_world_body() const { return T_world_odom.compose(T_odom_body); }
};

}  // namespace simpleslam::fw
