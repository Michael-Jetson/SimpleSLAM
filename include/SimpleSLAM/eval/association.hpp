// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file association.hpp
/// 两条轨迹的关联与重采样：算误差之前，先把估计和参考的样本对上。
///
/// 轨迹本身是框架的数据类型（`framework/trajectory.hpp`），这里只是评测在它之上的操作。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/time.hpp>
#include <SimpleSLAM/framework/trajectory.hpp>

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

namespace simpleslam::eval {

namespace fw = simpleslam::fw;

using fw::Trajectory;
using fw::TrajectoryError;
using fw::TrajectoryFrame;
using fw::TrajectorySample;

/// 一对已关联的样本。保留各自的时刻，便于事后检查关联是否勉强。
struct Association {
    std::size_t estimate_index = 0;
    std::size_t reference_index = 0;
    fw::Timestamp estimate_time{0};
    fw::Timestamp reference_time{0};

    [[nodiscard]] fw::Duration gap() const noexcept {
        return std::chrono::abs(estimate_time - reference_time);
    }
};

/// 把参考轨迹插值到估计的每个时刻上，得到一条与估计逐帧同时刻的参考。
///
/// 参考与估计的采样时刻不同步时（KITTI raw：估计在扫描终点，真值在 OXTS 时刻），
/// 最近邻关联是在拿相邻时刻的位置冒充同一时刻的位置——在 KITTI 的车速下，
/// 差 50 ms 就是半米多。放宽最近邻容差只会把这个错误合法化。插值才是显式换算：
/// 每个估计时刻都用两侧括点在李群上插值，括点间隔超过 max_gap 或缺一侧括点的帧
/// 不参与比较。`brackets_used` 是被用作括点的原始参考样本数。
struct Resampled {
    Trajectory trajectory;
    std::size_t brackets_used = 0;
};

[[nodiscard]] Resampled resampleOnto(const Trajectory& reference, const Trajectory& estimate,
                                     fw::Duration max_gap);

/// 把轨迹的右侧换成另一个传感器：`T_ref_new = T_ref_old · T_old_new`。
///
/// 估计跟踪 body、真值跟踪相机时，先用 body→相机的外参把估计转到相机上再比；
/// 反过来也行。转换是显式的一步，结果的 tracked 随之改名。
[[nodiscard]] Trajectory rightMultiplied(const Trajectory& trajectory, const SE3d& T_old_new,
                                         std::string new_tracked);

/// 两条轨迹跟踪的是不是同一个传感器。这一条与对不对齐无关：左乘对齐消不掉右侧外参。
[[nodiscard]] std::string trackedMismatch(const Trajectory& a, const Trajectory& b);

/// 两条轨迹**不经对齐**能否直接比较。不能比时返回原因，可比时返回空串。
///
/// 时钟域不同就是在比两把不同的尺子；参考系不同（world 与 odom）就是在比两个
/// 不同 gauge 下的位姿。后者可以靠对齐消掉，所以只在"不对齐"时才是错误。
/// kUnknown 表示外部数据、系别由使用者负责，不在此拦截。
[[nodiscard]] std::string compatibilityProblem(const Trajectory& a, const Trajectory& b);

/// 时间关联：对估计的每个样本，在参考里找最近且不超过容差的一个。
///
/// 不做插值：插值会把参考轨迹的采样稀疏度藏起来，让稀疏真值看着很准。
/// 需要插值的场合显式调用 Trajectory::interpolate。
///
/// 时钟域在这里硬校验：把它写成字段却不检查，等于把注释当成了约束。
/// 参考系不在这里拦——两条不同系的轨迹正是要靠对齐来比，那是 ATE 的事
/// （见 absoluteTrajectoryError）。
[[nodiscard]] std::vector<Association> associate(const Trajectory& estimate,
                                                 const Trajectory& reference,
                                                 fw::Duration tolerance);

}  // namespace simpleslam::eval
