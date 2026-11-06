// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file trajectory_module.hpp
/// 轨迹收集：把运行中的结果攒成可导出、可评测的轨迹。
///
/// 收三条，不是一条，因为它们回答的是三个不同的问题：
///
/// - **里程计轨迹**（odom 系）：前端自己走出来的，不受全局校正影响。
///   问"前端漂了多少"看它。
/// - **在线轨迹**（world 系）：直接取里程计结果自带的 `T_world_odom`。
///   这条轨迹是**事实记录**，不是本模块推算出来的：哪一帧用了哪份校正，
///   只有产出该帧的里程计模块知道。本模块自己订阅校正再按接收进度换算，
///   会被 Latest 合并带偏："后到的校正作用到更早发布的位姿上"，数字仍然像模像样。
///   也不能用最终的 T_world_odom 回头重算：那会得出一条系统从未输出过的轨迹。
/// - **关键帧轨迹**（world 系，优化后）：从位姿图按需生成。
///   问"建图最终精度"看它，这也是通常拿去和真值比的那条。

#include <SimpleSLAM/comm/module.hpp>
#include <SimpleSLAM/framework/trajectory.hpp>
#include <SimpleSLAM/framework/odometry_result.hpp>
#include <SimpleSLAM/modules/topics.hpp>

#include <cstdint>

namespace simpleslam::modules {

/// 把里程计结果攒成里程计轨迹与在线轨迹。轨迹模块用它，在线评测也用它：两边攒法相同。
class TrajectoryRecorder {
public:
    /// 里程计输出的是 body 的位姿：跟踪的传感器写明，评测才能拒绝拿它去比相机真值。
    TrajectoryRecorder();

    /// 收一条结果：初始化中的不收；时间不严格递增的跳过并计数。
    void add(const fw::OdometryResult& result);

    [[nodiscard]] const fw::Trajectory& odometry() const noexcept { return odometry_; }
    [[nodiscard]] const fw::Trajectory& online() const noexcept { return online_; }
    /// 在线轨迹里有多少帧带着一份已生效的校正。
    [[nodiscard]] std::uint64_t corrected() const noexcept { return corrected_; }
    [[nodiscard]] std::uint64_t skipped() const noexcept { return skipped_; }
    /// 收到的里程计结果数。
    [[nodiscard]] std::uint64_t received() const noexcept { return received_; }
    /// 最后一帧采用的校正身份。
    [[nodiscard]] const fw::SolutionIdentity& lastCorrection() const noexcept {
        return last_correction_;
    }

private:
    fw::Trajectory odometry_;
    fw::Trajectory online_;
    fw::SolutionIdentity last_correction_;
    std::uint64_t corrected_ = 0;
    std::uint64_t skipped_ = 0;
    std::uint64_t received_ = 0;
};

/// 优化后的关键帧轨迹：从位姿图的快照生成，哪个线程上调都安全。
[[nodiscard]] fw::Trajectory keyframeTrajectory(const SlamResources& resources);

class TrajectoryModule final : public comm::Module {
public:
    explicit TrajectoryModule(SlamResources& resources);

    [[nodiscard]] const fw::Trajectory& odometry() const noexcept { return recorder_.odometry(); }
    [[nodiscard]] const fw::Trajectory& online() const noexcept { return recorder_.online(); }

    /// 优化后的关键帧轨迹。按需从位姿图生成，不常驻内存。
    [[nodiscard]] fw::Trajectory keyframes() const { return keyframeTrajectory(*resources_); }

    /// 在线轨迹里有多少帧带着一份已生效的校正。
    [[nodiscard]] std::uint64_t corrected() const noexcept { return recorder_.corrected(); }
    [[nodiscard]] std::uint64_t skipped() const noexcept { return recorder_.skipped(); }
    /// 收到的里程计结果数。与里程计发布的结果数对账：少了就是轨迹缺帧。
    [[nodiscard]] std::uint64_t received() const noexcept { return recorder_.received(); }
    /// 最后一帧采用的校正身份。事实记录，不是本模块的推断。
    [[nodiscard]] const fw::SolutionIdentity& lastCorrection() const noexcept {
        return recorder_.lastCorrection();
    }

protected:
    /// 只订阅里程计结果：校正的生效时机写在结果里，不必也不该在这里重建。
    void onInit() override;

private:
    SlamResources* resources_;
    TrajectoryRecorder recorder_;
};

}  // namespace simpleslam::modules
