// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file trajectory.hpp
/// 轨迹：按时间排序的位姿序列，评测与导出的共同输入。
///
/// 一条轨迹自带三样语义：位姿左侧的参考系（world 还是 odom）、位姿右侧**跟踪的
/// 传感器**（body、激光还是相机）、时钟域。评测最容易出错的地方不是算 RMSE，
/// 而是拿两条不同系、不同时钟、跟踪不同传感器的轨迹去比——所以这些是字段，不是注释。
///
/// 左右两侧的差别要紧：世界系的 SE3 对齐是左乘，只能消掉参考系的差异；跟踪的传感器
/// 不同是右侧差一个外参，转弯时它表现为随航向变化的杆臂误差，任何左乘对齐都消不掉。
///
/// 插值在李群上做（rplus + rminus），不是分别插值平移与四元数再拼回去。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace simpleslam::fw {

struct TrajectorySample {
    Timestamp time{0};
    SE3d pose = SE3d::Identity();
};

/// 轨迹所在的参考系。比较两条轨迹前必须一致，或者显式对齐过。
enum class TrajectoryFrame : std::uint8_t {
    kWorld,   ///< 全局系（回环校正之后）
    kOdom,    ///< 里程计系（只有前端，无全局校正）
    kUnknown, ///< 外部数据，系别由使用者负责
};

class TrajectoryError : public std::runtime_error {
public:
    explicit TrajectoryError(const std::string& what) : std::runtime_error(what) {}
};

class Trajectory {
public:
    Trajectory() = default;
    /// 样本的时间必须严格递增，否则抛 TrajectoryError。
    explicit Trajectory(std::vector<TrajectorySample> samples);

    /// 追加一个样本。时间必须严格递增——评测输入允许重复时间戳是错误之源。
    void push(TrajectorySample sample);
    void push(Timestamp time, const SE3d& pose) { push(TrajectorySample{time, pose}); }

    [[nodiscard]] std::size_t size() const noexcept { return samples_.size(); }
    [[nodiscard]] bool empty() const noexcept { return samples_.empty(); }
    [[nodiscard]] const TrajectorySample& at(std::size_t i) const { return samples_.at(i); }
    [[nodiscard]] const std::vector<TrajectorySample>& samples() const noexcept { return samples_; }
    [[nodiscard]] auto begin() const noexcept { return samples_.begin(); }
    [[nodiscard]] auto end() const noexcept { return samples_.end(); }

    [[nodiscard]] TrajectoryFrame frame() const noexcept { return frame_; }
    void setFrame(TrajectoryFrame frame) noexcept { frame_ = frame; }
    [[nodiscard]] ClockDomainId clock() const noexcept { return clock_; }
    void setClock(ClockDomainId clock) noexcept { clock_ = clock; }
    [[nodiscard]] const std::string& name() const noexcept { return name_; }
    void setName(std::string name) { name_ = std::move(name); }
    /// 跟踪的传感器坐标系（位姿的右侧）。空表示未声明，由使用者负责。
    [[nodiscard]] const std::string& tracked() const noexcept { return tracked_; }
    void setTracked(std::string tracked) { tracked_ = std::move(tracked); }

    [[nodiscard]] TimeRange span() const noexcept {
        if (samples_.empty()) return {};
        return TimeRange{samples_.front().time, samples_.back().time};
    }

    /// 轨迹长度（平移路径积分）。RPE 的"每米误差"用它做分母。
    [[nodiscard]] double pathLength() const;

    /// 在李群上线性插值。区间外返回空——外推不是插值，不要悄悄做。
    [[nodiscard]] std::optional<SE3d> interpolate(Timestamp time) const;

    /// 在 time 处插值，但只在两侧括点都存在、且括点间隔不超过 max_gap 时才插。
    ///
    /// 括点离得太远时插出来的位姿没有依据：两条样本之间车可能已经拐过弯。
    /// 这时返回空，调用方把这一帧记为"无参考"，而不是拿一个编出来的位姿去比。
    [[nodiscard]] std::optional<SE3d> interpolateWithin(Timestamp time, Duration max_gap) const;

    /// 最近邻样本的下标；超出容差返回空。评测的时间关联用它。
    [[nodiscard]] std::optional<std::size_t> nearest(Timestamp time, Duration tolerance) const;

    /// 左乘一个变换（换参考系 / 对齐）。
    [[nodiscard]] Trajectory transformedBy(const SE3d& transform) const;

    /// 只缩放平移（Sim3 对齐的尺度部分）。旋转不变；参考系、时钟、跟踪的传感器照抄。
    [[nodiscard]] Trajectory scaled(double scale) const;

private:
    /// 第一个时间大于 time 的下标。
    [[nodiscard]] std::size_t upperBound(Timestamp time) const noexcept;

    std::vector<TrajectorySample> samples_;
    TrajectoryFrame frame_ = TrajectoryFrame::kUnknown;
    ClockDomainId clock_{};
    std::string name_;
    std::string tracked_;
};

}  // namespace simpleslam::fw
