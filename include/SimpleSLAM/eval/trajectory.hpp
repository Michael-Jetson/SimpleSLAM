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

#include <algorithm>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace simpleslam::eval {

namespace fw = simpleslam::fw;

struct TrajectorySample {
    fw::Timestamp time{0};
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
    explicit Trajectory(std::vector<TrajectorySample> samples) : samples_(std::move(samples)) {
        requireSorted();
    }

    /// 追加一个样本。时间必须严格递增——评测输入允许重复时间戳是错误之源。
    void push(TrajectorySample sample) {
        if (!samples_.empty() && sample.time <= samples_.back().time) {
            throw TrajectoryError("轨迹样本的时间必须严格递增");
        }
        samples_.push_back(std::move(sample));
    }
    void push(fw::Timestamp time, const SE3d& pose) { push(TrajectorySample{time, pose}); }

    [[nodiscard]] std::size_t size() const noexcept { return samples_.size(); }
    [[nodiscard]] bool empty() const noexcept { return samples_.empty(); }
    [[nodiscard]] const TrajectorySample& at(std::size_t i) const { return samples_.at(i); }
    [[nodiscard]] const std::vector<TrajectorySample>& samples() const noexcept { return samples_; }
    [[nodiscard]] auto begin() const noexcept { return samples_.begin(); }
    [[nodiscard]] auto end() const noexcept { return samples_.end(); }

    [[nodiscard]] TrajectoryFrame frame() const noexcept { return frame_; }
    void setFrame(TrajectoryFrame frame) noexcept { frame_ = frame; }
    [[nodiscard]] fw::ClockDomainId clock() const noexcept { return clock_; }
    void setClock(fw::ClockDomainId clock) noexcept { clock_ = clock; }
    [[nodiscard]] const std::string& name() const noexcept { return name_; }
    void setName(std::string name) { name_ = std::move(name); }
    /// 跟踪的传感器坐标系（位姿的右侧）。空表示未声明，由使用者负责。
    [[nodiscard]] const std::string& tracked() const noexcept { return tracked_; }
    void setTracked(std::string tracked) { tracked_ = std::move(tracked); }

    [[nodiscard]] fw::TimeRange span() const noexcept {
        if (samples_.empty()) return {};
        return fw::TimeRange{samples_.front().time, samples_.back().time};
    }

    /// 轨迹长度（平移路径积分）。RPE 的"每米误差"用它做分母。
    [[nodiscard]] double pathLength() const {
        double length = 0.0;
        for (std::size_t i = 1; i < samples_.size(); ++i) {
            length += (samples_[i].pose.translation() - samples_[i - 1].pose.translation()).norm();
        }
        return length;
    }

    /// 在李群上线性插值。区间外返回空——外推不是插值，不要悄悄做。
    [[nodiscard]] std::optional<SE3d> interpolate(fw::Timestamp time) const {
        if (samples_.empty()) return std::nullopt;
        if (time < samples_.front().time || time > samples_.back().time) return std::nullopt;
        const std::size_t hi = upperBound(time);
        if (hi == 0) return samples_.front().pose;
        if (hi >= samples_.size()) return samples_.back().pose;
        const auto& a = samples_[hi - 1];
        const auto& b = samples_[hi];
        const double denom = static_cast<double>((b.time - a.time).count());
        if (denom <= 0.0) return a.pose;
        const double alpha = static_cast<double>((time - a.time).count()) / denom;
        SE3d::Tangent delta = b.pose.rminus(a.pose);
        delta.coeffs() *= alpha;
        return a.pose.rplus(delta);
    }

    /// 在 time 处插值，但只在两侧括点都存在、且括点间隔不超过 max_gap 时才插。
    ///
    /// 括点离得太远时插出来的位姿没有依据：两条样本之间车可能已经拐过弯。
    /// 这时返回空，调用方把这一帧记为"无参考"，而不是拿一个编出来的位姿去比。
    [[nodiscard]] std::optional<SE3d> interpolateWithin(fw::Timestamp time,
                                                        fw::Duration max_gap) const {
        if (samples_.empty()) return std::nullopt;
        if (time < samples_.front().time || time > samples_.back().time) return std::nullopt;
        const std::size_t hi = upperBound(time);
        if (hi == 0) return samples_.front().pose;  // 恰好落在第一条上
        if (samples_[hi - 1].time == time) return samples_[hi - 1].pose;
        if (hi >= samples_.size()) return std::nullopt;
        if (samples_[hi].time - samples_[hi - 1].time > max_gap) return std::nullopt;
        return interpolate(time);
    }

    /// 最近邻样本的下标；超出容差返回空。评测的时间关联用它。
    [[nodiscard]] std::optional<std::size_t> nearest(fw::Timestamp time,
                                                     fw::Duration tolerance) const {
        if (samples_.empty()) return std::nullopt;
        const std::size_t hi = upperBound(time);
        std::size_t best = hi >= samples_.size() ? samples_.size() - 1 : hi;
        if (hi > 0) {
            const auto prev_gap = absDuration(time - samples_[hi - 1].time);
            const auto next_gap = hi < samples_.size() ? absDuration(samples_[hi].time - time)
                                                       : fw::Duration::max();
            best = prev_gap <= next_gap ? hi - 1 : hi;
        }
        if (absDuration(samples_[best].time - time) > tolerance) return std::nullopt;
        return best;
    }

    /// 左乘一个变换（换参考系 / 对齐）。
    [[nodiscard]] Trajectory transformedBy(const SE3d& transform) const {
        Trajectory out;
        out.frame_ = frame_;
        out.clock_ = clock_;
        out.name_ = name_;
        out.tracked_ = tracked_;
        out.samples_.reserve(samples_.size());
        for (const auto& sample : samples_) {
            out.samples_.push_back(TrajectorySample{sample.time, transform.compose(sample.pose)});
        }
        return out;
    }

    /// 只缩放平移（Sim3 对齐的尺度部分）。旋转不变。
    [[nodiscard]] Trajectory scaled(double scale) const {
        Trajectory out;
        out.frame_ = frame_;
        out.clock_ = clock_;
        out.name_ = name_;
        out.samples_.reserve(samples_.size());
        for (const auto& sample : samples_) {
            out.samples_.push_back(
                TrajectorySample{sample.time, SE3d(Vec3d(sample.pose.translation() * scale),
                                                   sample.pose.quat())});
        }
        return out;
    }

    static fw::Duration absDuration(fw::Duration d) noexcept {
        return d.count() < 0 ? -d : d;
    }

private:
    void requireSorted() const {
        for (std::size_t i = 1; i < samples_.size(); ++i) {
            if (samples_[i].time <= samples_[i - 1].time) {
                throw TrajectoryError("轨迹样本的时间必须严格递增");
            }
        }
    }

    /// 第一个时间大于 time 的下标。
    [[nodiscard]] std::size_t upperBound(fw::Timestamp time) const noexcept {
        std::size_t lo = 0;
        std::size_t hi = samples_.size();
        while (lo < hi) {
            const std::size_t mid = lo + (hi - lo) / 2;
            if (samples_[mid].time <= time) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    std::vector<TrajectorySample> samples_;
    TrajectoryFrame frame_ = TrajectoryFrame::kUnknown;
    fw::ClockDomainId clock_{};
    std::string name_;
    std::string tracked_;
};

/// 一对已关联的样本。保留各自的时刻，便于事后检查关联是否勉强。
struct Association {
    std::size_t estimate_index = 0;
    std::size_t reference_index = 0;
    fw::Timestamp estimate_time{0};
    fw::Timestamp reference_time{0};

    [[nodiscard]] fw::Duration gap() const noexcept {
        return Trajectory::absDuration(estimate_time - reference_time);
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

[[nodiscard]] inline Resampled resampleOnto(const Trajectory& reference,
                                            const Trajectory& estimate, fw::Duration max_gap) {
    Resampled out;
    out.trajectory.setFrame(reference.frame());
    out.trajectory.setClock(reference.clock());
    out.trajectory.setName(reference.name());
    out.trajectory.setTracked(reference.tracked());
    std::vector<bool> used(reference.size(), false);
    const auto& samples = reference.samples();
    for (const auto& sample : estimate) {
        const auto pose = reference.interpolateWithin(sample.time, max_gap);
        if (!pose) continue;
        out.trajectory.push(sample.time, *pose);
        // 记下这次用到的括点：第一个晚于该时刻的样本及其前一个
        const auto hi = std::upper_bound(samples.begin(), samples.end(), sample.time,
                                         [](fw::Timestamp t, const TrajectorySample& s) {
                                             return t < s.time;
                                         });
        const auto index = static_cast<std::size_t>(hi - samples.begin());
        if (index == 0) continue;  // 不会发生：插值成功意味着不早于第一条样本
        used[index - 1] = true;
        if (index < samples.size() && samples[index - 1].time != sample.time) used[index] = true;
    }
    for (const bool flag : used) out.brackets_used += flag ? 1 : 0;
    return out;
}

/// 把轨迹的右侧换成另一个传感器：`T_ref_new = T_ref_old · T_old_new`。
///
/// 估计跟踪 body、真值跟踪相机时，先用 body→相机的外参把估计转到相机上再比；
/// 反过来也行。转换是显式的一步，结果的 tracked 随之改名。
[[nodiscard]] inline Trajectory rightMultiplied(const Trajectory& trajectory,
                                                const SE3d& T_old_new, std::string new_tracked) {
    Trajectory out;
    for (const auto& sample : trajectory) out.push(sample.time, sample.pose.compose(T_old_new));
    out.setFrame(trajectory.frame());
    out.setClock(trajectory.clock());
    out.setName(trajectory.name());
    out.setTracked(std::move(new_tracked));
    return out;
}

/// 两条轨迹跟踪的是不是同一个传感器。这一条与对不对齐无关：左乘对齐消不掉右侧外参。
[[nodiscard]] inline std::string trackedMismatch(const Trajectory& a, const Trajectory& b) {
    if (a.tracked().empty() || b.tracked().empty() || a.tracked() == b.tracked()) return {};
    return "两条轨迹跟踪的传感器不同（" + a.tracked() + " 与 " + b.tracked()
           + "），先用外参把其中一条转换过去";
}

/// 两条轨迹**不经对齐**能否直接比较。不能比时返回原因，可比时返回空串。
///
/// 时钟域不同就是在比两把不同的尺子；参考系不同（world 与 odom）就是在比两个
/// 不同 gauge 下的位姿。后者可以靠对齐消掉，所以只在"不对齐"时才是错误。
/// kUnknown 表示外部数据、系别由使用者负责，不在此拦截。
[[nodiscard]] inline std::string compatibilityProblem(const Trajectory& a, const Trajectory& b) {
    if (a.clock() != b.clock()) {
        return "两条轨迹的时钟域不同（" + std::to_string(a.clock().value) + " 与 "
               + std::to_string(b.clock().value) + "），必须先显式换算";
    }
    if (a.frame() != b.frame() && a.frame() != TrajectoryFrame::kUnknown
        && b.frame() != TrajectoryFrame::kUnknown) {
        return "两条轨迹的参考系不同，不对齐就不能直接比";
    }
    return {};
}

/// 时间关联：对估计的每个样本，在参考里找最近且不超过容差的一个。
///
/// 不做插值：插值会把参考轨迹的采样稀疏度藏起来，让稀疏真值看着很准。
/// 需要插值的场合显式调用 Trajectory::interpolate。
///
/// 时钟域在这里硬校验：把它写成字段却不检查，等于把注释当成了约束。
/// 参考系不在这里拦——两条不同系的轨迹正是要靠对齐来比，那是 ATE 的事
/// （见 absoluteTrajectoryError）。
[[nodiscard]] inline std::vector<Association> associate(const Trajectory& estimate,
                                                        const Trajectory& reference,
                                                        fw::Duration tolerance) {
    if (estimate.clock() != reference.clock()) {
        throw TrajectoryError("两条轨迹的时钟域不同（" + std::to_string(estimate.clock().value)
                              + " 与 " + std::to_string(reference.clock().value)
                              + "），必须先显式换算");
    }
    std::vector<Association> pairs;
    pairs.reserve(estimate.size());
    for (std::size_t i = 0; i < estimate.size(); ++i) {
        const auto& sample = estimate.at(i);
        const auto match = reference.nearest(sample.time, tolerance);
        if (!match) continue;
        pairs.push_back(Association{i, *match, sample.time, reference.at(*match).time});
    }
    return pairs;
}

}  // namespace simpleslam::eval
