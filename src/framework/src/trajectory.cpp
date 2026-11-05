// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/framework/trajectory.hpp>

#include <algorithm>
#include <chrono>
#include <functional>

namespace simpleslam::fw {

namespace {

void requireStrictlyIncreasing(const std::vector<TrajectorySample>& samples) {
    const auto not_increasing = [](const TrajectorySample& a, const TrajectorySample& b) {
        return b.time <= a.time;
    };
    const auto unordered = std::ranges::adjacent_find(samples, not_increasing);
    if (unordered != samples.end()) throw TrajectoryError("轨迹样本的时间必须严格递增");
}

}  // namespace

Trajectory::Trajectory(std::vector<TrajectorySample> samples) : samples_(std::move(samples)) {
    requireStrictlyIncreasing(samples_);
}

void Trajectory::push(TrajectorySample sample) {
    if (!samples_.empty() && sample.time <= samples_.back().time) {
        throw TrajectoryError("轨迹样本的时间必须严格递增");
    }
    samples_.push_back(std::move(sample));
}

double Trajectory::pathLength() const {
    double length = 0.0;
    for (std::size_t i = 1; i < samples_.size(); ++i) {
        length += (samples_[i].pose.translation() - samples_[i - 1].pose.translation()).norm();
    }
    return length;
}

std::optional<SE3d> Trajectory::interpolate(Timestamp time) const {
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

std::optional<SE3d> Trajectory::interpolateWithin(Timestamp time, Duration max_gap) const {
    if (samples_.empty()) return std::nullopt;
    if (time < samples_.front().time || time > samples_.back().time) return std::nullopt;
    const std::size_t hi = upperBound(time);
    if (hi == 0) return samples_.front().pose;  // 恰好落在第一条上
    if (samples_[hi - 1].time == time) return samples_[hi - 1].pose;
    if (hi >= samples_.size()) return std::nullopt;
    if (samples_[hi].time - samples_[hi - 1].time > max_gap) return std::nullopt;
    return interpolate(time);
}

std::optional<std::size_t> Trajectory::nearest(Timestamp time, Duration tolerance) const {
    if (samples_.empty()) return std::nullopt;
    const std::size_t hi = upperBound(time);
    std::size_t best = hi >= samples_.size() ? samples_.size() - 1 : hi;
    if (hi > 0) {
        const auto prev_gap = std::chrono::abs(time - samples_[hi - 1].time);
        const auto next_gap = hi < samples_.size() ? std::chrono::abs(samples_[hi].time - time)
                                                   : Duration::max();
        best = prev_gap <= next_gap ? hi - 1 : hi;
    }
    if (std::chrono::abs(samples_[best].time - time) > tolerance) return std::nullopt;
    return best;
}

Trajectory Trajectory::transformedBy(const SE3d& transform) const {
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

Trajectory Trajectory::scaled(double scale) const {
    Trajectory out;
    out.frame_ = frame_;
    out.clock_ = clock_;
    out.name_ = name_;
    out.samples_.reserve(samples_.size());
    for (const auto& sample : samples_) {
        out.samples_.push_back(TrajectorySample{
            sample.time, SE3d(Vec3d(sample.pose.translation() * scale), sample.pose.quat())});
    }
    return out;
}

std::size_t Trajectory::upperBound(Timestamp time) const noexcept {
    const auto it = std::ranges::upper_bound(samples_, time, std::ranges::less{},
                                             &TrajectorySample::time);
    return static_cast<std::size_t>(it - samples_.begin());
}

}  // namespace simpleslam::fw
