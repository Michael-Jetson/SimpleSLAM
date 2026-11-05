// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/eval/association.hpp>

#include <algorithm>
#include <format>
#include <functional>
#include <utility>

namespace simpleslam::eval {

namespace {

[[nodiscard]] std::string clockMismatch(const Trajectory& a, const Trajectory& b) {
    return std::format("两条轨迹的时钟域不同（{} 与 {}），必须先显式换算", a.clock().value,
                       b.clock().value);
}

}  // namespace

Resampled resampleOnto(const Trajectory& reference, const Trajectory& estimate,
                       fw::Duration max_gap) {
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
        const auto hi = std::ranges::upper_bound(samples, sample.time, std::ranges::less{},
                                                 &TrajectorySample::time);
        const auto index = static_cast<std::size_t>(hi - samples.begin());
        if (index == 0) continue;  // 不会发生：插值成功意味着不早于第一条样本
        used[index - 1] = true;
        if (index < samples.size() && samples[index - 1].time != sample.time) used[index] = true;
    }
    out.brackets_used = static_cast<std::size_t>(std::ranges::count(used, true));
    return out;
}

Trajectory rightMultiplied(const Trajectory& trajectory, const SE3d& T_old_new,
                           std::string new_tracked) {
    Trajectory out;
    for (const auto& sample : trajectory) out.push(sample.time, sample.pose.compose(T_old_new));
    out.setFrame(trajectory.frame());
    out.setClock(trajectory.clock());
    out.setName(trajectory.name());
    out.setTracked(std::move(new_tracked));
    return out;
}

std::string trackedMismatch(const Trajectory& a, const Trajectory& b) {
    if (a.tracked().empty() || b.tracked().empty() || a.tracked() == b.tracked()) return {};
    return std::format("两条轨迹跟踪的传感器不同（{} 与 {}），先用外参把其中一条转换过去",
                       a.tracked(), b.tracked());
}

std::string compatibilityProblem(const Trajectory& a, const Trajectory& b) {
    if (a.clock() != b.clock()) return clockMismatch(a, b);
    if (a.frame() != b.frame() && a.frame() != TrajectoryFrame::kUnknown
        && b.frame() != TrajectoryFrame::kUnknown) {
        return "两条轨迹的参考系不同，不对齐就不能直接比";
    }
    return {};
}

std::vector<Association> associate(const Trajectory& estimate, const Trajectory& reference,
                                   fw::Duration tolerance) {
    if (estimate.clock() != reference.clock()) {
        throw TrajectoryError(clockMismatch(estimate, reference));
    }
    std::vector<Association> pairs;
    pairs.reserve(estimate.size());
    for (std::size_t i = 0; i < estimate.size(); ++i) {
        const auto& sample = estimate.at(i);
        const auto match = reference.nearest(sample.time, tolerance);
        if (!match) continue;
        pairs.push_back(Association{.estimate_index = i,
                                    .reference_index = *match,
                                    .estimate_time = sample.time,
                                    .reference_time = reference.at(*match).time});
    }
    return pairs;
}

}  // namespace simpleslam::eval
