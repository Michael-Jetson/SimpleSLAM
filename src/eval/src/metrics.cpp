// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/eval/metrics.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace simpleslam::eval {

namespace {

constexpr double kRadToDeg = 57.295779513082320876798154814105;

/// 相对位姿的旋转角（度）。用四元数实部，数值上比 acos(trace) 稳。
double rotationAngleDeg(const SE3d& delta) {
    const Eigen::Quaterniond q = delta.quat().normalized();
    const double w = std::clamp(std::abs(q.w()), 0.0, 1.0);
    return 2.0 * std::acos(w) * kRadToDeg;
}

}  // namespace

Statistics statisticsOf(std::vector<double> values) {
    Statistics stats;
    stats.count = values.size();
    if (values.empty()) return stats;

    std::sort(values.begin(), values.end());
    stats.min = values.front();
    stats.max = values.back();
    stats.median = values.size() % 2 == 1
                       ? values[values.size() / 2]
                       : 0.5 * (values[values.size() / 2 - 1] + values[values.size() / 2]);

    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    stats.mean = sum / static_cast<double>(values.size());

    double square_sum = 0.0;
    double variance_sum = 0.0;
    for (const double v : values) {
        square_sum += v * v;
        const double d = v - stats.mean;
        variance_sum += d * d;
    }
    stats.rmse = std::sqrt(square_sum / static_cast<double>(values.size()));
    stats.stddev = std::sqrt(variance_sum / static_cast<double>(values.size()));
    return stats;
}

std::string toString(AlignmentKind kind) {
    switch (kind) {
        case AlignmentKind::kNone: return "none";
        case AlignmentKind::kFirstPose: return "first_pose";
        case AlignmentKind::kSe3: return "se3";
        case AlignmentKind::kSim3: return "sim3";
    }
    return "none";
}

std::optional<AlignmentKind> parseAlignmentKind(std::string_view text) {
    if (text == "none") return AlignmentKind::kNone;
    if (text == "first_pose") return AlignmentKind::kFirstPose;
    if (text == "se3") return AlignmentKind::kSe3;
    if (text == "sim3") return AlignmentKind::kSim3;
    return std::nullopt;
}

Alignment computeAlignment(const Trajectory& estimate, const Trajectory& reference,
                           std::span<const Association> associations, AlignmentKind kind) {
    Alignment alignment;
    alignment.kind = kind;
    if (kind == AlignmentKind::kNone || associations.empty()) {
        alignment.kind = AlignmentKind::kNone;
        return alignment;
    }

    if (kind == AlignmentKind::kFirstPose) {
        const auto& first = associations.front();
        const SE3d& est = estimate.at(first.estimate_index).pose;
        const SE3d& ref = reference.at(first.reference_index).pose;
        alignment.transform = ref.compose(est.inverse());
        return alignment;
    }

    // Umeyama 需要至少三对点才能定出旋转；不够就退化成首帧对齐并如实记下来。
    if (associations.size() < 3) {
        return computeAlignment(estimate, reference, associations, AlignmentKind::kFirstPose);
    }

    Eigen::Matrix3Xd source(3, associations.size());
    Eigen::Matrix3Xd target(3, associations.size());
    for (std::size_t i = 0; i < associations.size(); ++i) {
        source.col(static_cast<Eigen::Index>(i)) =
            estimate.at(associations[i].estimate_index).pose.translation();
        target.col(static_cast<Eigen::Index>(i)) =
            reference.at(associations[i].reference_index).pose.translation();
    }

    const bool with_scale = kind == AlignmentKind::kSim3;
    const Eigen::Matrix4d transform = Eigen::umeyama(source, target, with_scale);
    const Eigen::Matrix3d scaled_rotation = transform.block<3, 3>(0, 0);
    const double scale = with_scale ? std::cbrt(std::abs(scaled_rotation.determinant())) : 1.0;
    const Eigen::Matrix3d rotation = scale > 0.0 ? Eigen::Matrix3d(scaled_rotation / scale)
                                                 : scaled_rotation;

    alignment.scale = scale;
    alignment.transform = SE3d(Vec3d(transform.block<3, 1>(0, 3)),
                               Eigen::Quaterniond(rotation).normalized());
    return alignment;
}

AteResult absoluteTrajectoryError(const Trajectory& estimate, const Trajectory& original,
                                  const AteOptions& options) {
    // 跟踪的传感器不同，任何左乘对齐都救不回来：直接拒绝
    if (const auto problem = trackedMismatch(estimate, original); !problem.empty()) {
        throw TrajectoryError(problem);
    }
    // 不对齐就直接比的话，两条轨迹必须已经在同一个系、同一个 gauge 下。
    // 对齐本身就是那次显式换算，所以只有 kNone 才要求系别相同。
    if (options.alignment == AlignmentKind::kNone) {
        if (const auto problem = compatibilityProblem(estimate, original); !problem.empty()) {
            throw TrajectoryError(problem);
        }
    }
    // 插值模式下先把参考插到估计的每个时刻上，之后逐帧同时刻比较
    const bool interpolate = options.association == AssociationMode::kInterpolate;
    Resampled resampled;
    if (interpolate) {
        if (estimate.clock() != original.clock()) {
            throw TrajectoryError("两条轨迹的时钟域不同，必须先显式换算");
        }
        resampled = resampleOnto(original, estimate, options.max_interpolation_gap);
    }
    const Trajectory& reference = interpolate ? resampled.trajectory : original;

    AteResult result;
    result.estimate_samples = estimate.size();
    // 覆盖率的分母始终是原始参考：插值出来的样本不是真值的采样
    result.reference_samples = original.size();

    const auto associations =
        associate(estimate, reference, interpolate ? fw::Duration{0} : options.tolerance);
    result.pairs = associations.size();
    result.reference_span = original.span();
    if (options.target_window) {
        result.target_span = *options.target_window;
        result.target_declared = true;
    }
    if (!associations.empty()) {
        result.covered_span =
            fw::TimeRange{reference.at(associations.front().reference_index).time,
                          reference.at(associations.back().reference_index).time};
        // 最近邻允许多对一，所以覆盖率的分子要数"不同的参考样本"；
        // 顺带量出相邻被关联参考样本之间的最大间隔，暴露中途整段缺失
        std::size_t previous = associations.front().reference_index;
        result.matched_reference_samples = interpolate ? resampled.brackets_used : 1;
        for (std::size_t i = 1; i < associations.size(); ++i) {
            const std::size_t current = associations[i].reference_index;
            if (current == previous) continue;
            if (!interpolate) ++result.matched_reference_samples;
            const auto gap = reference.at(current).time - reference.at(previous).time;
            if (gap > result.max_reference_gap) result.max_reference_gap = gap;
            previous = current;
        }
    }
    result.alignment = computeAlignment(estimate, reference, associations, options.alignment);
    if (associations.empty()) return result;

    std::vector<double> translation_errors;
    std::vector<double> rotation_errors;
    translation_errors.reserve(associations.size());
    rotation_errors.reserve(associations.size());

    for (const auto& pair : associations) {
        const SE3d& raw = estimate.at(pair.estimate_index).pose;
        const SE3d scaled = result.alignment.scale == 1.0
                                ? raw
                                : SE3d(Vec3d(raw.translation() * result.alignment.scale),
                                       raw.quat());
        const SE3d aligned = result.alignment.transform.compose(scaled);
        const SE3d& truth = reference.at(pair.reference_index).pose;
        const SE3d delta = truth.inverse().compose(aligned);
        translation_errors.push_back(delta.translation().norm());
        rotation_errors.push_back(rotationAngleDeg(delta));
    }

    result.translation = statisticsOf(std::move(translation_errors));
    result.rotation_deg = statisticsOf(std::move(rotation_errors));
    return result;
}

RpeResult relativePoseError(const Trajectory& estimate, const Trajectory& original,
                            const RpeOptions& options) {
    if (const auto problem = trackedMismatch(estimate, original); !problem.empty()) {
        throw TrajectoryError(problem);
    }
    RpeResult result;
    result.delta_kind = options.delta_kind;
    result.normalized = options.normalize;

    const bool interpolate = options.association == AssociationMode::kInterpolate;
    Resampled resampled;
    if (interpolate) {
        if (estimate.clock() != original.clock()) {
            throw TrajectoryError("两条轨迹的时钟域不同，必须先显式换算");
        }
        resampled = resampleOnto(original, estimate, options.max_interpolation_gap);
    }
    const Trajectory& reference = interpolate ? resampled.trajectory : original;
    const auto associations =
        associate(estimate, reference, interpolate ? fw::Duration{0} : options.tolerance);
    if (associations.size() < 2) return result;

    const Alignment alignment =
        computeAlignment(estimate, reference, associations, options.alignment);

    const auto aligned_pose = [&](std::size_t index) {
        const SE3d& raw = estimate.at(index).pose;
        const SE3d scaled = alignment.scale == 1.0
                                ? raw
                                : SE3d(Vec3d(raw.translation() * alignment.scale), raw.quat());
        return alignment.kind == AlignmentKind::kNone ? scaled
                                                      : alignment.transform.compose(scaled);
    };

    std::vector<double> translation_errors;
    std::vector<double> rotation_errors;

    // 按距离取段时，走过的路沿参考轨迹**自身的采样**累积：估计稀疏时，相邻两对之间的
    // 参考点不能跳过——只数被关联上的那几个点，量到的是弦长，不是走过的路
    const auto& original_samples = original.samples();
    std::vector<double> cumulative(original_samples.size(), 0.0);
    for (std::size_t k = 1; k < original_samples.size(); ++k) {
        cumulative[k] = cumulative[k - 1]
                        + (original_samples[k].pose.translation()
                           - original_samples[k - 1].pose.translation())
                              .norm();
    }
    // 参考轨迹上某一时刻处的累积路程：在括住它的两个样本之间线性插值
    const auto distanceAt = [&](fw::Timestamp t) {
        const auto upper = std::upper_bound(
            original_samples.begin(), original_samples.end(), t,
            [](fw::Timestamp value, const TrajectorySample& sample) { return value < sample.time; });
        if (upper == original_samples.begin()) return 0.0;
        const auto after = static_cast<std::size_t>(upper - original_samples.begin());
        if (after >= original_samples.size()) return cumulative.back();
        const auto before = after - 1;
        const double span =
            static_cast<double>((original_samples[after].time - original_samples[before].time).count());
        const double part = static_cast<double>((t - original_samples[before].time).count());
        return cumulative[before] + (cumulative[after] - cumulative[before]) * (part / span);
    };
    const auto timeOf = [&](std::size_t association) {
        return reference.at(associations[association].reference_index).time;
    };

    std::size_t start = 0;
    for (std::size_t i = 0; i < associations.size(); ++i) {
        // 找到与 start 相距一个 delta 的那一对
        double travelled = 0.0;
        std::size_t end = start;
        if (options.delta_kind == RpeDelta::kTime) {
            while (end + 1 < associations.size()
                   && (reference.at(associations[end].reference_index).time
                       - reference.at(associations[start].reference_index).time)
                          < options.delta_time) {
                ++end;
            }
            if (end == start) break;
            if ((reference.at(associations[end].reference_index).time
                 - reference.at(associations[start].reference_index).time)
                < options.delta_time) {
                break;  // 剩下的长度不足一个间隔
            }
            travelled = static_cast<double>(
                            (reference.at(associations[end].reference_index).time
                             - reference.at(associations[start].reference_index).time)
                                .count())
                        * 1e-9;
        } else {
            const double origin = distanceAt(timeOf(start));
            while (end + 1 < associations.size() && travelled < options.delta_distance_m) {
                ++end;
                travelled = distanceAt(timeOf(end)) - origin;
            }
            if (end == start || travelled < options.delta_distance_m) break;
        }

        const SE3d est_delta =
            aligned_pose(associations[start].estimate_index)
                .inverse()
                .compose(aligned_pose(associations[end].estimate_index));
        const SE3d ref_delta = reference.at(associations[start].reference_index)
                                   .pose.inverse()
                                   .compose(reference.at(associations[end].reference_index).pose);
        const SE3d error = ref_delta.inverse().compose(est_delta);

        double translation = error.translation().norm();
        double rotation = rotationAngleDeg(error);
        if (options.normalize) {
            const double divisor =
                options.delta_kind == RpeDelta::kDistance ? travelled : std::max(travelled, 1e-9);
            translation /= divisor;
            rotation /= divisor;
        }
        translation_errors.push_back(translation);
        rotation_errors.push_back(rotation);

        start = end;
        if (start + 1 >= associations.size()) break;
        (void)i;
    }

    result.pairs = translation_errors.size();
    result.translation = statisticsOf(std::move(translation_errors));
    result.rotation_deg = statisticsOf(std::move(rotation_errors));
    return result;
}

std::vector<std::pair<std::string, double>> flatten(const AteResult& ate, const RpeResult& rpe) {
    std::vector<std::pair<std::string, double>> metrics{
        {"ate.translation.rmse", ate.translation.rmse},
        {"ate.translation.mean", ate.translation.mean},
        {"ate.translation.median", ate.translation.median},
        {"ate.translation.max", ate.translation.max},
        {"ate.rotation_deg.rmse", ate.rotation_deg.rmse},
        {"ate.rotation_deg.mean", ate.rotation_deg.mean},
        {"ate.pairs", static_cast<double>(ate.pairs)},
        {"ate.coverage", ate.coverage()},
        {"ate.reference_coverage", ate.referenceCoverage()},
        {"ate.matched_reference_samples", static_cast<double>(ate.matched_reference_samples)},
        {"ate.span_ratio", ate.spanRatio()},
        {"ate.max_reference_gap_s", ate.maxReferenceGap()},
        {"ate.scale", ate.alignment.scale},
        {"rpe.translation.rmse", rpe.translation.rmse},
        {"rpe.translation.mean", rpe.translation.mean},
        {"rpe.rotation_deg.rmse", rpe.rotation_deg.rmse},
        {"rpe.rotation_deg.mean", rpe.rotation_deg.mean},
        {"rpe.pairs", static_cast<double>(rpe.pairs)},
    };
    // 任务完成度只在声明了目标窗口时才有：没有目标就没有这个数，不拿参考跨度冒充
    if (ate.target_declared) metrics.emplace_back("ate.target_coverage", ate.targetCoverage());
    return metrics;
}

}  // namespace simpleslam::eval
