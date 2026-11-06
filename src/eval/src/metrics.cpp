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
double rotationAngleRad(const SE3d& delta) {
    const Eigen::Quaterniond q = delta.quat().normalized();
    const double w = std::clamp(std::abs(q.w()), 0.0, 1.0);
    return 2.0 * std::acos(w);
}

double rotationAngleDeg(const SE3d& delta) { return rotationAngleRad(delta) * kRadToDeg; }

/// 沿参考轨迹**自身的采样**累积的路程。估计稀疏时，相邻两对之间的参考点不能跳过：
/// 只数被关联上的那几个点，量到的是弦长，不是走过的路。
class PathLength {
public:
    explicit PathLength(const Trajectory& reference)
        : samples_(reference.samples()), cumulative_(samples_.size(), 0.0) {
        for (std::size_t k = 1; k < samples_.size(); ++k) {
            cumulative_[k] =
                cumulative_[k - 1]
                + (samples_[k].pose.translation() - samples_[k - 1].pose.translation()).norm();
        }
    }

    /// 某一时刻处的累积路程：在括住它的两个样本之间线性插值。
    [[nodiscard]] double at(fw::Timestamp t) const {
        const auto later = [](fw::Timestamp value, const TrajectorySample& sample) {
            return value < sample.time;
        };
        const auto upper = std::upper_bound(samples_.begin(), samples_.end(), t, later);
        if (upper == samples_.begin()) return 0.0;
        const auto after = static_cast<std::size_t>(upper - samples_.begin());
        if (after >= samples_.size()) return cumulative_.back();
        const auto before = after - 1;
        const double span =
            static_cast<double>((samples_[after].time - samples_[before].time).count());
        const double part = static_cast<double>((t - samples_[before].time).count());
        return cumulative_[before] + (cumulative_[after] - cumulative_[before]) * (part / span);
    }

private:
    const std::vector<TrajectorySample>& samples_;
    std::vector<double> cumulative_;
};

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

double percentileOf(std::vector<double> values, double p) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double rank = std::clamp(p, 0.0, 1.0) * static_cast<double>(values.size() - 1);
    const auto lower = static_cast<std::size_t>(std::floor(rank));
    const auto upper = std::min(lower + 1, values.size() - 1);
    const double fraction = rank - static_cast<double>(lower);
    return values[lower] + (values[upper] - values[lower]) * fraction;
}

std::string toString(AlignmentKind kind) {
    switch (kind) {
        case AlignmentKind::kNone: return "none";
        case AlignmentKind::kFirstPose: return "first_pose";
        case AlignmentKind::kSe3: return "se3";
        case AlignmentKind::kSim3: return "sim3";
        case AlignmentKind::kPosYaw: return "posyaw";
    }
    return "none";
}

std::optional<AlignmentKind> parseAlignmentKind(std::string_view text) {
    if (text == "none") return AlignmentKind::kNone;
    if (text == "first_pose") return AlignmentKind::kFirstPose;
    if (text == "se3") return AlignmentKind::kSe3;
    if (text == "sim3") return AlignmentKind::kSim3;
    if (text == "posyaw") return AlignmentKind::kPosYaw;
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

    if (kind == AlignmentKind::kPosYaw) {
        // 航向至少要两对点才定得出；不够就退化成首帧对齐并如实记下来
        if (associations.size() < 2) {
            return computeAlignment(estimate, reference, associations, AlignmentKind::kFirstPose);
        }
        Vec3d source_mean = Vec3d::Zero();
        Vec3d target_mean = Vec3d::Zero();
        for (const auto& pair : associations) {
            source_mean += estimate.at(pair.estimate_index).pose.translation();
            target_mean += reference.at(pair.reference_index).pose.translation();
        }
        source_mean /= static_cast<double>(associations.size());
        target_mean /= static_cast<double>(associations.size());
        // 去掉质心之后，只在水平面上求让两组点最贴合的转角：闭式解是一个 atan2
        double dot = 0.0;
        double cross = 0.0;
        for (const auto& pair : associations) {
            const Vec3d a = estimate.at(pair.estimate_index).pose.translation() - source_mean;
            const Vec3d b = reference.at(pair.reference_index).pose.translation() - target_mean;
            dot += a.x() * b.x() + a.y() * b.y();
            cross += a.x() * b.y() - a.y() * b.x();
        }
        // 估计在水平面上挤在一点时航向定不出来：只平移
        const double yaw = (dot == 0.0 && cross == 0.0) ? 0.0 : std::atan2(cross, dot);
        const Eigen::Quaterniond rotation(Eigen::AngleAxisd(yaw, Vec3d::UnitZ()));
        alignment.transform = SE3d(Vec3d(target_mean - rotation * source_mean), rotation);
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
    // 估计的位置挤在一点时尺度定不出来（Umeyama 要除以它们的方差）：退化成刚体对齐，
    // 实际用的对齐方式如实记下
    if (with_scale && (!std::isfinite(scale) || !(scale > 0.0) || !transform.allFinite())) {
        return computeAlignment(estimate, reference, associations, AlignmentKind::kSe3);
    }
    if (!transform.allFinite()) {
        return computeAlignment(estimate, reference, associations, AlignmentKind::kFirstPose);
    }
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
        result.errors.push_back(ErrorSample{.time = estimate.at(pair.estimate_index).time,
                                            .translation = translation_errors.back(),
                                            .rotation_deg = rotation_errors.back()});
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

    // 按距离取段时，走过的路沿参考轨迹自身的采样累积
    const PathLength path(original);
    const auto distanceAt = [&](fw::Timestamp t) { return path.at(t); };
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
        result.errors.push_back(
            ErrorSample{.time = reference.at(associations[start].reference_index).time,
                        .translation = translation,
                        .rotation_deg = rotation});

        start = end;
        if (start + 1 >= associations.size()) break;
        (void)i;
    }

    result.pairs = translation_errors.size();
    result.translation = statisticsOf(std::move(translation_errors));
    result.rotation_deg = statisticsOf(std::move(rotation_errors));
    return result;
}


double KittiResult::translationMean() const noexcept {
    if (segments.empty()) return 0.0;
    double sum = 0.0;
    for (const auto& segment : segments) sum += segment.translation;
    return sum / static_cast<double>(segments.size());
}

double KittiResult::rotationMean() const noexcept {
    if (segments.empty()) return 0.0;
    double sum = 0.0;
    for (const auto& segment : segments) sum += segment.rotation_rad;
    return sum / static_cast<double>(segments.size());
}

KittiResult kittiOdometryError(const Trajectory& estimate, const Trajectory& original,
                               const KittiOptions& options) {
    if (const auto problem = trackedMismatch(estimate, original); !problem.empty()) {
        throw TrajectoryError(problem);
    }
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

    KittiResult result;
    result.pairs = associations.size();
    if (associations.size() < 2 || options.step_frames == 0) return result;

    // 关联上的每一帧处沿真值走过的路：单调不减，所以段的终点可以二分查找
    const PathLength path(original);
    std::vector<double> distance(associations.size());
    for (std::size_t i = 0; i < associations.size(); ++i) {
        distance[i] = path.at(reference.at(associations[i].reference_index).time);
    }
    for (std::size_t first = 0; first < associations.size(); first += options.step_frames) {
        for (const double length : kKittiLengths) {
            // 官方的终点：第一个走过的路严格超过段长的帧
            const auto end = std::upper_bound(distance.begin() + static_cast<std::ptrdiff_t>(first),
                                              distance.end(), distance[first] + length);
            if (end == distance.end()) break;  // 更长的段也凑不齐
            const auto last = static_cast<std::size_t>(end - distance.begin());
            const SE3d& truth_first = reference.at(associations[first].reference_index).pose;
            const SE3d& truth_last = reference.at(associations[last].reference_index).pose;
            const SE3d& estimate_first = estimate.at(associations[first].estimate_index).pose;
            const SE3d& estimate_last = estimate.at(associations[last].estimate_index).pose;
            const SE3d truth_delta = truth_first.inverse().compose(truth_last);
            const SE3d estimate_delta = estimate_first.inverse().compose(estimate_last);
            // 官方的误差：估计的相对运动之逆乘真值的相对运动
            const SE3d error = estimate_delta.inverse().compose(truth_delta);
            result.segments.push_back(
                KittiSegment{.start = reference.at(associations[first].reference_index).time,
                             .length_m = length,
                             .translation = error.translation().norm() / length,
                             .rotation_rad = rotationAngleRad(error) / length});
        }
    }
    return result;
}

}  // namespace simpleslam::eval
