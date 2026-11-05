// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file metrics.hpp
/// 轨迹精度指标：ATE 与 RPE。
///
/// 两条纪律：
/// 1. **对齐方式是结果的一部分**。同一条轨迹按 SE3 对齐与按 Sim3 对齐，
///    ATE 可以差一个量级。所以 AteResult 里带着用了哪种对齐、解出的尺度是多少，
///    报告里不会只剩一个孤零零的数字。
/// 2. **平移与旋转分开统计**。把它们加权成一个标量需要一个长度尺度，
///    那个尺度是任意的；框架不替使用者选。

#include <SimpleSLAM/eval/trajectory.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace simpleslam::eval {

/// 一组误差的统计量。count 一并给出：样本太少时 RMSE 没有意义。
struct Statistics {
    std::size_t count = 0;
    double rmse = 0.0;
    double mean = 0.0;
    double median = 0.0;
    double stddev = 0.0;
    double min = 0.0;
    double max = 0.0;
};

[[nodiscard]] Statistics statisticsOf(std::vector<double> values);

/// 对齐方式。评测报告必须写明用了哪一种。
enum class AlignmentKind : std::uint8_t {
    kNone,       ///< 不对齐：两条轨迹已在同一系同一 gauge
    kFirstPose,  ///< 用首帧对齐：估计的首帧搬到参考的首帧
    kSe3,        ///< Umeyama 刚体最小二乘对齐
    kSim3,       ///< Umeyama 带尺度对齐（单目等尺度不可观的情形）
};

[[nodiscard]] std::string toString(AlignmentKind kind);
/// 解析对齐方式。认不出来返回空——默认成某一种会让报告说谎。
[[nodiscard]] std::optional<AlignmentKind> parseAlignmentKind(std::string_view text);

struct Alignment {
    AlignmentKind kind = AlignmentKind::kNone;
    SE3d transform = SE3d::Identity();  ///< 作用在估计上：aligned = transform ∘ (scale · est)
    double scale = 1.0;
};

/// 求对齐变换。associations 为空时返回恒等对齐。
[[nodiscard]] Alignment computeAlignment(const Trajectory& estimate, const Trajectory& reference,
                                         const std::vector<Association>& associations,
                                         AlignmentKind kind);

/// 估计与参考怎么配对。
///
/// kNearest：各取最近的一条，超出容差不配。只在两边采样时刻本就对齐时成立。
/// kInterpolate：把参考插值到估计的每个时刻（resampleOnto），括点间隔超过
/// max_interpolation_gap 或缺括点的帧不参与。两边采样不同步时用它——
/// 放宽最近邻容差是拿相邻时刻的位置冒充同一时刻。
enum class AssociationMode : std::uint8_t { kNearest, kInterpolate };

[[nodiscard]] constexpr const char* toString(AssociationMode mode) noexcept {
    return mode == AssociationMode::kInterpolate ? "interpolate" : "nearest";
}

struct AteOptions {
    fw::Duration tolerance = std::chrono::milliseconds(20);
    AlignmentKind alignment = AlignmentKind::kSe3;
    AssociationMode association = AssociationMode::kNearest;
    fw::Duration max_interpolation_gap = std::chrono::milliseconds(150);
    /// 评测的**目标窗口**：这次运行应当产出位姿的时间范围（来源描述的
    /// evaluation_window）。给了它，任务完成度按它算；参考轨迹为插值多留的括点支撑
    /// 不在其内，不能把"没用到的支撑样本"算成"没完成的任务"。
    std::optional<fw::TimeRange> target_window;
};

struct AteResult {
    Statistics translation;      ///< 米
    Statistics rotation_deg;     ///< 度
    Alignment alignment;
    std::size_t pairs = 0;
    std::size_t estimate_samples = 0;
    std::size_t reference_samples = 0;
    /// 关联覆盖到的参考时间区间，以及参考轨迹自身的时间区间。
    fw::TimeRange covered_span{};
    fw::TimeRange reference_span{};
    /// 声明过的目标窗口。没声明时 target_declared 为 false，任务完成度无从计算。
    fw::TimeRange target_span{};
    bool target_declared = false;

    /// 被关联到的**不同**参考样本数。关联是最近邻、允许多对一，所以它不等于 pairs。
    /// 它衡量的是"参考样本用了多少"，不是"任务完成了多少"——括点支撑样本本来就不会
    /// 被关联上。
    std::size_t matched_reference_samples = 0;
    /// 关联覆盖区间内，相邻两个被关联参考样本之间的最大间隔。
    /// 中间整段掉线时，跨度比例仍然漂亮，这个数会原形毕露。
    fw::Duration max_reference_gap{0};

    /// **关联率**：已输出的估计里有多少能关联到参考。太低说明时间对不上。
    ///
    /// 它不是任务完成率：算法提前退出、只输出开头一小段，关联率照样可以是 100%。
    /// 要回答"跑完了没有"，看 `spanRatio()` 和 `maxReferenceGap()`。
    [[nodiscard]] double coverage() const noexcept {
        return estimate_samples == 0 ? 0.0
                                     : static_cast<double>(pairs)
                                           / static_cast<double>(estimate_samples);
    }

    /// **参考侧覆盖率**：参考轨迹里有多少个样本被关联上了，值域 [0, 1]。
    ///
    /// 分子必须是**不同**的参考样本数：最近邻允许多个估计落到同一个参考样本上，
    /// 用 pairs 作分子会算出大于 1 的覆盖率——11 个估计挤在 2 个参考样本上会得到 5.5。
    [[nodiscard]] double referenceCoverage() const noexcept {
        return reference_samples == 0 ? 0.0
                                      : static_cast<double>(matched_reference_samples)
                                            / static_cast<double>(reference_samples);
    }

    /// **参考跨度利用率**：关联覆盖的时间跨度占参考轨迹自身跨度的比例。
    ///
    /// 参考轨迹常常比任务长（插值要的括点、数据集的整段真值），所以它不是任务完成度；
    /// 任务完成度看 `targetCoverage()`。只衡量首尾：要判断中间有没有整段缺失，
    /// 配合 `maxReferenceGap()` 一起看。
    [[nodiscard]] double spanRatio() const noexcept {
        const double total = static_cast<double>(reference_span.span().count());
        if (total <= 0.0) return 0.0;
        return static_cast<double>(covered_span.span().count()) / total;
    }

    /// **任务完成度**：关联覆盖的时间跨度与目标窗口的交集，占目标窗口的比例。
    /// 没声明目标窗口时为 0，且 target_declared 为 false——不拿参考跨度冒充。
    [[nodiscard]] double targetCoverage() const noexcept {
        if (!target_declared || pairs == 0) return 0.0;
        if (target_span.empty()) {
            // 单帧任务：目标是一个时刻，盖住它就是完成
            return covered_span.begin <= target_span.begin && covered_span.end >= target_span.end
                       ? 1.0
                       : 0.0;
        }
        const auto begin = std::max(covered_span.begin, target_span.begin);
        const auto end = std::min(covered_span.end, target_span.end);
        if (end <= begin) return 0.0;
        return static_cast<double>((end - begin).count())
               / static_cast<double>(target_span.span().count());
    }

    /// 最大连续缺口（秒）。
    [[nodiscard]] double maxReferenceGap() const noexcept {
        return std::chrono::duration<double>(max_reference_gap).count();
    }
};

[[nodiscard]] AteResult absoluteTrajectoryError(const Trajectory& estimate,
                                                const Trajectory& reference,
                                                const AteOptions& options = {});

/// RPE 的间隔定义。按时间与按距离是两种口径，KITTI 用距离，TUM 用时间。
enum class RpeDelta : std::uint8_t { kTime, kDistance };

struct RpeOptions {
    fw::Duration tolerance = std::chrono::milliseconds(20);
    AlignmentKind alignment = AlignmentKind::kNone;  ///< RPE 本身不需要全局对齐
    AssociationMode association = AssociationMode::kNearest;
    fw::Duration max_interpolation_gap = std::chrono::milliseconds(150);
    RpeDelta delta_kind = RpeDelta::kTime;
    fw::Duration delta_time = std::chrono::seconds(1);
    double delta_distance_m = 100.0;
    /// 把误差除以间隔长度，得到"每米漂移"。KITTI 报表用这个口径。
    bool normalize = false;
};

struct RpeResult {
    Statistics translation;   ///< 米，或 normalize 时为 米/米
    Statistics rotation_deg;  ///< 度，或 normalize 时为 度/米
    std::size_t pairs = 0;
    RpeDelta delta_kind = RpeDelta::kTime;
    bool normalized = false;
};

[[nodiscard]] RpeResult relativePoseError(const Trajectory& estimate, const Trajectory& reference,
                                          const RpeOptions& options = {});

/// 把一次评测的全部数字摊平成"指标名 → 值"，供基线比对与报表使用。
[[nodiscard]] std::vector<std::pair<std::string, double>> flatten(const AteResult& ate,
                                                                  const RpeResult& rpe);

}  // namespace simpleslam::eval
