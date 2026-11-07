// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file evaluators.hpp
/// 内置的评测函数：`ate`、`rpe`、`kitti`、`timing`。口径见 `metrics.hpp`：对齐方式是结果的
/// 一部分，平移与旋转分开统计，按距离取段时走过的路沿真值累积。入口的目录登记它们
/// （`app/catalogs.hpp`）。

#include <SimpleSLAM/eval/evaluator.hpp>
#include <SimpleSLAM/eval/metrics.hpp>

#include <string>
#include <string_view>

namespace simpleslam::eval {

/// 绝对轨迹误差：对齐之后逐对比较位姿。
class AteEvaluator final : public EvaluatorBase {
public:
    static constexpr std::string_view kTypeName = "ate";

    struct Config {
        std::string estimate = "odometry";
        std::string alignment = "se3";
        std::string association = "nearest";
        double tolerance_s = 0.02;
        double max_interpolation_gap_s = 0.15;

        template <class V>
        void visit(V& v) {
            v("estimate", estimate, "评哪条估计：odometry / online / keyframes");
            v("alignment", alignment,
              "对齐方式：none / first_pose / se3 / sim3 / posyaw（位置加航向，给 VIO）");
            v("association", association,
              "关联：nearest 最近邻 / interpolate 把真值插值到估计的时刻");
            v("tolerance_s", tolerance_s, "最近邻关联的容差（秒）");
            v("max_interpolation_gap_s", max_interpolation_gap_s, "插值关联的最大括点间隔（秒）");
        }
    };

    /// 写法不对（认不出的对齐方式、负的容差）时抛 `std::invalid_argument`：装配时就报。
    explicit AteEvaluator(const Config& config);

    [[nodiscard]] Needs needs() const override { return Needs{.groundtruth = true}; }
    [[nodiscard]] Evaluation evaluate(const IEvaluationInputs& inputs) const override;

private:
    Config config_;
    AteOptions options_;
};

/// 相对位姿误差：按距离（KITTI 的口径，归一化成每米）或按时间（TUM 的口径）取段。
class RpeEvaluator final : public EvaluatorBase {
public:
    static constexpr std::string_view kTypeName = "rpe";

    struct Config {
        std::string estimate = "odometry";
        double delta_m = 100.0;
        double delta_s = 0.0;
        std::string association = "nearest";
        double tolerance_s = 0.02;
        double max_interpolation_gap_s = 0.15;

        template <class V>
        void visit(V& v) {
            v("estimate", estimate, "评哪条估计：odometry / online / keyframes");
            v("delta_m", delta_m, "按距离取段的长度（米），误差按每米归一");
            v("delta_s", delta_s, "按时间取段的长度（秒）；大于 0 时以它为准");
            v("association", association,
              "关联：nearest 最近邻 / interpolate 把真值插值到估计的时刻");
            v("tolerance_s", tolerance_s, "最近邻关联的容差（秒）");
            v("max_interpolation_gap_s", max_interpolation_gap_s, "插值关联的最大括点间隔（秒）");
        }
    };

    /// 两种间隔都不大于 0、写法不对时抛 `std::invalid_argument`。
    explicit RpeEvaluator(const Config& config);

    [[nodiscard]] Needs needs() const override { return Needs{.groundtruth = true}; }
    [[nodiscard]] Evaluation evaluate(const IEvaluationInputs& inputs) const override;

private:
    Config config_;
    RpeOptions options_;
};

/// KITTI 里程计官方口径：段长 100–800 米，每隔 `step_frames` 帧一个起点，走过的路沿真值累积，
/// 不对齐。平移误差是百分比，旋转误差是每米的度数，都是全部段的平均。与官方的一处不同：
/// 官方要求两边帧数相同、按行号配对，这里按时刻关联，只在关联上的帧上取段。
class KittiEvaluator final : public EvaluatorBase {
public:
    static constexpr std::string_view kTypeName = "kitti";

    struct Config {
        std::string estimate = "odometry";
        std::int64_t step_frames = 10;
        std::string association = "nearest";
        double tolerance_s = 0.02;
        double max_interpolation_gap_s = 0.15;

        template <class V>
        void visit(V& v) {
            v("estimate", estimate, "评哪条估计：odometry / online / keyframes");
            v("step_frames", step_frames, "每隔多少个关联上的帧取一个起点（官方是 10）");
            v("association", association,
              "关联：nearest 最近邻 / interpolate 把真值插值到估计的时刻");
            v("tolerance_s", tolerance_s, "最近邻关联的容差（秒）");
            v("max_interpolation_gap_s", max_interpolation_gap_s, "插值关联的最大括点间隔（秒）");
        }
    };

    /// 起点间隔不是正数、写法不对时抛 `std::invalid_argument`。
    explicit KittiEvaluator(const Config& config);

    [[nodiscard]] Needs needs() const override { return Needs{.groundtruth = true}; }
    [[nodiscard]] Evaluation evaluate(const IEvaluationInputs& inputs) const override;

private:
    Config config_;
    KittiOptions options_;
};

/// 耗时：读一个度量（默认里程计每帧的 `track.duration`），给 mean、median、p95、max。
/// 统计量与 evo、rpg 同一套，p95 在相邻样本之间线性插值。
class TimingEvaluator final : public EvaluatorBase {
public:
    static constexpr std::string_view kTypeName = "timing";

    struct Config {
        std::string key = "track.duration";

        template <class V>
        void visit(V& v) {
            v("key", key, "读哪个度量：模块与插件用 log_.metric 记下的键");
        }
    };

    /// 键是空的时抛 `std::invalid_argument`。
    explicit TimingEvaluator(const Config& config);

    [[nodiscard]] Needs needs() const override { return Needs{}; }
    [[nodiscard]] Evaluation evaluate(const IEvaluationInputs& inputs) const override;

private:
    Config config_;
};

}  // namespace simpleslam::eval
