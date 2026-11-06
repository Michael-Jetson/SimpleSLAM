// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file evaluators.hpp
/// 内置的评测函数：`ate`、`rpe`。口径见 `metrics.hpp`：对齐方式是结果的一部分，平移与旋转
/// 分开统计，按距离取段时走过的路沿真值累积。入口的目录登记它们（`app/catalogs.hpp`）。

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
            v("alignment", alignment, "对齐方式：none / first_pose / se3 / sim3");
            v("association", association,
              "关联：nearest 最近邻 / interpolate 把真值插值到估计的时刻");
            v("tolerance_s", tolerance_s, "最近邻关联的容差（秒）");
            v("max_interpolation_gap_s", max_interpolation_gap_s, "插值关联的最大括点间隔（秒）");
        }
    };

    /// 写法不对（认不出的对齐方式、负的容差）时抛 `std::invalid_argument`：装配时就报。
    explicit AteEvaluator(const Config& config);

    [[nodiscard]] Needs needs() const override { return Needs{.groundtruth = true}; }
    [[nodiscard]] Evaluation evaluate(const EvaluationInputs& inputs) const override;

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
    [[nodiscard]] Evaluation evaluate(const EvaluationInputs& inputs) const override;

private:
    Config config_;
    RpeOptions options_;
};

}  // namespace simpleslam::eval
