// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// 测试用的评测函数：真值有几个样本，并按同一个数加权。多序列的汇总按权重平均：
/// 执行器的测试与多序列子进程的入口共用它，子进程得登记同一个实现。

#include <SimpleSLAM/eval/evaluator.hpp>

#include <string_view>

namespace simpleslam::test {

class CountEvaluator final : public eval::EvaluatorBase {
public:
    static constexpr std::string_view kTypeName = "count";

    struct Config {
        template <class V>
        void visit(V& /*v*/) {}
    };

    explicit CountEvaluator(const Config& /*config*/) {}

    [[nodiscard]] eval::Needs needs() const override { return {.groundtruth = true}; }
    [[nodiscard]] eval::Evaluation evaluate(const eval::EvaluationInputs& inputs) const override {
        const auto pair = inputs.pair("odometry");
        const auto frames = static_cast<double>(pair.reference.size());
        eval::Evaluation out;
        out.samples = pair.reference.size();
        out.add("frames", frames, "", "N", frames);
        return out;
    }
};

}  // namespace simpleslam::test
