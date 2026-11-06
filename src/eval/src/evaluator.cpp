// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/eval/evaluator.hpp>

#include <stdexcept>

namespace simpleslam::eval {

fw::Trajectory EvaluationInputs::estimate(std::string_view name) const {
    throw std::runtime_error("这份评测输入不单独给估计轨迹（要的是 " + std::string(name) + "）");
}

std::vector<MetricSample> EvaluationInputs::metrics(std::string_view key) const {
    throw std::runtime_error("这份评测输入没有逐帧度量（要的是 " + std::string(key) + "）");
}

std::vector<EventSample> EvaluationInputs::events(std::string_view key) const {
    throw std::runtime_error("这份评测输入没有事件（要的是 " + std::string(key) + "）");
}

}  // namespace simpleslam::eval
