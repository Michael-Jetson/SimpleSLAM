// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/eval/evaluators.hpp>

#include <cmath>
#include <format>
#include <stdexcept>
#include <utility>

namespace simpleslam::eval {

namespace {

/// 秒到纳秒：四舍五入；非有限、负数与装不下的量拒绝。
fw::Duration seconds(double value, const char* name) {
    if (!std::isfinite(value) || value < 0.0 || value > 9e9) {
        throw std::invalid_argument(
            std::format("{} 要是不小于 0 的有限秒数，得到 {}", name, value));
    }
    return fw::Duration{std::llround(value * 1e9)};
}

AssociationMode association(const std::string& text) {
    if (text == "nearest") return AssociationMode::kNearest;
    if (text == "interpolate") return AssociationMode::kInterpolate;
    throw std::invalid_argument("association 只收 nearest / interpolate，得到 " + text);
}

void requireEstimate(const std::string& name) {
    if (name != "odometry" && name != "online" && name != "keyframes") {
        throw std::invalid_argument("estimate 只收 odometry / online / keyframes，得到 " + name);
    }
}

/// 一族误差的统计量：rmse 进汇总表，其余只进结果。
void addStatistics(Evaluation& out, const std::string& quantity, const Statistics& stats,
                   const std::string& unit, const std::string& label) {
    out.add(quantity + ".rmse", stats.rmse, unit, label);
    out.add(quantity + ".mean", stats.mean, unit);
    out.add(quantity + ".median", stats.median, unit);
    out.add(quantity + ".std", stats.stddev, unit);
    out.add(quantity + ".min", stats.min, unit);
    out.add(quantity + ".max", stats.max, unit);
}

Table errorTable(const std::string& name, const std::vector<ErrorSample>& errors,
                 const std::string& translation_unit, const std::string& rotation_unit) {
    Table table{.name = name,
                .columns = {{.name = "time_ns", .unit = "ns"},
                            {.name = "translation", .unit = translation_unit},
                            {.name = "rotation", .unit = rotation_unit}},
                .rows = {}};
    table.rows.reserve(errors.size());
    for (const auto& error : errors) {
        table.rows.push_back({Cell{static_cast<std::int64_t>(error.time.count())},
                              Cell{error.translation}, Cell{error.rotation_deg}});
    }
    return table;
}

}  // namespace

AteEvaluator::AteEvaluator(const Config& config) : config_(config) {
    requireEstimate(config_.estimate);
    const auto alignment = parseAlignmentKind(config_.alignment);
    if (!alignment) {
        throw std::invalid_argument("alignment 只收 none / first_pose / se3 / sim3，得到 "
                                    + config_.alignment);
    }
    options_.alignment = *alignment;
    options_.association = association(config_.association);
    options_.tolerance = seconds(config_.tolerance_s, "tolerance_s");
    options_.max_interpolation_gap =
        seconds(config_.max_interpolation_gap_s, "max_interpolation_gap_s");
}

Evaluation AteEvaluator::evaluate(const EvaluationInputs& inputs) const {
    ComparablePair pair = inputs.pair(config_.estimate);
    AteOptions options = options_;
    if (const auto window = inputs.targetWindow()) options.target_window = *window;
    const AteResult ate = absoluteTrajectoryError(pair.estimate, pair.reference, options);

    Evaluation out;
    out.notes = std::move(pair.notes);
    out.samples = ate.pairs;
    out.notes.push_back(std::format("关联（{}）：{} 对，估计 {} 个", toString(options.association),
                                    ate.pairs, ate.estimate_samples));
    if (ate.alignment.kind != options.alignment && ate.pairs > 0) {
        out.notes.push_back(std::format("请求的对齐方式是 {}，实际用的是 {}",
                                        toString(options.alignment), toString(ate.alignment.kind)));
    }
    if (ate.pairs == 0) {
        out.notes.push_back("没有关联上的样本，ATE 无结果");
        return out;
    }
    addStatistics(out, "translation", ate.translation, "m", "ATE_t");
    addStatistics(out, "rotation", ate.rotation_deg, "deg", "ATE_r");
    out.add("scale", ate.alignment.scale, "");
    out.add("coverage", ate.coverage(), "");
    out.add("reference_coverage", ate.referenceCoverage(), "");
    out.add("matched_reference_samples", static_cast<double>(ate.matched_reference_samples), "");
    out.add("span_ratio", ate.spanRatio(), "");
    out.add("max_reference_gap", ate.maxReferenceGap(), "s");
    if (ate.target_declared) out.add("target_coverage", ate.targetCoverage(), "");
    out.tables.push_back(errorTable("errors", ate.errors, "m", "deg"));
    return out;
}

RpeEvaluator::RpeEvaluator(const Config& config) : config_(config) {
    requireEstimate(config_.estimate);
    if (!std::isfinite(config_.delta_m) || config_.delta_m < 0.0) {
        throw std::invalid_argument(std::format("delta_m 要是不小于 0 的有限数，得到 {}",
                                                config_.delta_m));
    }
    const auto delta_time = seconds(config_.delta_s, "delta_s");
    if (config_.delta_s > 0.0) {
        options_.delta_kind = RpeDelta::kTime;
        options_.delta_time = delta_time;
        options_.normalize = false;
    } else if (config_.delta_m > 0.0) {
        options_.delta_kind = RpeDelta::kDistance;
        options_.delta_distance_m = config_.delta_m;
        options_.normalize = true;
    } else {
        throw std::invalid_argument("delta_m 与 delta_s 至少一个要大于 0");
    }
    options_.association = association(config_.association);
    options_.tolerance = seconds(config_.tolerance_s, "tolerance_s");
    options_.max_interpolation_gap =
        seconds(config_.max_interpolation_gap_s, "max_interpolation_gap_s");
}

Evaluation RpeEvaluator::evaluate(const EvaluationInputs& inputs) const {
    ComparablePair pair = inputs.pair(config_.estimate);
    const RpeResult rpe = relativePoseError(pair.estimate, pair.reference, options_);

    Evaluation out;
    out.notes = std::move(pair.notes);
    out.samples = rpe.pairs;
    const bool by_time = rpe.delta_kind == RpeDelta::kTime;
    out.notes.push_back(by_time ? std::format("按时间取段，每段 {} 秒", config_.delta_s)
                                : std::format("按距离取段，每段 {} 米，误差按每米归一",
                                              config_.delta_m));
    if (rpe.pairs == 0) {
        // 零误差与"无结果"不能长得一样
        out.notes.push_back("轨迹长度不足一个间隔，没有可比的段，RPE 无结果");
        return out;
    }
    const std::string translation_unit = rpe.normalized ? "m/m" : "m";
    const std::string rotation_unit = rpe.normalized ? "deg/m" : "deg";
    addStatistics(out, "translation", rpe.translation, translation_unit, "RPE_t");
    addStatistics(out, "rotation", rpe.rotation_deg, rotation_unit, "RPE_r");
    out.tables.push_back(errorTable("segments", rpe.errors, translation_unit, rotation_unit));
    return out;
}

}  // namespace simpleslam::eval
