// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include "online_evaluation.hpp"

#include "evaluation.hpp"

#include <exception>
#include <stdexcept>
#include <utility>

namespace simpleslam::app {

namespace {

/// 运行中的快照作为评测函数的输入：轨迹是到目前为止攒下的，真值是运行之前备好的，
/// 度量与事件取自诊断汇聚点。目标窗口说的是整次运行，截至此刻不适用，所以没有。
class SnapshotInputs final : public eval::IEvaluationInputs {
public:
    SnapshotInputs(const modules::TrajectoryRecorder& recorder,
                   const OnlineEvaluationModule::Setup& setup)
        : recorder_(recorder), setup_(setup) {}

    [[nodiscard]] eval::ComparablePair pair(std::string_view estimate) const override {
        if (!setup_.reference) {
            throw std::runtime_error("数据没有真值，在线评测无从比较");
        }
        eval::ComparablePair pair{
            .estimate = this->estimate(estimate), .reference = *setup_.reference, .notes = {}};
        toReferenceSensor(pair, [this] { return setup_.calibration; });
        pair.notes.push_back("截至此刻的轨迹（在线评测）");
        return pair;
    }

    [[nodiscard]] std::optional<fw::TimeRange> targetWindow() const override {
        return std::nullopt;
    }

    [[nodiscard]] fw::Trajectory estimate(std::string_view name) const override {
        if (name == "odometry") return recorder_.odometry();
        if (name == "online") return recorder_.online();
        if (name == "keyframes") return modules::keyframeTrajectory(*setup_.resources);
        throw std::runtime_error("没有这条估计: " + std::string(name));
    }

    [[nodiscard]] std::vector<eval::MetricSample> metrics(std::string_view key) const override {
        std::vector<eval::MetricSample> out;
        for (const auto& record : setup_.diagnostics->metrics(key)) {
            const auto& metric = std::get<fw::MetricRecord>(record.payload);
            out.push_back(eval::MetricSample{.frame = frameOf(record.header),
                                             .data_time = timeOf(record.header),
                                             .source = record.header.source,
                                             .value = metric.value,
                                             .unit = metric.unit});
        }
        return out;
    }

    [[nodiscard]] std::vector<eval::EventSample> events(std::string_view key) const override {
        std::vector<eval::EventSample> out;
        for (const auto& record : setup_.diagnostics->events(key)) {
            const auto& event = std::get<fw::EventRecord>(record.payload);
            out.push_back(eval::EventSample{.frame = frameOf(record.header),
                                            .data_time = timeOf(record.header),
                                            .module = record.header.module,
                                            .source = record.header.source,
                                            .detail = event.detail});
        }
        return out;
    }

private:
    static std::optional<std::uint64_t> frameOf(const fw::DiagnosticHeader& header) {
        return header.has_frame ? std::optional(header.frame_sequence) : std::nullopt;
    }
    static std::optional<fw::Timestamp> timeOf(const fw::DiagnosticHeader& header) {
        return header.has_frame ? std::optional(header.data_time) : std::nullopt;
    }

    const modules::TrajectoryRecorder& recorder_;
    const OnlineEvaluationModule::Setup& setup_;
};

}  // namespace

OnlineEvaluationModule::OnlineEvaluationModule(Setup setup) : setup_(std::move(setup)) {
    if (setup_.resources == nullptr || setup_.diagnostics == nullptr) {
        throw std::invalid_argument("在线评测要系统的资源与诊断汇聚点");
    }
    if (setup_.interval <= comm::Duration::zero()) {
        throw std::invalid_argument("在线评测的间隔必须为正");
    }
}

void OnlineEvaluationModule::onInit() {
    subscribe(modules::topics::kOdometry, 0, &OnlineEvaluationModule::onOdometry);
    createTimer(setup_.interval, &OnlineEvaluationModule::evaluateNow);
}

void OnlineEvaluationModule::onOdometry(const fw::OdometryResult& result) {
    recorder_.add(result);
    last_packet_ = result.packet_sequence;
    last_time_ = result.time;
}

void OnlineEvaluationModule::evaluateNow() {
    if (recorder_.odometry().empty()) return;  // 还没有位姿，不评
    // 度量记在最近一条结果所在的那一包上，与里程计的度量同一个输入序号
    log_.setFrame({.sequence = last_packet_,
                   .data_time_ns = last_time_.count(),
                   .session = 0,
                   .parameter_revision = 0});
    const SnapshotInputs inputs(recorder_, setup_);
    for (const auto& entry : setup_.evaluators) {
        // 在线评测是额外的观察：一个评测函数出错只记一条警告，不影响这次运行
        try {
            const eval::Evaluation result = entry.evaluator->evaluate(inputs);
            eval::checkEvaluation(entry.name, result);
            for (const auto& scalar : result.scalars) {
                log_.metric(entry.name + "." + scalar.name, scalar.value, scalar.unit);
            }
            log_.info("在线{}", summaryLine(entry.name, entry.type, result));
        } catch (const std::exception& e) {
            log_.warn("在线评测 {} 出错: {}", entry.name, e.what());
        }
    }
}

}  // namespace simpleslam::app
