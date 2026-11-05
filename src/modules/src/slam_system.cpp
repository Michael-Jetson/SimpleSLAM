// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/modules/slam_system.hpp>

#include <format>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace simpleslam::modules {

std::vector<std::string> RunOutcome::reasons() const {
    std::vector<std::string> out;
    const auto add = [&out](bool bad, std::string what) {
        if (bad) out.push_back(std::move(what));
    };
    add(!inputComplete(), std::format("输入不完整：{}", fw::toString(input)));
    add(cancelled, "运行被取消：" + cancel_reason);
    add(!drained, "排空预算用尽，仍有未处理的工作");
    add(!shutdown_drained && !cancelled, "关闭时排空预算用尽，已接纳的工作没有做完");
    add(packets_without_outcome > 0, std::format("{} 个送达的包没有终态", packets_without_outcome));
    add(contract_errors > 0, std::format("{} 帧违反包契约、没进估计", contract_errors));
    add(failed_frames > 0, std::format("{} 帧算法失败、没有结果", failed_frames));
    add(failed_corrections > 0, std::format("{} 次校正准备失败、没有生效", failed_corrections));
    add(reinitializations > 0,
        std::format("{} 次按恢复协议重新初始化，轨迹与关键帧链在那里断开", reinitializations));
    add(isolated_failures > 0,
        std::format("{} 次回调异常，对应的工作没有做完", isolated_failures));
    add(foreign_clock_drops > 0,
        std::format("{} 条数据不在参考时钟域、被拒收", foreign_clock_drops));
    add(out_of_order_imu > 0, std::format("{} 条 IMU 时间倒流或重复", out_of_order_imu));
    add(dropped_triggers > 0,
        std::format("{} 个触发帧被丢掉（下游处理不过来，或等括点积压）", dropped_triggers));
    add(rejected_triggers > 0, std::format("{} 个触发帧时刻不前进、被拒收", rejected_triggers));
    add(imu_backlog_drops > 0, std::format("{} 条 IMU 因积压超限被丢掉", imu_backlog_drops));
    add(attachment_drops > 0,
        std::format("{} 条图像 / GNSS 因待附着队列超限被丢掉", attachment_drops));
    add(pending_triggers > 0, std::format("{} 个触发帧到最后都没发出", pending_triggers));
    add(trajectory_missing > 0, std::format("轨迹少了 {} 个已发布的结果", trajectory_missing));
    add(trajectory_skipped > 0,
        std::format("轨迹跳过了 {} 个时间不递增的结果", trajectory_skipped));
    add(graph_missing_keyframes > 0,
        std::format("位姿图少了 {} 个关键帧", graph_missing_keyframes));
    add(index_missing_keyframes > 0,
        std::format("位置识别少索引了 {} 个关键帧", index_missing_keyframes));
    add(unsolved_constraints > 0,
        std::format("{} 条回环 / 先验约束没有进入最后一次成功求解", unsolved_constraints));
    add(rejected_constraints > 0,
        std::format("{} 条回环内容不合法或端点不存在、没进图", rejected_constraints));
    add(calibration_mixed, "位姿图混入了不同标定的节点，未产出校正");
    return out;
}

std::vector<std::string> RunOutcome::notes() const {
    std::vector<std::string> out;
    const auto add = [&out](bool present, std::string what) {
        if (present) out.push_back(std::move(what));
    };
    add(propagation_gaps > 0,
        std::format("{} 次算法状态与下一包的传播起点接不上，已按缺支撑政策处理",
                    propagation_gaps));
    add(awaiting_support_frames > 0,
        std::format("{} 帧初始化前缺支撑，按算法政策等待、没有位姿", awaiting_support_frames));
    add(rejected_for_gap_frames > 0,
        std::format("{} 帧缺支撑，按算法政策拒绝、没有位姿", rejected_for_gap_frames));
    add(degraded_for_gap_frames > 0,
        std::format("{} 帧缺支撑，结果按降级处理", degraded_for_gap_frames));
    add(failed_solutions > 0 && unsolved_constraints == 0,
        std::format("{} 次求解失败，之后的成功求解已覆盖全部约束", failed_solutions));
    add(!finalized, "结局取自运行关闭之前，不含关闭时的排空");
    return out;
}

SlamSystem::SlamSystem(Options options)
    : options_(options), runtime_(comm::RuntimeOptions{.offline = options.offline}) {
    runtime_.setErrorReporter([this](std::string_view where, std::string_view what) {
        fw::DiagnosticRecord record;
        record.header.module = std::string(where);
        record.level = fw::DiagnosticClass::kObservation;
        record.payload = fw::EventRecord{"comm.error", std::string(what)};
        diagnostics_.submit(record);
    });
}

SensorIoModule& SlamSystem::setSource(std::unique_ptr<fw::ISensorSource> source,
                                      fw::PacketizerPolicy policy, const std::string& name) {
    descriptor_ = source->descriptor();
    if (descriptor_.declared && policy.imu == fw::ImuRequirement::kRequired
        && !descriptor_.has_imu) {
        throw comm::WiringError(
            std::format("数据来源 {} 没有惯性流，打包策略却要求 IMU", descriptor_.name));
    }
    sensor_io_ =
        &runtime_.addModule<SensorIoModule>(name, std::move(source), policy, resources_);
    return *sensor_io_;
}

void SlamSystem::init() {
    if (!descriptor_.required_extrinsics.empty()) {
        const auto missing = fw::missingExtrinsics(*resources_.calibrationSnapshot(),
                                                   descriptor_.required_extrinsics,
                                                   descriptor_.assumed_extrinsics_allowed);
        if (!missing.empty()) {
            std::string list;
            for (const auto& sensor : missing) list += (list.empty() ? "" : "、") + sensor;
            throw comm::WiringError(std::format(
                "缺少必需的外参：{}（数据集没有给出，也没有外部标定文件补上）", list));
        }
    }
    // 没有里程计就没有处理进度，也就谈不上在途多少
    if (!options_.offline && sensor_io_ != nullptr && odometry_ != nullptr) {
        sensor_io_->packetizer().setMaxInFlight(options_.online_max_in_flight);
    }
    if (options_.collect_trajectory && trajectory_ == nullptr) {
        trajectory_ = &runtime_.addModule<TrajectoryModule>("/slam/trajectory", resources_);
    }
    runtime_.init();
}

std::size_t SlamSystem::runOffline(std::size_t max_packets) {
    if (sensor_io_ == nullptr || finalized_) return 0;
    std::size_t processed = 0;
    std::size_t attempts = 0;
    while ((max_packets == 0 || attempts < max_packets) && !runtime_.stopRequested()) {
        const auto pumped = sensor_io_->pump(1);
        processed += pumped.published;
        attempts += pumped.published;
        advanceClockToData();
        settle();
        if (pumped.finished || pumped.stopped) break;
    }
    advanceClockToData();
    settle();
    return processed;
}

void SlamSystem::advanceClockToData() {
    auto* clock = runtime_.virtualClock();
    if (clock == nullptr || sensor_io_ == nullptr || finalized_) return;
    const auto& packetizer = sensor_io_->packetizer();
    if (!origin_set_) {
        const auto first = packetizer.firstObserved();
        if (!first) return;
        if (first->count() > 0 && clock->now().count() == 0) {
            runtime_.shiftTimers(comm::Duration{first->count()});
            clock->advanceTo(comm::Time{first->count()});
        }
        origin_set_ = true;
    }
    const auto data_time = packetizer.observedUntil();
    if (data_time.count() > 0) clock->advanceTo(comm::Time{data_time.count()});
}

std::size_t SlamSystem::settle(std::size_t max_rounds) {
    if (finalized_) return 0;
    std::size_t total = 0;
    bool quiescent = false;
    for (std::size_t i = 0; i < max_rounds; ++i) {
        const std::size_t done = runtime_.spinOnce(options_.spin_budget);
        total += done;
        if (done == 0) {
            quiescent = true;
            break;
        }
    }
    if (!quiescent) all_settled_ = false;
    return total;
}

const RunResult& SlamSystem::result() const {
    if (!finalized_) throw std::logic_error("运行还没有终结：先 shutdown() 或 cancel()");
    return final_result_;
}

void SlamSystem::finalize(comm::ShutdownMode mode, std::string reason) {
    if (comm::inRuntimeThread()) {
        throw std::logic_error("回调内不能同步终结运行；请 requestStop() 后由控制线程终结");
    }
    if (finalized_) return;
    runtime_.shutdown(mode);
    if (runtime_.state() != comm::RuntimeState::kDone) {
        throw std::logic_error("运行时尚未完成收尾，不能生成最终结果");
    }
    resources_.freeze();
    final_result_.outcome = collectOutcome();
    auto& outcome = final_result_.outcome;
    outcome.shutdown_drained = runtime_.shutdownDrained();
    outcome.cancelled = runtime_.cancelled();
    outcome.cancel_reason = std::move(reason);
    outcome.finalized = true;
    if (trajectory_ != nullptr) {
        final_result_.odometry = trajectory_->odometry();
        final_result_.online = trajectory_->online();
        final_result_.keyframes = trajectory_->keyframes();
    }
    finalized_ = true;
}

RunOutcome SlamSystem::collectOutcome() const {
    RunOutcome outcome;
    outcome.drained = all_settled_;
    outcome.isolated_failures = runtime_.isolatedFailures();
    const auto keyframes = resources_.keyframes.snapshot()->size();
    if (sensor_io_ != nullptr) {
        outcome.input = sensor_io_->outcome();
        const auto& packetizer = sensor_io_->packetizer();
        outcome.foreign_clock_drops = packetizer.foreignClockDrops();
        outcome.out_of_order_imu = packetizer.outOfOrderImu();
        outcome.dropped_triggers = packetizer.droppedTriggers();
        outcome.rejected_triggers = packetizer.rejectedTriggers();
        outcome.imu_backlog_drops = packetizer.imuBacklogDrops();
        outcome.attachment_drops = packetizer.attachmentDrops();
        outcome.pending_triggers = packetizer.pendingTriggers();
    }
    if (odometry_ != nullptr) {
        outcome.contract_errors = odometry_->contractErrors();
        outcome.failed_frames = odometry_->failedFrames();
        outcome.failed_corrections = odometry_->failedCorrections();
        outcome.reinitializations = odometry_->reinitializations();
        outcome.propagation_gaps = odometry_->propagationGaps();
        outcome.awaiting_support_frames = odometry_->awaitingSupportFrames();
        outcome.rejected_for_gap_frames = odometry_->rejectedForGapFrames();
        outcome.degraded_for_gap_frames = odometry_->degradedForGapFrames();
        if (sensor_io_ != nullptr) {
            // 送到里程计的每一包都要有终态：少了就是"送达了却没有下落"
            const auto delivered = sensor_io_->publishedPackets();
            const auto received = odometry_->receivedPackets();
            outcome.packets_without_outcome = delivered > received ? delivered - received : 0;
        }
        if (trajectory_ != nullptr) {
            const auto published = odometry_->publishedResults();
            const auto received = trajectory_->received();
            outcome.trajectory_missing = published > received ? published - received : 0;
        }
    }
    if (trajectory_ != nullptr) outcome.trajectory_skipped = trajectory_->skipped();
    if (place_recognizer_ != nullptr) {
        const auto indexed = place_recognizer_->indexedFrames();
        outcome.index_missing_keyframes = keyframes > indexed ? keyframes - indexed : 0;
    }
    const auto graph = resources_.graph.snapshot();
    if (graph_optimizer_ != nullptr) {
        outcome.failed_solutions = graph_optimizer_->failedSolutions();
        outcome.rejected_constraints = graph_optimizer_->rejectedConstraints();
        outcome.unsolved_constraints = graph_optimizer_->unsolvedRequiredConstraints();
        const auto nodes = graph->nodeCount();
        outcome.graph_missing_keyframes = keyframes > nodes ? keyframes - nodes : 0;
    }
    outcome.calibration_mixed = graph->calibration_mixed;
    return outcome;
}

}  // namespace simpleslam::modules
