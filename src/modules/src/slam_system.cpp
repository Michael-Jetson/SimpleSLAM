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
    add(packets_without_outcome > 0, std::format("{} 个送达的包没有终态", packets_without_outcome));
    add(contract_errors > 0, std::format("{} 帧违反包契约、没进估计", contract_errors));
    add(output_contract_errors > 0,
        std::format("{} 帧跟踪了、产物却违反契约，结果作废", output_contract_errors));
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

namespace {

/// 日志对象记下的事件与度量，按诊断的格式交给汇聚点：事件是控制类，度量是指标类。
fw::DiagnosticRecord toDiagnostic(const logging::Event& event) {
    fw::DiagnosticRecord record;
    record.header.module = event.module;
    record.header.source = event.source;
    record.header.process_time = fw::Timestamp{event.time_ns};
    if (event.frame) {
        record.header.has_frame = true;
        record.header.frame_sequence = event.frame->sequence;
        record.header.data_time = fw::Timestamp{event.frame->data_time_ns};
        record.header.session = fw::SessionId{event.frame->session};
        record.header.parameter_revision = event.frame->parameter_revision;
    }
    if (event.kind == logging::Event::Kind::kMetric) {
        record.level = fw::DiagnosticClass::kMetric;
        record.payload = fw::MetricRecord{event.key, event.value, event.unit};
    } else {
        record.level = fw::DiagnosticClass::kControl;
        record.payload = fw::EventRecord{event.key, event.detail};
    }
    return record;
}

}  // namespace

SlamSystem::SlamSystem(Options options)
    : options_(options),
      log_router_(logging::Router::create(options.log)),
      router_closer_(log_router_),
      runtime_(comm::RuntimeOptions{.offline = options.offline, .log_router = log_router_}) {
    log_router_->setEventSink(
        [this](const logging::Event& event) { diagnostics_.submit(toDiagnostic(event)); });
    runtime_.setErrorReporter([this](std::string_view where, std::string_view what) {
        fw::DiagnosticRecord record;
        record.header.module = std::string(where);
        record.level = fw::DiagnosticClass::kObservation;
        record.payload = fw::EventRecord{"comm.error", std::string(what)};
        diagnostics_.submit(record);
        log_router_->root(where).error("{}", what);
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

OdometryModule& SlamSystem::setOdometry(std::unique_ptr<plugin::OdometryBase> algorithm) {
    odometry_ = &runtime_.addModule<OdometryModule>(std::string(kOdometryModule),
                                                    std::move(algorithm), resources_);
    return *odometry_;
}

PlaceRecognizerModule& SlamSystem::setPlaceRecognizer(
    std::unique_ptr<plugin::PlaceRecognizerBase> algorithm) {
    place_recognizer_ = &runtime_.addModule<PlaceRecognizerModule>(
        std::string(kPlaceRecognizerModule), std::move(algorithm), resources_);
    return *place_recognizer_;
}

GraphOptimizerModule& SlamSystem::setGraphOptimizer(
    std::unique_ptr<plugin::GraphOptimizerBase> algorithm, OdometryEdgePolicy edge_policy) {
    graph_optimizer_ = &runtime_.addModule<GraphOptimizerModule>(
        std::string(kGraphOptimizerModule), std::move(algorithm), resources_, edge_policy);
    return *graph_optimizer_;
}

std::vector<comm::ParamComponent> builtinModuleParams(bool with_backend) {
    std::vector<comm::ParamComponent> components;
    if (with_backend) {
        const auto& uses = GraphOptimizerModule::kParams;
        components.push_back(comm::ParamComponent{
            .component = "模块 " + std::string(kGraphOptimizerModule),
            .module = std::string(kGraphOptimizerModule),
            .uses = std::vector<comm::ParamUse>(uses.begin(), uses.end())});
    }
    return components;
}

void SlamSystem::init() {
    if (!runtime_.params().defined()) {
        auto resolved = comm::resolveParams(builtinModuleParams(graph_optimizer_ != nullptr), {});
        runtime_.params().define(std::move(resolved.definitions), std::move(resolved.values));
    }
    if (!descriptor_.required_extrinsics.empty()) {
        const auto missing = fw::missingExtrinsics(*resources_.calibrationSnapshot(),
                                                   descriptor_.required_extrinsics);
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
        trajectory_ = &runtime_.addModule<TrajectoryModule>(std::string(kTrajectoryModule),
                                                            resources_);
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

std::size_t SlamSystem::runOnline(comm::Duration pump_period, std::size_t packets_per_tick) {
    if (options_.offline) {
        throw std::logic_error("runOnline 要在线模式：SlamSystemOptions::offline = false");
    }
    if (sensor_io_ == nullptr || finalized_) return 0;
    // 读完即停止输入：来源是回放，读完就是运行结束；停止之后 spin() 等已接纳的工作走完
    sensor_io_->startPumping(pump_period, packets_per_tick, SensorIoModule::Pacing::kDataTime,
                             [this] { runtime_.requestStop(); });
    runtime_.spin();  // 起模块线程，阻塞到停止输入、已接纳的工作走完
    return sensor_io_->publishedPackets();
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

std::size_t SlamSystem::settle() {
    if (finalized_) return 0;
    return runtime_.spinUntilIdle();
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
        outcome.output_contract_errors = odometry_->outputContractErrors();
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
