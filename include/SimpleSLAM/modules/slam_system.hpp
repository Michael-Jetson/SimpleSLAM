// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file slam_system.hpp
/// 装配根：把资源、模块与算法接成一套系统。
///
/// 写算法的人只需要三步：给出数据来源、指定算法类型、跑起来。
/// 资源的写者归属、话题接线、校正回流、诊断汇聚都在这里一次性接好。

#include <SimpleSLAM/comm/runtime.hpp>
#include <SimpleSLAM/modules/graph_optimizer_module.hpp>
#include <SimpleSLAM/modules/odometry_module.hpp>
#include <SimpleSLAM/modules/place_recognizer_module.hpp>
#include <SimpleSLAM/modules/sensor_io_module.hpp>
#include <SimpleSLAM/modules/trajectory_module.hpp>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace simpleslam::modules {

/// 一次运行的结局：输入完整、处理完成、输出完整是三件事（不变量 19）。
///
/// "来源读完了"只说明输入这一侧到了头。处理完成要求**每个已接纳的输入都有终态证据**：
/// 送达里程计的包都落在"跟踪了 / 按政策等待 / 按政策拒绝 / 违反契约 / 失败"之一，
/// 失败与回调异常都不为零时不算完成。输出完整要求必需产物按账对上：轨迹收到了
/// 发布的每个结果，图与位置识别追上了每个关键帧，每条必需约束都进了最后一次成功
/// 求解。严格评测要的是三者同时成立；拿不完整的运行出数字是明确选项，
/// 而且要带着这里的原因一起报告。
///
/// 按算法声明的政策等待或拒绝的缺支撑帧有终态、有说明，不算处理失败——它们写在
/// notes() 里，缺了多少位姿由评测的目标窗口覆盖率如实反映。
struct RunOutcome {
    // ── 输入 ──
    fw::SourceOutcome input = fw::SourceOutcome::kRunning;

    // ── 处理 ──
    bool drained = true;                       ///< 每次步进都跑到了静止
    bool shutdown_drained = true;              ///< 关闭时把已接纳的工作排空了
    std::uint64_t packets_without_outcome = 0; ///< 送达里程计、却没有终态的包
    std::uint64_t contract_errors = 0;         ///< 违反包契约、没进估计的帧
    std::uint64_t failed_frames = 0;           ///< 算法失败（含状态不能再用）、没有结果的帧
    std::uint64_t failed_corrections = 0;      ///< 准备失败、没有生效的校正
    std::uint64_t reinitializations = 0;       ///< 按恢复协议重新初始化、轨迹断开的次数
    std::uint64_t isolated_failures = 0;       ///< 回调异常：执行边界接住的丢失工作
    std::uint64_t foreign_clock_drops = 0;     ///< 不在参考时钟域、被拒收的数据
    std::uint64_t out_of_order_imu = 0;        ///< 时间倒流或重复、被丢弃的 IMU
    std::uint64_t dropped_triggers = 0;        ///< 下游处理不过来或等括点积压、被丢掉的触发帧
    std::uint64_t rejected_triggers = 0;       ///< 时刻不前进、被拒收的触发帧
    std::uint64_t imu_backlog_drops = 0;       ///< IMU 积压超限被丢掉的样本
    std::uint64_t attachment_drops = 0;        ///< 待附着队列超限被丢掉的图像 / GNSS
    std::size_t pending_triggers = 0;          ///< 到最后都没发出的触发帧

    // ── 输出 ──
    std::uint64_t trajectory_missing = 0;      ///< 发布了、轨迹却没收到的结果
    std::uint64_t trajectory_skipped = 0;      ///< 时间不递增、被轨迹跳过的结果
    std::size_t graph_missing_keyframes = 0;   ///< 图没追上的关键帧
    std::size_t index_missing_keyframes = 0;   ///< 位置识别没索引到的关键帧
    std::size_t unsolved_constraints = 0;      ///< 没进最后一次成功求解的必需约束
    std::uint64_t rejected_constraints = 0;    ///< 内容不合法或端点不存在、没进图的回环
    bool calibration_mixed = false;            ///< 图里混入了不同标定的节点

    // ── 按政策处理、有说明的 ──
    std::uint64_t propagation_gaps = 0;        ///< 算法状态与下一包的传播起点接不上的次数
    std::uint64_t awaiting_support_frames = 0; ///< 初始化前缺支撑、按政策等待的帧
    std::uint64_t rejected_for_gap_frames = 0; ///< 缺支撑、按政策拒绝的帧
    std::uint64_t degraded_for_gap_frames = 0; ///< 缺支撑、结果被降级的帧
    std::uint64_t failed_solutions = 0;        ///< 历史求解失败次数（之后成功过就已恢复）

    /// 关闭之后固定下来的最终结局。false 表示运行还没终结，这只是当前的快照。
    bool finalized = false;
    /// 运行以取消结束：未决工作被明确放弃，不是完成。
    bool cancelled = false;
    std::string cancel_reason;

    [[nodiscard]] bool inputComplete() const noexcept {
        return input == fw::SourceOutcome::kCleanEof;
    }
    [[nodiscard]] bool processingComplete() const noexcept {
        return !cancelled && drained && shutdown_drained && packets_without_outcome == 0 && contract_errors == 0
               && failed_frames == 0 && failed_corrections == 0 && reinitializations == 0
               && isolated_failures == 0 && foreign_clock_drops == 0 && out_of_order_imu == 0
               && dropped_triggers == 0 && rejected_triggers == 0 && imu_backlog_drops == 0
               && attachment_drops == 0 && pending_triggers == 0;
    }
    [[nodiscard]] bool outputComplete() const noexcept {
        return trajectory_missing == 0 && trajectory_skipped == 0
               && graph_missing_keyframes == 0 && index_missing_keyframes == 0
               && unsolved_constraints == 0 && rejected_constraints == 0 && !calibration_mixed;
    }
    [[nodiscard]] bool complete() const noexcept {
        return inputComplete() && processingComplete() && outputComplete();
    }

    /// 不完整的原因，按人话列出来。完整时为空。
    [[nodiscard]] std::vector<std::string> reasons() const {
        std::vector<std::string> out;
        const auto add = [&out](bool bad, std::string what) {
            if (bad) out.push_back(std::move(what));
        };
        const auto n = [](auto count) { return std::to_string(count); };
        add(!inputComplete(), std::string("输入不完整：") + std::string(fw::toString(input)));
        add(cancelled, "运行被取消：" + cancel_reason);
        add(!drained, "排空预算用尽，仍有未处理的工作");
        add(!shutdown_drained && !cancelled, "关闭时排空预算用尽，已接纳的工作没有做完");
        add(packets_without_outcome > 0, n(packets_without_outcome) + " 个送达的包没有终态");
        add(contract_errors > 0, n(contract_errors) + " 帧违反包契约、没进估计");
        add(failed_frames > 0, n(failed_frames) + " 帧算法失败、没有结果");
        add(failed_corrections > 0, n(failed_corrections) + " 次校正准备失败、没有生效");
        add(reinitializations > 0,
            n(reinitializations) + " 次按恢复协议重新初始化，轨迹与关键帧链在那里断开");
        add(isolated_failures > 0, n(isolated_failures) + " 次回调异常，对应的工作没有做完");
        add(foreign_clock_drops > 0, n(foreign_clock_drops) + " 条数据不在参考时钟域、被拒收");
        add(out_of_order_imu > 0, n(out_of_order_imu) + " 条 IMU 时间倒流或重复");
        add(dropped_triggers > 0, n(dropped_triggers) + " 个触发帧被丢掉（下游处理不过来，或等括点积压）");
        add(rejected_triggers > 0, n(rejected_triggers) + " 个触发帧时刻不前进、被拒收");
        add(imu_backlog_drops > 0, n(imu_backlog_drops) + " 条 IMU 因积压超限被丢掉");
        add(attachment_drops > 0, n(attachment_drops) + " 条图像 / GNSS 因待附着队列超限被丢掉");
        add(pending_triggers > 0, n(pending_triggers) + " 个触发帧到最后都没发出");
        add(trajectory_missing > 0, "轨迹少了 " + n(trajectory_missing) + " 个已发布的结果");
        add(trajectory_skipped > 0, "轨迹跳过了 " + n(trajectory_skipped) + " 个时间不递增的结果");
        add(graph_missing_keyframes > 0, "位姿图少了 " + n(graph_missing_keyframes) + " 个关键帧");
        add(index_missing_keyframes > 0,
            "位置识别少索引了 " + n(index_missing_keyframes) + " 个关键帧");
        add(unsolved_constraints > 0,
            n(unsolved_constraints) + " 条回环 / 先验约束没有进入最后一次成功求解");
        add(rejected_constraints > 0, n(rejected_constraints) + " 条回环内容不合法或端点不存在、没进图");
        add(calibration_mixed, "位姿图混入了不同标定的节点，未产出校正");
        return out;
    }

    /// 有终态、有说明、不算不完整的事实。与 reasons() 一起进报告。
    [[nodiscard]] std::vector<std::string> notes() const {
        std::vector<std::string> out;
        const auto add = [&out](bool present, std::string what) {
            if (present) out.push_back(std::move(what));
        };
        const auto n = [](auto count) { return std::to_string(count); };
        add(propagation_gaps > 0, n(propagation_gaps)
                                      + " 次算法状态与下一包的传播起点接不上，已按缺支撑政策处理");
        add(awaiting_support_frames > 0,
            n(awaiting_support_frames) + " 帧初始化前缺支撑，按算法政策等待、没有位姿");
        add(rejected_for_gap_frames > 0,
            n(rejected_for_gap_frames) + " 帧缺支撑，按算法政策拒绝、没有位姿");
        add(degraded_for_gap_frames > 0,
            n(degraded_for_gap_frames) + " 帧缺支撑，结果按降级处理");
        add(failed_solutions > 0 && unsolved_constraints == 0,
            n(failed_solutions) + " 次求解失败，之后的成功求解已覆盖全部约束");
        add(!finalized, "结局取自运行关闭之前，不含关闭时的排空");
        return out;
    }
};

/// 一次运行的最终结果视图：终结时固定下来的结局与产物副本。
///
/// 终结之后对外读它，而不是读已经关闭的模块——模块的可变方法不再有意义，
/// 这份视图才是"这次运行的结果"。
struct RunResult {
    RunOutcome outcome;
    eval::Trajectory odometry;   ///< 里程计轨迹（odom 系）
    eval::Trajectory online;     ///< 在线轨迹（world 系，当时实际输出过的值）
    eval::Trajectory keyframes;  ///< 优化后的关键帧轨迹（world 系）
};

/// 装配选项。放在命名空间作用域而不是嵌套在类里：嵌套类的默认成员初始化器
/// 要到外层类结束才完整，无法用于外层类自己的默认实参。
struct SlamSystemOptions {
    /// 离线：单线程步进、虚拟时钟、可重复。在线：一个模块一个线程 + 单调时钟。
    bool offline = true;
    /// 每轮步进的任务预算，防止持续到达的消息让一次步进不返回。
    std::size_t spin_budget = 4096;
    /// 在线时入口允许多少包在途。队列只为突发准备，冗余给足：200 包是 10 Hz 激光的 20 秒。
    /// 超过它就丢触发帧，IMU 并入下一包，并记进运行结局。离线不限，不丢。
    std::size_t online_max_in_flight = 200;
    /// 是否收集轨迹。评测、导出、回归都要它，代价是每帧一个位姿。
    bool collect_trajectory = true;
};

/// 装配、离线步进、结果查询与终结由控制线程串行调用；在线模块通过 requestStop()
/// 请求停止，控制线程待 spin() 返回后完成 shutdown()/cancel()。
class SlamSystem {
public:
    using Options = SlamSystemOptions;

    explicit SlamSystem(Options options = Options{})
        : options_(options),
          runtime_(comm::RuntimeOptions{.offline = options.offline}) {
        runtime_.setErrorReporter([this](std::string_view where, std::string_view what) {
            fw::DiagnosticRecord record;
            record.header.module = std::string(where);
            record.level = fw::DiagnosticClass::kObservation;
            record.payload = fw::EventRecord{"comm.error", std::string(what)};
            diagnostics_.submit(record);
        });
    }

    [[nodiscard]] SlamResources& resources() noexcept { return resources_; }

    /// 写入标定，返回本次提交生成的版本号。
    /// 数据集自带标定时直接传 `source.calibration()`，不必手抄一遍。运行终结之后拒绝。
    std::uint64_t setCalibration(fw::CalibrationData calibration) {
        return resources_.setCalibration(std::move(calibration));
    }
    [[nodiscard]] comm::Runtime& runtime() noexcept { return runtime_; }
    [[nodiscard]] fw::MemoryDiagnosticSink& diagnostics() noexcept { return diagnostics_; }

    /// 接上数据来源。来源声明过能力时，这里就核对它与打包策略是否相容——
    /// 例如策略要求 IMU 而来源没有惯性流，装配期就失败，而不是跑起来之后每包都缺支撑。
    SensorIoModule& setSource(std::unique_ptr<fw::ISensorSource> source,
                              fw::PacketizerPolicy policy = {},
                              const std::string& name = "/slam/sensor_io") {
        descriptor_ = source->descriptor();
        if (descriptor_.declared && policy.imu == fw::ImuRequirement::kRequired
            && !descriptor_.has_imu) {
            throw comm::WiringError("数据来源 " + descriptor_.name
                                    + " 没有惯性流，打包策略却要求 IMU");
        }
        sensor_io_ = &runtime_.addModule<SensorIoModule>(name, std::move(source), policy,
                                                         resources_);
        return *sensor_io_;
    }

    /// 数据来源声明的能力。进评测报告与实验产物。
    [[nodiscard]] const fw::SourceDescriptor& sourceDescriptor() const noexcept {
        return descriptor_;
    }

    /// 指定里程计算法。Args 转发给算法的构造函数。
    template <class Algorithm, class... Args>
    OdometryModule& setOdometry(Args&&... args) {
        odometry_ = &runtime_.addModule<OdometryModule>(
            "/slam/odometry", std::make_unique<Algorithm>(std::forward<Args>(args)...), resources_,
            &diagnostics_);
        return *odometry_;
    }

    template <class Algorithm, class... Args>
    PlaceRecognizerModule& setPlaceRecognizer(Args&&... args) {
        place_recognizer_ = &runtime_.addModule<PlaceRecognizerModule>(
            "/slam/place_recognizer", std::make_unique<Algorithm>(std::forward<Args>(args)...),
            resources_, &diagnostics_);
        return *place_recognizer_;
    }

    template <class Algorithm, class... Args>
    GraphOptimizerModule& setGraphOptimizer(Args&&... args) {
        graph_optimizer_ = &runtime_.addModule<GraphOptimizerModule>(
            "/slam/graph_optimizer", std::make_unique<Algorithm>(std::forward<Args>(args)...),
            resources_, &diagnostics_);
        return *graph_optimizer_;
    }

    /// 装配完成。来源要求的外参此刻必须已知——"来源有哪些标定"与"最终装配齐没齐"
    /// 是两件事：数据集可以缺外参，只要外部标定文件补上了就行。
    void init() {
        if (!descriptor_.required_extrinsics.empty()) {
            const auto missing =
                fw::missingExtrinsics(*resources_.calibrationSnapshot(),
                                      descriptor_.required_extrinsics,
                                      descriptor_.assumed_extrinsics_allowed);
            if (!missing.empty()) {
                std::string list;
                for (const auto& sensor : missing) list += (list.empty() ? "" : "、") + sensor;
                throw comm::WiringError("缺少必需的外参：" + list
                                        + "（数据集没有给出，也没有外部标定文件补上）");
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

    /// 离线跑完整个数据源：喂一包、把虚拟时钟推到该包的数据时刻、跑到静止，
    /// 再喂下一包。因此结果可逐位复现，且**依赖定时器的业务也会被覆盖**：
    /// 只喂包不推时钟的话，虚拟时钟一直停在 0，定时器永远不触发。
    ///
    /// 时钟由数据时间驱动是离线的正确语义：回放的"现在"就是数据的"现在"。
    /// 离线调度固定为：**每一步都推进时钟并跑到静止**，不论这一步有没有凑成包。
    /// 只在成包时才跑的话，读了样本却没凑成包（让出）的那些步里，定时器与其他模块
    /// 一次都得不到运行。
    ///
    /// 时间原点：时钟从 0 起，第一次拿到数据时整体对到第一条数据的时刻——装配期排下的
    /// 定时器随之改从那里起算，而不是在长时间戳（Unix 纳秒）的第一步集体"补"触发一次。
    /// 之后时钟跟的是**已读样本的数据时间**，不是成包时刻：一段只有 IMU、迟迟等不到
    /// 触发帧的数据，时间照样在走。
    /// EOF 政策：把时钟推到最后一条数据的时刻再跑到静止；晚于最后数据时刻的定时器
    /// 不再触发——离线的"现在"到数据结束为止，不向后外推。
    ///
    /// max_packets 限制的是发出的包数（按尝试计）：一条 IMU 同时让好几帧凑齐括点时，
    /// 也只发预算允许的那几个。运行终结之后什么也不做。
    std::size_t runOffline(std::size_t max_packets = 0) {
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

    /// 这次运行的结局。评测报告要在它之后写：报告前没有它，就说不清结论建立在
    /// 完整还是残缺的运行之上。
    ///
    /// 运行终结（shutdown）之后返回固定下来的最终结局，含关闭时的排空；之前返回的是
    /// 当前快照，finalized 为 false。
    [[nodiscard]] RunOutcome runOutcome() const {
        return finalized_ ? final_result_.outcome : collectOutcome();
    }

    /// 运行是否已经终结。终结之后系统是只读的。
    [[nodiscard]] bool finalized() const noexcept { return finalized_; }

    /// 把虚拟时钟推进到已读样本的数据时刻。在线模式（单调时钟）下是空操作。
    void advanceClockToData() {
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

    /// 处理到系统静止（没有就绪任务为止），带轮数上限兜底。
    ///
    /// 轮数用完仍有工作时，这次运行记为"没排空"（RunOutcome::drained）：
    /// 兜底的上限不能悄悄吞掉没处理完的工作。
    std::size_t settle(std::size_t max_rounds = 256) {
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

    void start() { runtime_.start(); }
    void spin() { runtime_.spin(); }
    void requestStop() { runtime_.requestStop(); }

    /// 终结：停输入 → 等静止 → 完成已接纳工作（排空、各模块核对补读游标）→ 收尾 →
    /// 固定最终结果。
    ///
    /// 之后系统是只读的：result() 与 runOutcome() 返回固定下来的那一份；步进、改标定、
    /// 改参数一律拒绝。模块在析构时才销毁，析构时不再重复收尾。
    void shutdown() { finalize(comm::ShutdownMode::kFinish, {}); }

    /// 取消：停止新工作，不再排空；未决操作随之明确结案，结果标为取消并写明原因。
    void cancel(std::string reason) { finalize(comm::ShutdownMode::kCancel, std::move(reason)); }

    /// 终结之后固定下来的最终结果。运行还没终结时抛 std::logic_error：
    /// 那时读到的只能是半截的产物。
    [[nodiscard]] const RunResult& result() const {
        if (!finalized_) throw std::logic_error("运行还没有终结：先 shutdown() 或 cancel()");
        return final_result_;
    }

    [[nodiscard]] SensorIoModule* sensorIo() noexcept { return sensor_io_; }
    [[nodiscard]] OdometryModule* odometry() noexcept { return odometry_; }
    [[nodiscard]] PlaceRecognizerModule* placeRecognizer() noexcept { return place_recognizer_; }
    [[nodiscard]] GraphOptimizerModule* graphOptimizer() noexcept { return graph_optimizer_; }
    [[nodiscard]] TrajectoryModule* trajectory() noexcept { return trajectory_; }

private:
    void finalize(comm::ShutdownMode mode, std::string reason) {
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

    /// 按账汇总各模块的终态证据与产物进度。模块在 Runtime 析构之前一直存活，
    /// 关闭之后读它们同样安全。
    [[nodiscard]] RunOutcome collectOutcome() const {
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

    Options options_;
    SlamResources resources_;
    fw::MemoryDiagnosticSink diagnostics_;
    comm::Runtime runtime_;

    fw::SourceDescriptor descriptor_;
    bool all_settled_ = true;
    bool finalized_ = false;
    bool origin_set_ = false;
    RunResult final_result_;
    SensorIoModule* sensor_io_ = nullptr;
    OdometryModule* odometry_ = nullptr;
    PlaceRecognizerModule* place_recognizer_ = nullptr;
    GraphOptimizerModule* graph_optimizer_ = nullptr;
    TrajectoryModule* trajectory_ = nullptr;
};

}  // namespace simpleslam::modules
