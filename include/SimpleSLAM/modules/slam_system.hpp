// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file slam_system.hpp
/// 装配根：把资源、模块与算法接成一套系统。
///
/// 写算法的人只需要三步：给出数据来源、指定算法类型、跑起来。
/// 资源的写者归属、话题接线、校正回流、诊断汇聚都在这里一次性接好。

#include <SimpleSLAM/comm/runtime.hpp>
#include <SimpleSLAM/framework/diagnostics.hpp>
#include <SimpleSLAM/logging/construction.hpp>
#include <SimpleSLAM/logging/router.hpp>
#include <SimpleSLAM/modules/graph_optimizer_module.hpp>
#include <SimpleSLAM/modules/odometry_module.hpp>
#include <SimpleSLAM/modules/place_recognizer_module.hpp>
#include <SimpleSLAM/modules/sensor_io_module.hpp>
#include <SimpleSLAM/modules/trajectory_module.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
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
    std::uint64_t packets_without_outcome = 0; ///< 送达里程计、却没有终态的包
    std::uint64_t contract_errors = 0;         ///< 违反包契约、没进估计的帧
    std::uint64_t output_contract_errors = 0;  ///< 跟踪了、产物却违反契约而作废的帧
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
        return !cancelled && packets_without_outcome == 0 && contract_errors == 0
               && output_contract_errors == 0 && failed_frames == 0 && failed_corrections == 0
               && reinitializations == 0 && isolated_failures == 0 && foreign_clock_drops == 0
               && out_of_order_imu == 0 && dropped_triggers == 0 && rejected_triggers == 0
               && imu_backlog_drops == 0 && attachment_drops == 0 && pending_triggers == 0;
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
    [[nodiscard]] std::vector<std::string> reasons() const;

    /// 有终态、有说明、不算不完整的事实。与 reasons() 一起进报告。
    [[nodiscard]] std::vector<std::string> notes() const;
};

/// 一次运行的最终结果视图：终结时固定下来的结局与产物副本。
///
/// 终结之后对外读它，而不是读已经关闭的模块——模块的可变方法不再有意义，
/// 这份视图才是"这次运行的结果"。
struct RunResult {
    RunOutcome outcome;
    fw::Trajectory odometry;   ///< 里程计轨迹（odom 系）
    fw::Trajectory online;     ///< 在线轨迹（world 系，当时实际输出过的值）
    fw::Trajectory keyframes;  ///< 优化后的关键帧轨迹（world 系）
};

/// 装配选项。放在命名空间作用域而不是嵌套在类里：嵌套类的默认成员初始化器
/// 要到外层类结束才完整，无法用于外层类自己的默认实参。
struct SlamSystemOptions {
    /// 离线：单线程步进、虚拟时钟、可重复。在线：一个模块一个线程 + 单调时钟。
    bool offline = true;
    /// 在线时入口允许多少包在途。队列只为突发准备，冗余给足：200 包是 10 Hz 激光的 20 秒。
    /// 超过它就丢触发帧，IMU 并入下一包，并记进运行结局。离线不限，不丢。
    std::size_t online_max_in_flight = 200;
    /// 是否收集轨迹。评测、导出、回归都要它，代价是每帧一个位姿。
    bool collect_trajectory = true;
    /// 日志写到哪里、各来源的等级。日志组件是系统自带的，后台线程随系统建、随系统收尾。
    logging::LogOptions log{};
};

/// 系统里各模块的名字，也是它们日志来源的根。
inline constexpr std::string_view kSensorIoModule = "/slam/sensor_io";
inline constexpr std::string_view kOdometryModule = "/slam/odometry";
inline constexpr std::string_view kPlaceRecognizerModule = "/slam/place_recognizer";
inline constexpr std::string_view kGraphOptimizerModule = "/slam/graph_optimizer";
inline constexpr std::string_view kTrajectoryModule = "/slam/trajectory";

/// 内置模块自己用到的参数，写者是模块本身。有后端（位置识别与图优化）时才有图优化模块的。
[[nodiscard]] std::vector<comm::ParamComponent> builtinModuleParams(bool with_backend);

/// 装配、离线步进、结果查询与终结由控制线程串行调用；在线模块通过 requestStop()
/// 请求停止，控制线程待 spin() 返回后完成 shutdown()/cancel()。
class SlamSystem {
public:
    using Options = SlamSystemOptions;

    /// 通信层的异常与积压上报接到诊断汇聚上。
    explicit SlamSystem(Options options = Options{});

    [[nodiscard]] SlamResources& resources() noexcept { return resources_; }

    /// 写入标定，返回本次提交生成的版本号。
    /// 数据集自带标定时直接传 `source.calibration()`，不必手抄一遍。运行终结之后拒绝。
    std::uint64_t setCalibration(fw::CalibrationData calibration) {
        return resources_.setCalibration(std::move(calibration));
    }
    [[nodiscard]] comm::Runtime& runtime() noexcept { return runtime_; }
    [[nodiscard]] fw::MemoryDiagnosticSink& diagnostics() noexcept { return diagnostics_; }
    /// 系统自带的日志组件：一个后台线程落盘，算法线程不等 I/O。模块的日志对象是它的根；
    /// 入口经它接上 run.log，也用 `root(kOdometryModule)` 把经槽构造的插件挂到模块下面。
    [[nodiscard]] logging::Router& logRouter() noexcept { return *log_router_; }

    /// 接上数据来源。来源声明过能力时，这里就核对它与打包策略是否相容——
    /// 例如策略要求 IMU 而来源没有惯性流，装配期就失败，而不是跑起来之后每包都缺支撑。
    SensorIoModule& setSource(std::unique_ptr<fw::ISensorSource> source,
                              fw::PacketizerPolicy policy = {},
                              const std::string& name = std::string(kSensorIoModule));

    /// 数据来源声明的能力。进评测报告与实验产物。
    [[nodiscard]] const fw::SourceDescriptor& sourceDescriptor() const noexcept {
        return descriptor_;
    }

    /// 指定里程计算法。目录还原出来的实现走这个入口。
    OdometryModule& setOdometry(std::unique_ptr<plugin::OdometryBase> algorithm);
    PlaceRecognizerModule& setPlaceRecognizer(
        std::unique_ptr<plugin::PlaceRecognizerBase> algorithm);
    /// `edge_policy` 是算法没给跨关键帧约束时里程计边的兜底权重。
    GraphOptimizerModule& setGraphOptimizer(std::unique_ptr<plugin::GraphOptimizerBase> algorithm,
                                            OdometryEdgePolicy edge_policy = {});

    /// 按类型构造算法，Args 转发给它的构造函数。手工装配（不经 YAML）时用；算法的日志
    /// 来源同样挂在模块下面，构造函数里新建的子插件挂在它下面。
    template <class Algorithm, class... Args>
    OdometryModule& setOdometry(Args&&... args) {
        return setOdometry(build<Algorithm>(kOdometryModule, std::forward<Args>(args)...));
    }

    template <class Algorithm, class... Args>
    PlaceRecognizerModule& setPlaceRecognizer(Args&&... args) {
        return setPlaceRecognizer(
            build<Algorithm>(kPlaceRecognizerModule, std::forward<Args>(args)...));
    }

    template <class Algorithm, class... Args>
    GraphOptimizerModule& setGraphOptimizer(Args&&... args) {
        return setGraphOptimizer(
            build<Algorithm>(kGraphOptimizerModule, std::forward<Args>(args)...));
    }

    /// 装配完成。来源要求的外参此刻必须已知——"来源有哪些标定"与"最终装配齐没齐"
    /// 是两件事：数据集可以缺外参，只要外部标定文件补上了就行。参数存储还没定义时（没有经过
    /// 入口装配），按内置模块的声明以默认值定义；入口在这之前把全部组件的参数连同配置的值定好。
    void init();

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
    std::size_t runOffline(std::size_t max_packets = 0);

    /// 在线跑完整个数据源：一个模块一个线程，来源按数据时间的节奏放行——第一条数据对到
    /// 开始的时刻，之后数据时刻为 t 的样本到 开始 + (t − t0) 才读。来源读完、触发帧排空之后
    /// 停止输入，已接纳的工作走完才返回。返回发出的包数。
    ///
    /// 要在线模式（`SlamSystemOptions::offline = false`）。`pump_period` 是放行的检查周期，
    /// 也是节奏的分辨率；`packets_per_tick` 是落后时一次补发的上限。之后照常 shutdown()。
    /// 结果不保证逐位复现：线程调度、过载丢帧都随运行而变。
    std::size_t runOnline(comm::Duration pump_period = std::chrono::milliseconds(1),
                          std::size_t packets_per_tick = 8);

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
    void advanceClockToData();

    /// 处理到系统静止（没有就绪任务为止）。不设上限：回调无限自发布时挂住，定期上报
    /// （`comm::Runtime::spinUntilIdle`）。返回处理的批数。
    std::size_t settle();

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
    [[nodiscard]] const RunResult& result() const;

    [[nodiscard]] SensorIoModule* sensorIo() noexcept { return sensor_io_; }
    [[nodiscard]] OdometryModule* odometry() noexcept { return odometry_; }
    [[nodiscard]] PlaceRecognizerModule* placeRecognizer() noexcept { return place_recognizer_; }
    [[nodiscard]] GraphOptimizerModule* graphOptimizer() noexcept { return graph_optimizer_; }
    [[nodiscard]] TrajectoryModule* trajectory() noexcept { return trajectory_; }

private:
    void finalize(comm::ShutdownMode mode, std::string reason);

    template <class Algorithm, class... Args>
    std::unique_ptr<Algorithm> build(std::string_view module, Args&&... args) {
        std::string_view type = "plugin";
        if constexpr (requires { Algorithm::kTypeName; }) type = Algorithm::kTypeName;
        const logging::detail::ConstructionScope scope(logging::detail::componentNode(
            logging::detail::nodeOf(log_router_->root(module)), "plugin", type));
        return std::make_unique<Algorithm>(std::forward<Args>(args)...);
    }

    /// 按账汇总各模块的终态证据与产物进度。模块在 Runtime 析构之前一直存活，
    /// 关闭之后读它们同样安全。
    [[nodiscard]] RunOutcome collectOutcome() const;

    /// 系统析构的最后一步收起日志组件：模块与插件都已析构，它们的日志都已写完。
    struct RouterCloser {
        explicit RouterCloser(std::shared_ptr<logging::Router> owned) : router(std::move(owned)) {}
        ~RouterCloser() { router->close(); }
        RouterCloser(const RouterCloser&) = delete;
        RouterCloser& operator=(const RouterCloser&) = delete;
        std::shared_ptr<logging::Router> router;
    };

    Options options_;
    std::shared_ptr<logging::Router> log_router_;
    RouterCloser router_closer_;
    SlamResources resources_;
    fw::MemoryDiagnosticSink diagnostics_;
    comm::Runtime runtime_;

    fw::SourceDescriptor descriptor_;
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
