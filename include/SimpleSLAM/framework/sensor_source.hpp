// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file sensor_source.hpp
/// 数据来源与打包器：类型固定，来源开放。
///
/// 来源只需要产出四类内部类型之一；打包器把它们组织成 SensorPacket，并负责时间契约。
///
/// ## 一条水位线
///
/// **生成水位** `generated_until_`：已经组装出包的最后一个参考时刻。下一包的 IMU 区间从这里起算，
/// 更早的原始 IMU 也从这里开始回收。包拷走了自己的 IMU，所以回收不需要等下游。
///
/// ## 过载只丢触发帧
///
/// 下游处理不过来时，能丢的只有触发帧，不能丢 IMU：状态传播要连续，丢一段 IMU 算法就接不上了。
/// 在途的包（已发出、下游还没报告处理完）达到上限时，凑齐括点的触发帧直接丢掉，
/// 生成水位不动，它那段 IMU 自然并入下一包。
///
/// ## IMU 是能力，不是前提
///
/// 发包是否必须等 IMU，由策略表达（ImuRequirement），不写死。纯 LiDAR 里程计
/// 声明 kNotRequired 就能发包；LIO 声明 kRequired 并给出 post_support，
/// 打包器按水位线等到支撑区间到齐再发。
///
/// ## 边界样本
///
/// 去畸变与预积分要的是区间端点上的**插值**，不是最近邻。所以取样时向两侧各多取
/// 一个跨过端点的样本；取不到就如实收窄声明区间并标缺口，而不是声明一个取不到的区间。

#include <SimpleSLAM/framework/sensor_packet.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace simpleslam::fw {

/// 来源产出的一条数据。
using SensorDatum = std::variant<Ptr<LidarScan>, Ptr<CameraFrame>, Ptr<ImuSample>, Ptr<GnssFix>>;

/// 来源走到尽头时的终态。
///
/// "next() 返回空"只说明没有下一条了，说不清是**正常读完**还是**读到一半没了**
/// ：回放一个被截断的录制文件，SensorIO 看到的
/// 同样是一个空 optional，于是把不完整的来源当成正常 EOF，实验产物里也就没有
/// 任何痕迹。允许拿完整前缀继续跑是合理的，但那必须是一个明确选项并写进报告。
enum class SourceOutcome : std::uint8_t {
    kRunning,     ///< 还没结束
    kCleanEof,    ///< 正常读完，数据完整
    kIncomplete,  ///< 读到了一个完好的前缀，但文件/流不完整（截断、缺结束标记）
    kError,       ///< 读取过程中出错，已停止
};

[[nodiscard]] constexpr std::string_view toString(SourceOutcome outcome) noexcept {
    switch (outcome) {
        case SourceOutcome::kRunning: return "Running";
        case SourceOutcome::kCleanEof: return "CleanEof";
        case SourceOutcome::kIncomplete: return "Incomplete";
        case SourceOutcome::kError: return "Error";
    }
    return "Unknown";
}

/// 逐点时间是量出来的还是算出来的。
enum class PointTimeSource : std::uint8_t {
    kAbsent,       ///< 没有逐点时间
    kMeasured,     ///< 传感器给出
    kReconstructed,///< 由方位角等几何量反推。相位、切帧位置、旋向都是假设
};

/// 参考时刻在物理上对应扫描的哪一刻。
///
/// 数据集给的那一个时间戳（KITTI odometry 的 times.txt）只能说明"内部标签一致"，
/// 说不清它是起点、终点还是相机触发时刻。标成 kDatasetProvided 而不是硬说成终点。
enum class ReferenceKind : std::uint8_t {
    kUnknown,
    kScanStart,
    kScanEnd,
    kTrigger,          ///< 外部触发时刻（如前向相机触发）
    kDatasetProvided,  ///< 数据集给定，物理含义未经证实
};

/// 来源的能力描述。
///
/// 装配要据它决定打包策略、能不能去畸变、真值能不能直接比，所以这些必须是字段，
/// 不能靠类名暗示——"名字里有 Raw 就一定有完整惯性流"不是一个能验证的说法。
struct SourceDescriptor {
    /// 来源是否真的声明了下面这些能力。默认实现不声明——不声明比猜一个好，
    /// 装配时也只对声明过的来源做能力核对。
    bool declared = false;
    std::string name;
    std::string body_sensor;        ///< body 系绑在哪个传感器上
    MotionCompensation motion_compensation = MotionCompensation::kUnknown;
    PointTimeSource point_time_source = PointTimeSource::kAbsent;
    ReferenceKind reference_kind = ReferenceKind::kUnknown;
    bool has_imu = false;           ///< 有惯性流
    bool has_navigation = false;    ///< 有导航状态（位姿/速度），与惯性流是两回事
    double imu_rate_hz = 0.0;       ///< **实测**采样率，不是标称值
    Duration max_imu_gap{0};        ///< 实测的最大相邻间隔
    TimeRange coverage{};           ///< 实际时间范围
    /// 评测的**目标窗口**：本次运行应当产出位姿的时间范围——选中的第一帧到最后一帧的
    /// 参考时刻。任务完成度按它算；为端点插值多留的括点支撑不在其内，
    /// 不能把"没用到的支撑样本"算成"没完成的任务"。
    TimeRange evaluation_window{};
    /// 时间修补政策。空表示没有修补任何数据；非空时写明修补方式（例如按标称周期
    /// 回推扫描起点），受影响的帧列在 repaired_frames。修补是显式选项，默认拒绝异常数据。
    std::string timing_repair;
    /// 被修补过时间的帧（来源自己的帧号）。与政策一起进有效身份：修补与否、修补了
    /// 哪些帧，结论都不再建立在同一份数据上。
    std::vector<std::size_t> repaired_frames;
    bool has_calibration = false;
    bool has_ground_truth = false;
    /// 装配时必须已知的外参（通常是触发传感器相对 body 的那一条）。来源只负责
    /// 说"需要哪些"，外参由谁提供——数据集自带还是外部标定文件——是装配的事。
    std::vector<std::string> required_extrinsics;
    /// 允许用假设值（kAssumed）顶替缺失外参。显式选项，会记进产物。
    bool assumed_extrinsics_allowed = false;
    /// 解释性的来源说明：同源关系、缺省解释、修补过的数据等。进评测报告，
    /// 让读结论的人知道这些数字建立在什么前提上。
    std::vector<std::string> notes;
};

/// 数据来源。离线是数据集读取器，在线是驱动适配器。
class ISensorSource {
public:
    virtual ~ISensorSource() = default;
    /// 取下一条数据；返回空表示数据结束（EOF 是显式状态，不是超时）。
    [[nodiscard]] virtual std::optional<SensorDatum> next() = 0;
    [[nodiscard]] virtual std::string name() const { return "ISensorSource"; }

    /// 能力描述。默认什么都不声明——不声明比猜一个好。
    [[nodiscard]] virtual SourceDescriptor descriptor() const {
        SourceDescriptor descriptor;
        descriptor.name = name();
        return descriptor;
    }

    /// 结束时的终态。默认实现对"能一次读完的来源"成立；凡是可能读到不完整数据的
    /// 来源（回放、网络流）都必须重写它，否则上层无从知道这次输入是不是完整的。
    [[nodiscard]] virtual SourceOutcome outcome() const { return SourceOutcome::kCleanEof; }
};

/// 发包是否必须有 IMU 支撑。这是能力声明，不是代码分支。
enum class ImuRequirement : std::uint8_t {
    kRequired,     ///< 必须等到覆盖区间的 IMU（LIO / VIO：去畸变与预积分都要）
    kNotRequired,  ///< 有就带上，没有也发（纯 LiDAR 里程计）
};

/// 打包策略。写成配置，不写成代码分支。
///
/// 三件事要分开：**测量间隔**是来源的事实（SourceDescriptor::max_imu_gap）；
/// **时间偏置余量**是算法的选择（post_support）；**端点插值的括点**是打包器的职责——
/// 终点之后第一条样本一到就能插值，不需要额外等。把测量间隔直接当成等待余量，
/// 会让低频惯性流下每个触发帧都等过下一帧，单槽等待时一帧也发不出来。
struct PacketizerPolicy {
    /// 触发传感器：LIO 由扫描触发，VIO 由相机触发。
    bool trigger_on_lidar = true;
    /// 发包是否必须有 IMU 支撑。
    ImuRequirement imu = ImuRequirement::kRequired;
    /// 参考时刻之后**额外**要等的时间偏置余量。零表示只等到终点的右括点。
    Duration post_support{0};
    /// 参考时刻**之前**额外保留的历史。只影响原始数据何时可回收，不影响发包。
    Duration pre_support{0};
    /// 算法要求的相邻 IMU 样本最大间隔。包内超过它就标 kImuGap；零表示不查。
    /// 默认只拦明显断流；做预积分的 LIO 应按自己的需要在配置里收紧。
    Duration max_imu_gap = std::chrono::milliseconds(500);
    /// 打包器的参考时钟域。不在这个域里的数据一律拒收并标 kClockDomain——
    /// 跨域融合必须先经显式换算，不能在这里悄悄混用。
    ClockDomainId clock_domain = kReferenceClock;
    /// IMU 积压上限。超限即显式报缺口，而不是无声增长。
    std::size_t max_imu_backlog = 100000;
    /// 等待括点的触发帧队列上限。低频惯性流下几个触发帧可能同时在等，IMU 晚到时等得更多。
    /// 冗余给足：200 帧是 10 Hz 激光的 20 秒。超限丢最旧的，它那段 IMU 并入下一包。
    std::size_t max_pending_triggers = 200;
    /// 在途上限：已发出、下游还没报告处理完的包数。达到上限时凑齐括点的触发帧直接丢掉，
    /// 它那段 IMU 并入下一包。0 表示不限：离线逐包步进，不需要丢。
    std::size_t max_in_flight = 0;
    /// 待附着的图像 / GNSS 上限。附不上的旧附件按时间窗清掉，数量另有此上限。
    std::size_t max_pending_attachments = 256;
    /// 附件的时间容差：超过它就不附带，而不是硬塞一个最近的。
    Duration attachment_tolerance = std::chrono::milliseconds(50);
};

/// 打包器。把散落的数据流组织成包，保证时间契约。
///
/// ## 使用方式
///
/// feed() 喂一条数据，可能返回一个包；一条 IMU 可能同时让队列里好几个触发帧凑齐
/// 括点，所以之后要反复 poll() 直到它返回空。要自己控制每一步发几个包时，用 ingest()
/// 只收数据，再按预算 poll()。
class Packetizer {
public:
    explicit Packetizer(PacketizerPolicy policy = {}) : policy_(policy) {}

    /// 喂入一条数据；若有触发帧凑齐了括点，返回最早的那个包。
    std::optional<SensorPacket> feed(const SensorDatum& datum);

    /// 只收下一条数据，不出包。出包交给 poll() / flush()：调用方据此控制每一步发几个
    /// 包——一条 IMU 可能同时让好几帧凑齐括点，一步只该发预算允许的那几个。
    void ingest(const SensorDatum& datum) { accept(datum); }

    /// 取下一个已凑齐的包。没有凑齐的触发帧时返回空。下游处理不过来时，凑齐的触发帧在这里被丢掉。
    std::optional<SensorPacket> poll();

    /// 数据结束。还在等括点的触发帧按"已有多少 IMU 就声明多少"逐个发出，
    /// 并标 kImuGap——丢掉最后几帧不如如实交代它们缺支撑。每次调用发一个。
    /// 这里不看在途上限：剩下的帧有限，也没有更新的帧要让路。
    std::optional<SensorPacket> flush();

    /// 下游的处理进度：序号不大于 sequence 的包都处理完了。累计的，丢一条由下一条补上。
    void notifyConsumedThrough(std::uint64_t sequence) noexcept {
        consumed_through_ = std::max(consumed_through_, std::min(sequence, sequence_));
    }

    /// 在途上限。由装配决定：有处理进度可看时才设。
    void setMaxInFlight(std::size_t limit) noexcept { policy_.max_in_flight = limit; }

    [[nodiscard]] bool waitingForWatermark() const noexcept { return !pending_triggers_.empty(); }
    /// 正在等括点的触发帧数。
    [[nodiscard]] std::size_t pendingTriggers() const noexcept { return pending_triggers_.size(); }
    /// 因为不在参考时钟域里而被拒收的数据条数。
    [[nodiscard]] std::uint64_t foreignClockDrops() const noexcept { return foreign_clock_drops_; }
    /// 因为时间倒流或重复而被丢弃的 IMU 条数。
    [[nodiscard]] std::uint64_t outOfOrderImu() const noexcept { return imu_out_of_order_; }
    /// 被丢掉的触发帧数：下游处理不过来，或者等括点的队列满了。它们的 IMU 都并入了下一包。
    [[nodiscard]] std::uint64_t droppedTriggers() const noexcept { return dropped_triggers_; }
    /// 因为时刻不前进而被拒收的触发帧数。
    [[nodiscard]] std::uint64_t rejectedTriggers() const noexcept { return rejected_triggers_; }
    /// 因为 IMU 积压超限而被丢掉的样本数。
    [[nodiscard]] std::uint64_t imuBacklogDrops() const noexcept { return imu_backlog_drops_; }
    /// 因为待附着队列超限而被丢掉的图像 / GNSS 条数。
    [[nodiscard]] std::uint64_t attachmentDrops() const noexcept { return attachment_drops_; }
    /// 仍在等待附着的图像与 GNSS 条数。
    [[nodiscard]] std::size_t pendingAttachments() const noexcept {
        return pending_images_.size() + pending_gnss_.size();
    }
    [[nodiscard]] std::size_t imuBacklog() const noexcept { return imu_.size(); }
    /// 在途的包数：已发出、下游还没报告处理完。
    [[nodiscard]] std::size_t inFlight() const noexcept {
        return static_cast<std::size_t>(sequence_ - consumed_through_);
    }
    [[nodiscard]] std::uint64_t sequence() const noexcept { return sequence_; }
    [[nodiscard]] Timestamp generatedUntil() const noexcept { return generated_until_; }
    /// **数据时间水位**：已收下的数据（参考时钟域内、时间有序）里最晚的时刻。
    ///
    /// 与生成水位是两件事：一段只有 IMU、迟迟等不到触发帧的数据，生成水位纹丝不动，
    /// 数据时间却一直在走。离线的"现在"跟的是它——否则长时间没有触发帧时，
    /// 超时、恢复、健康检查都拿不到正确的时间。
    [[nodiscard]] Timestamp observedUntil() const noexcept { return observed_until_; }
    /// 收下的第一条数据的时刻。离线运行据此确定时钟原点。还没收到数据时为空。
    [[nodiscard]] std::optional<Timestamp> firstObserved() const noexcept { return first_observed_; }

private:
    struct PendingTrigger {
        Trigger trigger;
        Timestamp reference_time{0};
    };

    /// 收下一条数据。时钟域不对的一律拒收：跨域融合要先经显式换算。
    void accept(const SensorDatum& datum);
    [[nodiscard]] bool acceptDomain(ClockDomainId domain);
    void pushImu(const Ptr<ImuSample>& sample);
    void observe(Timestamp time) noexcept;
    void setTrigger(Trigger trigger, Timestamp reference_time);
    template <class T>
    void pushAttachment(std::deque<Ptr<T>>& queue, const Ptr<T>& item);

    /// 队首的触发帧凑齐了没有：要 IMU 时等到终点（加算法要的余量）之后的第一条样本，那就是右括点。
    [[nodiscard]] bool frontReady() const;
    [[nodiscard]] bool overloaded() const noexcept;
    /// 丢掉队首的触发帧。生成水位不动，它那段 IMU 留给下一包，所以这不算缺支撑。
    void dropFront();
    SensorPacket buildFront();
    SensorPacket buildPacket(Trigger trigger, Timestamp reference_time);

    /// 取 [begin, end] 的样本，并向两侧各多取一个跨越端点的样本。
    /// 不从 imu_ 里移除——回收由 trimImu() 跟着生成水位做。
    [[nodiscard]] std::vector<ImuSample> takeImuSpan(Timestamp begin, Timestamp end) const;
    /// 回收原始 IMU：保留 生成水位 - pre_support 之后的全部样本，外加一个跨越该时刻的边界样本。
    void trimImu();
    template <class T>
    std::vector<Ptr<T>> takeNear(std::deque<Ptr<T>>& queue, Timestamp reference_time);
    template <class T>
    void pruneStale(std::deque<Ptr<T>>& queue, Timestamp reference_time);

    static Timestamp timeOf(const CameraFrame& frame) { return frame.capture_time; }
    static Timestamp timeOf(const GnssFix& fix) { return fix.time; }
    /// 本帧采集的起点：扫描取覆盖区间的起点，相机取曝光时刻。
    static Timestamp captureBegin(const Trigger& trigger, Timestamp reference_time);

    PacketizerPolicy policy_;
    std::deque<PendingTrigger> pending_triggers_;
    std::uint64_t foreign_clock_drops_ = 0;
    std::uint64_t imu_out_of_order_ = 0;
    std::uint64_t dropped_triggers_ = 0;
    std::uint64_t rejected_triggers_ = 0;
    std::uint64_t imu_backlog_drops_ = 0;
    std::uint64_t attachment_drops_ = 0;
    std::deque<Ptr<ImuSample>> imu_;
    std::deque<Ptr<CameraFrame>> pending_images_;
    std::deque<Ptr<GnssFix>> pending_gnss_;
    std::uint64_t consumed_through_ = 0;
    std::uint32_t dropped_since_packet_ = 0;

    Timestamp generated_until_{0};
    Timestamp observed_until_{0};
    std::optional<Timestamp> first_observed_;
    bool has_generated_ = false;

    GapReason pending_gap_ = GapReason::kNone;
    std::uint64_t sequence_ = 0;
};

}  // namespace simpleslam::fw
