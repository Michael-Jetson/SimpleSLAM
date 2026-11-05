// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file sensor_packet.hpp
/// 传感器数据包：里程计的唯一入口，模态差异由包内容表达，不由入口枚举表达。
///
/// 打包器必须保证的时间契约（提案 3.2.1）：
/// - 包的参考时刻独立于附件的采集时刻，附件各自带自己的时刻；
/// - 状态传播从"上次已接纳的参考时刻"（接纳水位）起算；对惯性支撑的需求是传播区间
///   与本帧采集区间的并集，首帧的需求从采集起点算，不由现有数据反推；
/// - 允许在线校准时间偏置时，要预留前后支撑区间；
/// - 缺口、失同步、EOF 都是显式字段，不是静默丢弃；
/// - 包拥有它引用的数据的寿命。

#include <SimpleSLAM/comm/types.hpp>
#include <SimpleSLAM/framework/sensor_data.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <cstdint>
#include <variant>
#include <vector>

namespace simpleslam::fw {

template <class T>
using Ptr = simpleslam::comm::MsgPtr<T>;

/// 触发这一帧的传感器。VIO 由相机触发，LIO 由扫描触发，结构相同。
using Trigger = std::variant<Ptr<LidarScan>, Ptr<CameraFrame>>;

/// 与本包参考时刻相近、但各有自身采集时刻的其他数据。
struct Attachments {
    std::vector<Ptr<CameraFrame>> images;
    std::vector<Ptr<GnssFix>> gnss;
};

/// 数据缺口的原因。默认 kNone；一旦非 kNone，下游必须显式处理。
enum class GapReason : std::uint8_t {
    kNone,
    kDroppedTrigger,   ///< 过载丢了触发帧（IMU 已并入本包）
    kImuGap,           ///< IMU 区间不连续：端点缺支撑，或中间相邻样本间隔超过要求
    kClockReset,       ///< 时间倒流或重复：乱序样本已被丢弃
    kSourceRestart,    ///< 数据源重启
    kClockDomain,      ///< 有数据不在打包器的参考时钟域里，已被拒收
};

struct SensorPacket {
    /// 发布序号，单调递增。消费者据此发现缺口并补读，不靠"事件不丢"假设。
    std::uint64_t sequence = 0;
    Trigger trigger;
    /// 本包参考时刻：等于触发传感器的参考时刻（扫描终点 / 图像曝光中点）。
    Timestamp reference_time{0};
    /// 状态传播区间：[上次已接纳的参考时刻, reference_time + 余量]。估计器从起点的
    /// 状态积分到终点。首包没有"上次接纳"，起点取本帧采集的起点。
    TimeRange propagation{};
    /// 本包对惯性支撑的**需求**：状态传播区间与本帧采集区间（扫描覆盖）的并集。
    /// 需求由触发帧与接纳水位决定，**不由现有数据反推**——首帧的需求从扫描起点算，
    /// 即使第一条 IMU 晚于它。
    TimeRange imu_required{};
    /// IMU 实际支撑的区间：两端有括点可插值的那一段。它盖不住需求时 gap 为 kImuGap，
    /// 并且如实收窄到实际可插值的范围，不声明一个取不到的区间。
    TimeRange imu_coverage{};
    /// 拥有型句柄。跨线程发布后由它保证寿命，取 view() 只在同步调用内部用。
    Batch<ImuSample> imu;
    Attachments attachments;
    GapReason gap = GapReason::kNone;
    /// 本包参考时刻两侧为在线时间偏置校准预留的支撑区间。
    Duration time_offset_support{0};
    /// 本包 IMU 样本里相邻两条的最大间隔。两端有样本不等于中间连续，
    /// 算法据此判断预积分可不可信；超过策略要求时 gap 为 kImuGap。
    Duration imu_max_gap{0};

    [[nodiscard]] bool triggeredByLidar() const noexcept {
        return std::holds_alternative<Ptr<LidarScan>>(trigger);
    }
    [[nodiscard]] const LidarScan* lidar() const noexcept {
        auto* p = std::get_if<Ptr<LidarScan>>(&trigger);
        return (p != nullptr && *p) ? p->get() : nullptr;
    }
    [[nodiscard]] const CameraFrame* camera() const noexcept {
        auto* p = std::get_if<Ptr<CameraFrame>>(&trigger);
        return (p != nullptr && *p) ? p->get() : nullptr;
    }

    /// 实际支撑是否盖住了需求。false 的包就是缺支撑的包，不能当完整包处理。
    [[nodiscard]] bool supportComplete() const {
        return !imu.view().empty() && imu_coverage.begin <= imu_required.begin
               && imu_coverage.end >= imu_required.end;
    }

    /// IMU 是否真的覆盖了声明的区间。打包器发包前、里程计消费前各查一次。
    [[nodiscard]] bool imuCoversDeclaredRange() const {
        const auto samples = imu.view();
        if (samples.empty()) return imu_coverage.empty();
        return samples.front().time <= imu_coverage.begin
               && samples.back().time >= imu_coverage.end;
    }
};

}  // namespace simpleslam::fw
