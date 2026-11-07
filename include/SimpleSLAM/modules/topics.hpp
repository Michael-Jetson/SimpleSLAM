// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file topics.hpp
/// 话题名与共享资源。名字集中在这里，模块不各写各的字符串。

#include <SimpleSLAM/comm/types.hpp>
#include <SimpleSLAM/framework/calibration.hpp>
#include <SimpleSLAM/framework/events.hpp>
#include <SimpleSLAM/framework/ids.hpp>
#include <SimpleSLAM/framework/keyframe.hpp>
#include <SimpleSLAM/framework/odometry_result.hpp>
#include <SimpleSLAM/framework/pose_graph.hpp>
#include <SimpleSLAM/framework/sensor_packet.hpp>
#include <SimpleSLAM/framework/solution.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <cstdint>
#include <mutex>

namespace simpleslam::modules {

namespace topics {

using comm::TopicDesc;

/// 名字与消息类型一起登记：发布、订阅经描述符走，类型对不上编译不过。
inline constexpr TopicDesc<fw::SensorPacket> kSensorPacket{"/slam/sensor_packet"};
inline constexpr TopicDesc<fw::PacketConsumed> kPacketConsumed{"/slam/packet_consumed"};
inline constexpr TopicDesc<fw::OdometryResult> kOdometry{"/slam/odometry"};
inline constexpr TopicDesc<fw::KeyframeEvent> kKeyframe{"/slam/keyframe"};
inline constexpr TopicDesc<fw::LoopConstraint> kLoopConstraint{"/slam/loop_constraint"};
inline constexpr TopicDesc<fw::CorrectionEvent> kCorrection{"/slam/correction"};
inline constexpr TopicDesc<fw::TrackingLostEvent> kTrackingLost{"/slam/tracking_lost"};
inline constexpr TopicDesc<fw::RelocationRequest> kRelocationRequest{"/slam/relocation_request"};
inline constexpr TopicDesc<fw::RelocationResult> kRelocationResult{"/slam/relocation_result"};
inline constexpr TopicDesc<fw::MapRebuildRequest> kMapRebuildRequest{"/slam/map_rebuild_request"};
inline constexpr TopicDesc<fw::HealthEvent> kHealth{"/slam/health"};

}  // namespace topics

/// 全系统共享的资源。由 SlamSystem 持有，模块按需取读快照或写令牌。
///
/// 写者归属（装配期确定，写令牌保证）：
///   keyframes → 里程计模块；graph → 图优化模块；state → 里程计模块；
///   calibration → 装配期写入。
struct SlamResources {
    fw::KeyframeStore keyframes;
    fw::PoseGraph graph;
    fw::CurrentState state;
    /// 标定是资源，不是配置字段：多模块读、单写者、会被在线标定改（calibration.hpp）。
    fw::Calibration calibration;
    fw::ClockRegistry clocks;
    fw::SessionId session{1};

    SlamResources() : calibration_writer_(calibration.acquireWriteToken()) {}

    /// 提交一份标定，并在这里**生成**它的版本号。返回新版本。
    ///
    /// 版本与内容同出一源：文件里写的 revision 一律忽略（两份不同的标定都可以
    /// 声称自己是 1），持有者也不自行递增。只有经过这个入口，
    /// "这一帧用的是哪份标定"才有唯一答案。
    ///
    /// 整段"读当前版本 → 生成新版本 → 提交"在同一把锁下完成：背后只有一个写令牌，
    /// 不等于只有一个线程在调这个公开入口。两个线程各自读到同一个旧版本再提交，
    /// 就会一个修改丢失、数据版本与资源版本对不上。提交前按 validateCalibration()
    /// 校验，坏的标定不进真源。
    std::uint64_t setCalibration(fw::CalibrationData data);

    /// 运行终结时冻结：此后标定写入一律拒绝。其余资源的写令牌都在模块手里，
    /// 模块收尾之后不再有回调，它们自然不再变。
    void freeze();

    /// 就地改一份标定（在线标定用）。读-改-写与版本生成是同一个受锁操作。
    template <class F>
    std::uint64_t modifyCalibration(F&& mutate) {
        std::lock_guard lock(calibration_mutex_);
        auto data = *calibration.snapshot();
        mutate(data);
        return commitCalibrationLocked(std::move(data));
    }

    /// 当前标定版本。异步结果的身份靠它判断是否还成立（不变量 15）。
    ///
    /// 需要"算法用的和产物标的是同一份"时，取一次 calibrationSnapshot() 并全程用它，
    /// 不要先算完再来读最新版本贴标签。
    [[nodiscard]] std::uint64_t calibrationRevision() const {
        return calibration.snapshot()->revision;
    }
    [[nodiscard]] fw::Calibration::Snapshot calibrationSnapshot() const {
        return calibration.snapshot();
    }

private:
    std::uint64_t commitCalibrationLocked(fw::CalibrationData data);

    std::mutex calibration_mutex_;
    fw::WriteToken<fw::CalibrationData> calibration_writer_;
    bool frozen_ = false;  ///< 由 calibration_mutex_ 保护
};

}  // namespace simpleslam::modules
