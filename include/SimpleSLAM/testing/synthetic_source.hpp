// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file synthetic_source.hpp
/// 合成的激光 + IMU 来源：传感器在平面块场景（`planePatches()`）里沿 x 匀速前进，激光 10 Hz、
/// IMU 200 Hz。扫描在参考时刻一次看到全部点，没有运动畸变，也不带逐点时间；匀速运动下 IMU
/// 只量到重力。给方案的契约测试用，不是数据集，精度数字没有意义。

#include <SimpleSLAM/framework/sensor_source.hpp>
#include <SimpleSLAM/testing/scenes.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace simpleslam::testing {

/// 一帧扫描的样子：场景本身，或某种退化输入。
enum class SyntheticScan : std::uint8_t {
    kScene,     ///< 三块互相垂直的平面
    kEmpty,     ///< 没有点
    kSparse,    ///< 少于 10 个点
    kCoplanar,  ///< 只有地面
};

struct SyntheticSequence {
    int scans = 20;        ///< 扫描帧数
    double step_m = 0.1;   ///< 每帧沿 x 前进的距离
    /// 退化帧：帧号（从 0 起）到它的样子，其余帧是场景。
    std::map<int, SyntheticScan> degenerate{};
};

class SyntheticLidarImuSource final : public fw::ISensorSource {
public:
    static constexpr std::int64_t kScanPeriodNs = 100'000'000;  // 10 Hz
    static constexpr std::int64_t kImuPeriodNs = 5'000'000;     // 200 Hz

    explicit SyntheticLidarImuSource(SyntheticSequence sequence = {})
        : sequence_(std::move(sequence)) {}

    /// 第 k 帧（从 0 起）扫描的参考时刻。
    [[nodiscard]] static fw::Timestamp scanTime(int k) {
        return fw::Timestamp{static_cast<std::int64_t>(k + 1) * kScanPeriodNs};
    }

    [[nodiscard]] std::optional<fw::SensorDatum> next() override {
        if (emitted_ >= sequence_.scans) return std::nullopt;
        const std::int64_t scan_ns = scanTime(emitted_).count();
        if (next_imu_ns_ <= scan_ns) {
            auto sample = std::make_shared<fw::ImuSample>();
            sample->time = fw::Timestamp{next_imu_ns_};
            sample->accel = {0.0, 0.0, 9.81};  // 匀速：比力只有重力的反作用
            next_imu_ns_ += kImuPeriodNs;
            return fw::SensorDatum{fw::Ptr<fw::ImuSample>(std::move(sample))};
        }
        auto scan = std::make_shared<fw::LidarScan>();
        scan->reference_time = fw::Timestamp{scan_ns};
        scan->coverage =
            fw::TimeRange{fw::Timestamp{scan_ns - kScanPeriodNs}, scan->reference_time};
        scan->sensor = "lidar";
        const SE3d T_world_sensor(Vec3d(sequence_.step_m * emitted_, 0.0, 0.0),
                                  Eigen::Quaterniond::Identity());
        scan->points = transformed(cloudFor(emitted_), T_world_sensor.inverse());
        ++emitted_;
        return fw::SensorDatum{fw::Ptr<fw::LidarScan>(std::move(scan))};
    }

    [[nodiscard]] std::string name() const override { return "SyntheticLidarImuSource"; }

private:
    [[nodiscard]] fw::PointCloud cloudFor(int k) const {
        const auto it = sequence_.degenerate.find(k);
        switch (it == sequence_.degenerate.end() ? SyntheticScan::kScene : it->second) {
            case SyntheticScan::kEmpty: return {};
            case SyntheticScan::kSparse: return sparse();
            case SyntheticScan::kCoplanar: return coplanar();
            case SyntheticScan::kScene: break;
        }
        return planePatches();
    }

    SyntheticSequence sequence_;
    int emitted_ = 0;
    std::int64_t next_imu_ns_ = 0;
};

}  // namespace simpleslam::testing
