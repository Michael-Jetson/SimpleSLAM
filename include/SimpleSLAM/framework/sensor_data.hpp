// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file sensor_data.hpp
/// 内部传感器数据类型：类型固定，来源开放。
///
/// 两条约定：
/// 1. 点云按属性分列（SoA），热路径按访问模式取列，不为每个点构造对象。
/// 2. 跨线程发布的数据由不可变批次句柄持有寿命，`std::span` 只在同步调用内部当视图用。

#include <SimpleSLAM/framework/time.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace simpleslam::fw {

/// 不可变批次句柄。发布后不得再有可变别名。
template <class T>
class Batch {
public:
    Batch() = default;
    explicit Batch(std::vector<T> data)
        : data_(std::make_shared<const std::vector<T>>(std::move(data))) {}

    [[nodiscard]] std::span<const T> view() const noexcept {
        return data_ ? std::span<const T>(*data_) : std::span<const T>{};
    }
    [[nodiscard]] std::size_t size() const noexcept { return data_ ? data_->size() : 0U; }
    [[nodiscard]] bool empty() const noexcept { return size() == 0U; }
    [[nodiscard]] const T& operator[](std::size_t i) const { return (*data_)[i]; }

private:
    std::shared_ptr<const std::vector<T>> data_;
};

/// 按属性分列的点云。x/y/z 必有，其余列可空（空表示该属性不可用）。
struct PointCloud {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    std::vector<float> intensity;
    /// 每点相对 LidarScan::reference_time 的偏移，纳秒。去畸变与重切扫描都靠它。
    std::vector<std::int32_t> time_offset_ns;
    std::vector<std::uint16_t> ring;

    [[nodiscard]] std::size_t size() const noexcept { return x.size(); }
    [[nodiscard]] bool empty() const noexcept { return x.empty(); }

    void reserve(std::size_t n) {
        x.reserve(n);
        y.reserve(n);
        z.reserve(n);
    }

    void push(float px, float py, float pz) {
        x.push_back(px);
        y.push_back(py);
        z.push_back(pz);
    }

    /// 各列长度一致（可空列除外）。装配期与解码后各校验一次。
    [[nodiscard]] bool consistent() const noexcept {
        const std::size_t n = x.size();
        if (y.size() != n || z.size() != n) return false;
        if (!intensity.empty() && intensity.size() != n) return false;
        if (!time_offset_ns.empty() && time_offset_ns.size() != n) return false;
        if (!ring.empty() && ring.size() != n) return false;
        return true;
    }
};

enum class ShutterKind : std::uint8_t { kGlobal, kRolling };

/// 点云是否已经做过运动补偿。
///
/// 这件事不能从"有没有逐点时间列"反推：有的数据集发布的是补偿过的坐标却不带时间，
/// 有的带时间却没补偿。猜错的代价是补偿两次——第一次由数据集做，第二次由我们做。
enum class MotionCompensation : std::uint8_t {
    kUnknown,      ///< 未声明。要去畸变就必须先问清楚，不能默认
    kRaw,          ///< 原始扫描，带畸变
    kCompensated,  ///< 已由数据集补偿到参考时刻
};

/// 一帧激光扫描。参考时刻按公约取扫描终点（去畸变对准的时刻）。
struct LidarScan {
    ClockDomainId domain{};
    Timestamp reference_time{0};
    /// 首点到末点的实际采样区间。重切扫描后仍然成立。
    TimeRange coverage{};
    PointCloud points;
    std::string sensor;  ///< 设备档案 id
    /// 这一帧的点是否已经补偿到参考时刻。由读取器按数据集的事实写入，
    /// 去畸变据此决定补不补——对补偿过的扫描再补一次，同一段自车运动就被扣了两遍。
    MotionCompensation motion_compensation = MotionCompensation::kUnknown;
};

/// 一帧图像。采集时刻与曝光区间分开记录：附件按自身时刻查询轨迹。
struct CameraFrame {
    ClockDomainId domain{};
    Timestamp capture_time{0};  ///< 约定：曝光中点
    TimeRange exposure{};
    ShutterKind shutter = ShutterKind::kGlobal;
    Duration line_time{0};  ///< 卷帘行曝光间隔，全局快门为 0
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint8_t channels = 1;
    std::vector<std::uint8_t> data;  ///< 行优先 HWC
    std::string sensor;

    [[nodiscard]] bool consistent() const noexcept {
        return data.size()
               == static_cast<std::size_t>(width) * height * static_cast<std::size_t>(channels);
    }
};

/// 一条 IMU 样本。角速度 rad/s，加速度 m/s^2（比力，含重力）。
struct ImuSample {
    ClockDomainId domain{};
    Timestamp time{0};
    std::array<double, 3> gyro{0.0, 0.0, 0.0};
    std::array<double, 3> accel{0.0, 0.0, 0.0};
};

enum class GnssFixKind : std::uint8_t { kNone, kSingle, kDifferential, kFloatRtk, kFixedRtk };

/// 一次 GNSS 定位。位置在 ECEF，协方差同系。
struct GnssFix {
    ClockDomainId domain{};
    Timestamp time{0};
    std::array<double, 3> ecef{0.0, 0.0, 0.0};
    std::array<double, 9> covariance{};
    GnssFixKind kind = GnssFixKind::kNone;
    std::uint8_t satellites = 0;
};

}  // namespace simpleslam::fw
