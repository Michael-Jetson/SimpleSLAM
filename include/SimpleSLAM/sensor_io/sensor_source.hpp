#pragma once

/// @file sensor_source.hpp
/// 传感器数据源抽象接口
///
/// 每个数据来源（KITTI、EuRoC、ROS bag 等）实现此接口。
/// v0.5 只有 LiDAR 数据源；IMU/Image 提供默认空实现，子类按需覆盖。

#include <SimpleSLAM/core/types/common.hpp>
#include <SimpleSLAM/core/types/sensor_data.hpp>

#include <optional>

namespace simpleslam {

class ISensorSource {
public:
    virtual ~ISensorSource() = default;

    /// 是否还有未读数据
    [[nodiscard]] virtual bool hasNext() const = 0;

    /// 读取下一帧 LiDAR 扫描（无数据时返回 nullopt）
    virtual std::optional<LidarScan> nextScan() = 0;

    /// 读取下一个 IMU 样本（v0.5 默认不提供）
    virtual std::optional<ImuSample> nextImu() { return std::nullopt; }

    /// 读取下一帧图像（v0.5 默认不提供）
    virtual std::optional<ImageFrame> nextImage() { return std::nullopt; }

    /// 当前数据时间戳
    [[nodiscard]] virtual Timestamp currentTimestamp() const = 0;
};

}  // namespace simpleslam
