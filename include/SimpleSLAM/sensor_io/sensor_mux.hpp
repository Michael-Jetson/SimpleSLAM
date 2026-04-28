#pragma once

/// @file sensor_mux.hpp
/// 多传感器源时间排序合并器
///
/// 持有多个 ISensorSource，每次输出时间戳最早的数据。
/// v0.5 只有单源场景，但接口先建好为多源扩展做准备。

#include <SimpleSLAM/sensor_io/sensor_source.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace simpleslam {

class SensorMux {
public:
    /// 添加一个数据源（所有权转移）
    void addSource(std::unique_ptr<ISensorSource> source);

    /// 是否还有任意源有未读数据
    [[nodiscard]] bool hasNext() const;

    /// 获取所有源中时间戳最小的 LiDAR 扫描
    std::optional<LidarScan> nextScan();

    /// 已注册的数据源数量
    [[nodiscard]] size_t sourceCount() const { return sources_.size(); }

private:
    std::vector<std::unique_ptr<ISensorSource>> sources_;
};

}  // namespace simpleslam
