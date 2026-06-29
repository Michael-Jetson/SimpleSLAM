#pragma once

/// @file extrinsics_manager.hpp
/// 传感器外参管理——从 YAML 加载 T_body_sensor，提供统一查询接口
///
/// v1.0 外参为固定值（从配置读入）。
/// v2.5+ 可扩展为在线标定（updateExtrinsic 接口已预留）。

#include <SimpleSLAM/core/types/geometry.hpp>

#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace simpleslam {

class ExtrinsicsManager {
public:
    ExtrinsicsManager() = default;

    /// 手动注册传感器外参
    void registerSensor(const std::string& sensor_name, const SE3d& T_body_sensor) {
        std::unique_lock lock(mutex_);
        extrinsics_[sensor_name] = T_body_sensor;
    }

    /// 查询 T_body_sensor（不存在时抛异常）。
    /// 返回【副本】——不外泄指向 mutex 保护数据的引用（否则 updateExtrinsic 并发改之 → 竞争）。
    [[nodiscard]] SE3d T_body_sensor(const std::string& sensor_name) const {
        std::shared_lock lock(mutex_);
        auto it = extrinsics_.find(sensor_name);
        if (it == extrinsics_.end()) {
            throw std::runtime_error(
                "ExtrinsicsManager: unknown sensor: " + sensor_name);
        }
        return it->second;
    }

    [[nodiscard]] bool hasSensor(const std::string& sensor_name) const {
        std::shared_lock lock(mutex_);
        return extrinsics_.count(sensor_name) > 0;
    }

    [[nodiscard]] std::vector<std::string> sensorNames() const {
        std::shared_lock lock(mutex_);
        std::vector<std::string> names;
        names.reserve(extrinsics_.size());
        for (const auto& [name, _] : extrinsics_) {
            names.push_back(name);
        }
        return names;
    }

    /// 更新外参（在线标定用，v2.5+）
    void updateExtrinsic(const std::string& sensor_name, const SE3d& T_body_sensor) {
        std::unique_lock lock(mutex_);
        extrinsics_[sensor_name] = T_body_sensor;
    }

private:
    mutable std::shared_mutex mutex_;
    std::unordered_map<std::string, SE3d> extrinsics_;
};

}  // namespace simpleslam
