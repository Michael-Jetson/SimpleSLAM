// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/framework/calibration.hpp>

#include <cmath>
#include <span>
#include <string>
#include <vector>

#include "matrix_checks.hpp"

namespace simpleslam::fw {

std::string validateCalibration(const CalibrationData& calibration) {
    const auto finitePositive = [](double v) { return std::isfinite(v) && v > 0.0; };
    const auto finiteNonNegative = [](double v) { return std::isfinite(v) && v >= 0.0; };

    for (const auto& [sensor, extrinsic] : calibration.extrinsics) {
        const std::string where = "extrinsics." + sensor;
        if (!extrinsic.T_body_sensor.translation().allFinite()) return where + " 的平移含非有限值";
        const auto& q = extrinsic.T_body_sensor.quat();
        if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.0) > 1e-6) {
            return where + " 的旋转不是单位四元数";
        }
        if (!extrinsic.covariance_known && !(extrinsic.covariance.array() == 0.0).all()) {
            // 有值却没声明为已知：这组数没有含义，也导不出去。静默丢掉比报错更糟
            return where + " 的协方差有值，却没有声明 covariance_known";
        }
        if (extrinsic.covariance_known) {
            const Mat6d& c = extrinsic.covariance;
            if (!c.allFinite()) return where + " 的协方差含非有限值";
            switch (detail::definiteness(c)) {
                case detail::Definiteness::kAsymmetric: return where + " 的协方差不对称";
                case detail::Definiteness::kIndefinite: return where + " 的协方差不是半正定的";
                case detail::Definiteness::kSymmetricPsd: break;
            }
        }
    }
    const auto& imu = calibration.imu;
    if (!finiteNonNegative(imu.gyro_noise_density) || !finiteNonNegative(imu.accel_noise_density)
        || !finiteNonNegative(imu.gyro_bias_random_walk)
        || !finiteNonNegative(imu.accel_bias_random_walk)) {
        return "imu 的噪声参数必须是非负有限数";
    }
    if (!finitePositive(imu.rate_hz)) return "imu.rate_hz 必须是正的有限数";
    if (!finitePositive(imu.gravity)) return "imu.gravity 必须是正的有限数";
    for (const auto& [sensor, lidar] : calibration.lidars) {
        const std::string where = "lidars." + sensor;
        if (!finiteNonNegative(lidar.min_range_m) || !finitePositive(lidar.max_range_m)) {
            return where + " 的量程必须是有限数";
        }
        if (lidar.min_range_m >= lidar.max_range_m) return where + " 的最小量程不小于最大量程";
        if (lidar.rings == 0) return where + ".rings 不能为零";
        if (!finitePositive(lidar.rate_hz)) return where + ".rate_hz 必须是正的有限数";
    }
    return {};
}

std::vector<SensorName> missingExtrinsics(const CalibrationData& calibration,
                                          std::span<const SensorName> required,
                                          bool allow_assumed) {
    std::vector<SensorName> missing;
    for (const auto& sensor : required) {
        const auto* entry = calibration.extrinsic(sensor);
        const bool ok = entry != nullptr && entry->known()
                        && (allow_assumed || entry->source != CalibrationSource::kAssumed);
        if (!ok) missing.push_back(sensor);
    }
    return missing;
}

}  // namespace simpleslam::fw
