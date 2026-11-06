// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file calibration_io.hpp
/// 标定文件的读写与哈希。
///
/// 标定**不进** SlamConfig 的扁平字段表：它是按设备走的，传感器数目不定，
/// 每条外参是 7 个数加一个来源。硬塞进扁平 schema 只会得到
/// `extrinsics.T_body_lidar.qx` 这种字段名，而且换个传感器组合就要改 schema。
/// 所以标定是独立文件、独立版本、独立哈希——这也符合现实：标定随设备走，
/// 配置随实验走，两者的生命周期本来就不同。
///
/// 配置里只留一个 `calibration.file` 指向它。实验回放包同时钉住两个哈希。
///
/// 旋转可以写成四元数、欧拉角或 3×4 矩阵，三选一：手写标定时欧拉角最省事，
/// 从数据集导出时矩阵最直接。写出去时统一成四元数（无歧义、无万向锁）。

#include <SimpleSLAM/framework/calibration.hpp>

#include <stdexcept>
#include <string>

namespace simpleslam::config {

class CalibrationError : public std::runtime_error {
public:
    explicit CalibrationError(const std::string& what) : std::runtime_error(what) {}
};

[[nodiscard]] fw::CalibrationData loadCalibrationString(const std::string& text,
                                                        const std::string& origin = "<string>");
[[nodiscard]] fw::CalibrationData loadCalibrationFile(const std::string& path);

/// 规范化导出：键序固定、旋转统一成四元数。标定哈希基于它。
[[nodiscard]] std::string dumpCalibration(const fw::CalibrationData& calibration);
void writeCalibrationFile(const std::string& path, const fw::CalibrationData& calibration);

/// 标定哈希（FNV-1a 64，十六进制）。进实验回放包与任务记录的身份。
[[nodiscard]] std::string calibrationHash(const fw::CalibrationData& calibration);

}  // namespace simpleslam::config
