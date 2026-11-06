// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file trajectory_io.hpp
/// 轨迹读写：TUM 与 KITTI 两种通用格式。
///
/// 选这两种是为了直接接现成工具（evo、KITTI devkit），不是为了自创格式。
///
/// - **TUM**：每行 `timestamp tx ty tz qx qy qz qw`，时间单位秒，四元数 xyzw。
///   自带时间，可直接与真值做时间关联。
/// - **KITTI**：每行 12 个数，行主序的 3×4 位姿（R|t），**不带时间**。
///   读取时必须外部给出时刻来源：同名 `times.txt`，或固定采样周期。
///   把"没有时间"当成"时间等于行号"会让 RPE 的时间口径悄悄失真，所以这里要显式选。

#include <SimpleSLAM/framework/trajectory.hpp>

#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace simpleslam::eval {

namespace fw = simpleslam::fw;

using fw::Trajectory;
using fw::TrajectoryFrame;

enum class TrajectoryFormat : std::uint8_t { kTum, kKitti };

[[nodiscard]] std::string toString(TrajectoryFormat format);
[[nodiscard]] std::optional<TrajectoryFormat> parseTrajectoryFormat(std::string_view text);

class TrajectoryIoError : public std::runtime_error {
public:
    explicit TrajectoryIoError(const std::string& what) : std::runtime_error(what) {}
};

struct TrajectoryIoOptions {
    /// KITTI 的时刻来源：给出 times 文件就用它，否则按固定周期生成。
    std::string kitti_times_path;
    double kitti_period_s = 0.1;
    /// 写出时的小数位数。轨迹是给人看也给工具读的，默认给够。
    int precision = 9;
};

[[nodiscard]] Trajectory readTrajectory(const std::string& path, TrajectoryFormat format,
                                        const TrajectoryIoOptions& options = {});

void writeTrajectory(const std::string& path, const Trajectory& trajectory,
                     TrajectoryFormat format, const TrajectoryIoOptions& options = {});

/// 按扩展名与内容猜格式。猜不出来就报错，不默认成某一种。
[[nodiscard]] TrajectoryFormat detectTrajectoryFormat(const std::string& path);

}  // namespace simpleslam::eval
