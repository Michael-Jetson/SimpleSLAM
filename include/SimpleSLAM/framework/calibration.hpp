// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file calibration.hpp
/// 标定：外参、IMU 噪声模型、激光量程。一份设备档案，一个版本号。
///
/// 为什么是资源而不是配置字段：标定被多个模块读（里程计要外参去畸变、后端要它判断
/// 结果是否兼容、录制要把它写进回放包），而且在线标定会改它——"有状态、多读者、
/// 单写者"正是三分法里资源的定义。
///
/// `revision` 不是装饰：`SolutionIdentity::compatibleWith` 拿它判断异步结果是否
/// 还成立。标定一变，基于旧标定算出的解就失效了，这必须是机器可判的，不是人记的。
///
/// 两条容易错的约定：
/// - **body 系 = IMU 系**（公约）。外参一律写成 `T_body_sensor`，即
///   "传感器系下的点 → body 系"。数据集给的常常是反向或链式的（KITTI 给
///   `T_velo_imu`），换算在读取器里做一次，不散落到算法里。
/// - **协方差有扰动约定**。6×6 没有扰动约定就没有确定含义（不变量 14），
///   所以外参的协方差自带 `PerturbationConvention`。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/events.hpp>
#include <SimpleSLAM/framework/resources.hpp>

#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace simpleslam::fw {

/// 传感器名。用数据集/设备里的原名（"velodyne"、"cam0"、"imu"），不另造编号。
using SensorName = std::string;

/// 外参来源。在线估计出来的外参与标定板量出来的不是一回事，冻结策略也不同。
enum class CalibrationSource : std::uint8_t {
    kUnset,
    kDataset,   ///< 数据集自带（KITTI calib.txt 等）
    kFile,      ///< 标定文件
    kOnline,    ///< 在线估计
    kAssumed,   ///< 假设值（例如单位阵）。必须能被区分出来，否则"没标定"会被当成"标定过"
};

[[nodiscard]] constexpr const char* toString(CalibrationSource source) noexcept {
    switch (source) {
        case CalibrationSource::kUnset: return "unset";
        case CalibrationSource::kDataset: return "dataset";
        case CalibrationSource::kFile: return "file";
        case CalibrationSource::kOnline: return "online";
        case CalibrationSource::kAssumed: return "assumed";
    }
    return "unset";
}

/// 一个传感器相对 body（= IMU）的外参。
///
/// "来源"、"是否固定"、"协方差是否已知"是三件独立的事，各有字段，不从彼此推断：
/// 数据集给的外参可以带协方差也可以不带，在线估计的外参也可以被冻结。
/// 尤其不能拿"协方差数值上接近零"当"没有协方差"——1e-14 是一个很小的已知不确定度，
/// 不是没有。
struct Extrinsic {
    /// 传感器系下的点 → body 系：`p_body = T_body_sensor * p_sensor`。
    SE3d T_body_sensor = SE3d::Identity();
    /// 外参的不确定度，扰动约定见 convention。只有 covariance_known 时才有意义。
    Mat6d covariance = Mat6d::Zero();
    PerturbationConvention convention{};
    CalibrationSource source = CalibrationSource::kUnset;
    /// 协方差是否已知。false 时 covariance 不参与任何计算，导出时也不写。
    bool covariance_known = false;
    /// 估计器是否必须保持它不变。在线标定的外参置 false。
    bool fixed = true;

    [[nodiscard]] bool known() const noexcept { return source != CalibrationSource::kUnset; }
    /// 已知且精确：协方差已知并且恰好为零。数值近零不算。
    [[nodiscard]] bool exact() const noexcept {
        return covariance_known && (covariance.array() == 0.0).all();
    }
};

/// IMU 的噪声与偏置模型。预积分的协方差传播与 ESIKF 的过程噪声都要它。
/// 写死在算法里就换不了设备，所以它属于标定而不是算法常量。
struct ImuIntrinsics {
    double gyro_noise_density = 1.0e-3;      ///< rad/s/√Hz
    double accel_noise_density = 1.0e-2;     ///< m/s²/√Hz
    double gyro_bias_random_walk = 1.0e-5;   ///< rad/s²/√Hz
    double accel_bias_random_walk = 1.0e-4;  ///< m/s³/√Hz
    double rate_hz = 200.0;
    /// 当地重力大小（m/s²）。方向由初始化时的静止加速度定，不在这里写死。
    double gravity = 9.81;
};

/// 激光的量程与组织。这是设备能力，不是算法参数——算法可以在其内再收紧。
struct LidarIntrinsics {
    double min_range_m = 0.5;
    double max_range_m = 120.0;
    std::uint32_t rings = 64;
    double rate_hz = 10.0;
    /// 一次完整扫描的时长。逐点时间与去畸变都要它。
    [[nodiscard]] Duration scanPeriod() const {
        return Duration{static_cast<std::int64_t>(rate_hz > 0.0 ? 1e9 / rate_hz : 0.0)};
    }
};

struct CalibrationData {
    /// 标定版本。一变，基于旧标定的异步结果就身份失效（不变量 15）。
    ///
    /// **由提交入口统一生成**，不由文件声明、也不由持有者自行递增：
    /// 两份不同的标定文件都可以写 `revision: 1`，而资源自己的版本在任何一次写入
    /// 之后都会动。版本与内容必须同出一源，否则"这一帧用的是哪份标定"就说不准。
    /// 装配路径见 `SlamResources::setCalibration()`。
    std::uint64_t revision = 0;
    std::string device;  ///< 设备档案名，进实验回放包
    std::map<SensorName, Extrinsic> extrinsics;
    ImuIntrinsics imu;
    std::map<SensorName, LidarIntrinsics> lidars;

    [[nodiscard]] const Extrinsic* extrinsic(const SensorName& sensor) const {
        auto it = extrinsics.find(sensor);
        return it == extrinsics.end() ? nullptr : &it->second;
    }

    /// `T_body_sensor`。**没登记或来源未定都返回空**。
    ///
    /// 只看键在不在是不够的：一个 `source == kUnset` 的条目里躺着的是默认构造的
    /// 单位阵，把它当成外参返回，就是把"忘了标定"变成了"标定过，恰好是单位阵"。
    /// 这两件事在数值上一模一样，在结论上天差地别。
    ///
    /// 注意它与"body 就是这个传感器"不同：后者是一个**已知**的恒等变换，
    /// 登记时来源写 kDataset/kAssumed，照样返回单位阵。
    [[nodiscard]] std::optional<SE3d> T_body_sensor(const SensorName& sensor) const {
        const auto* entry = extrinsic(sensor);
        if (entry == nullptr || !entry->known()) return std::nullopt;
        return entry->T_body_sensor;
    }

    /// 外参是否已知且来源不是"假设"。需要真实几何的地方（杆臂、去畸变）用它把关。
    [[nodiscard]] bool hasMeasuredExtrinsic(const SensorName& sensor) const {
        const auto* entry = extrinsic(sensor);
        return entry != nullptr && entry->known()
               && entry->source != CalibrationSource::kAssumed;
    }

    /// 两个传感器之间：`T_a_b = T_body_a⁻¹ ∘ T_body_b`。
    [[nodiscard]] std::optional<SE3d> T_sensor_sensor(const SensorName& a,
                                                      const SensorName& b) const {
        const auto ta = T_body_sensor(a);
        const auto tb = T_body_sensor(b);
        if (!ta || !tb) return std::nullopt;
        return ta->inverse().compose(*tb);
    }

    [[nodiscard]] const LidarIntrinsics* lidar(const SensorName& sensor) const {
        auto it = lidars.find(sensor);
        return it == lidars.end() ? nullptr : &it->second;
    }

    /// 登记一条外参。同名覆盖。
    ///
    /// 不在这里动 revision：版本由提交入口生成。就地递增会让"草稿改了几次"
    /// 变成版本号的一部分，而同一次提交无论改了一条还是十条，对外都只是一个新版本。
    void set(const SensorName& sensor, const Extrinsic& extrinsic) {
        extrinsics[sensor] = extrinsic;
    }
};

/// 整份标定的语义校验。返回空串表示合法，否则是第一处问题的说明。
///
/// 文件、数据集读取器与在线提交三个入口共用这一套判据：只在文件入口校验的话，
/// 数据集或在线标定写进来的坏值就没人拦。四元数归一化只能让一个坏矩阵变成
/// "合法"的旋转，不能让它变成对的，所以这里查的是语义，不是能不能构造出来。
[[nodiscard]] std::string validateCalibration(const CalibrationData& calibration);

/// 缺哪些必需的外参。已知（来源不是 unset）才算有；允许假设时 assumed 也算。
[[nodiscard]] std::vector<SensorName> missingExtrinsics(const CalibrationData& calibration,
                                                      std::span<const SensorName> required,
                                                      bool allow_assumed = false);

using Calibration = Resource<CalibrationData>;

}  // namespace simpleslam::fw
