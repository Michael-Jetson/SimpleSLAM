// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file kitti.hpp
/// KITTI 数据集读取器：odometry 与 raw 两种布局。
///
/// 它们的目录结构、时间戳来源、有无 IMU 都不同，所以是两个类型，
/// 不是一个类加一个布尔开关——"这个序列到底有没有 IMU"应当在类型上就能看出来。
///
/// ## 四个让人栽跟头的地方
///
/// **1. 运动补偿状态**。odometry 基准发布的扫描**已经做过运动补偿**，raw 的没有。
/// 补偿状态因此是读取器声明的事实（`SourceDescriptor::motion_compensation`），
/// 不能从"`.bin` 里有没有时间列"反推——两个序列的 `.bin` 长得一模一样。
/// 对补偿过的扫描再去一次畸变，等于把同一段自车运动扣了两遍。
/// 所以 odometry 默认 `derive_point_times = false`：没有逐点时间，
/// 也就没有人会顺手再补一次。要按方位角反推（例如自己研究补偿效果）得显式打开，
/// 此时逐点时间标为 `kReconstructed`：相位、切帧位置、旋向都是假设，不是测量。
///
/// **2. 参考时刻的物理含义**。odometry 的 `times.txt` 只给一个时刻，真值按帧号对齐。
/// 拿它当本帧的参考时刻可以，`groundTruth()` 与发出的包也因此逐帧对齐；但这只说明
/// **内部标签一致**，不能证明这个时刻就是扫描终点。所以标为 `kDatasetProvided`，
/// 不硬说成 `kScanEnd`。raw 读的是 `timestamps_end`，那才是 `kScanEnd`。
///
/// **3. 真值的坐标系**。odometry 的 `poses/XX.txt` 是**左相机**在首帧相机系下的位姿，
/// 不是激光的。要得到激光系轨迹得做 `Tr⁻¹ · P_i · Tr`。默认就给 body 系
/// （`GroundTruthFrame::kBody`）；缺 `Tr` 时**直接拒绝**，不把未经转换的相机位姿
/// 贴上 body 的标签。
///
/// **4. OXTS 不等于完整惯性流**。硬件 OXTS 是 100 Hz，但 `_sync` 目录按约 10 Hz 的
/// 同步帧各取一条最近记录。读 `_sync/oxts` 得到的是导航状态，不是 100 Hz 惯性流。
/// 所以频率是**实测**的（`SourceDescriptor::imu_rate_hz` 与 `max_imu_gap`），
/// 不抄标称值。推荐的打包策略只等终点之后的第一条样本（右括点），不把实测间隔
/// 当成等待余量——那样低频导航流下每个触发帧都要等过下一帧。连续性要求按实测
/// 间隔放宽一半；做预积分的 LIO 应在配置里按自己的需要收紧。
///
/// **5. 窗口政策**。选中的扫描之外，只多留惯性流的左右括点（第一帧起点之前最后一条、
/// 最后一帧终点之后第一条），不按固定时长外扩——相位一变，固定时长就可能恰好把括点
/// 裁掉。真值只覆盖选中的扫描窗口，世界原点重置到第一帧选中扫描；两个读取器同一个政策。
///
/// ## body 系是谁
///
/// 公约里 body = IMU。odometry 序列没有 IMU，此时 body 取激光自身，外参是一个
/// **已知的恒等变换**（来源 kDataset）——这与"未知外参被猜成单位阵"不是一回事，
/// 后者一律标 kUnset 并让取用处失败。
/// raw 序列有 IMU（oxts），body 就是 IMU，激光外参来自 `calib_imu_to_velo.txt`。
/// 该文件缺失时读取不失败，这条外参保持"未知"：外部标定文件可以补上它。补没补上
/// 由装配核对（SourceDescriptor::required_extrinsics），没补上就失败，除非显式允许假设。

#include <SimpleSLAM/framework/trajectory.hpp>
#include <SimpleSLAM/framework/calibration.hpp>
#include <SimpleSLAM/framework/sensor_source.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace simpleslam::plugin::kitti {

namespace fw = simpleslam::fw;

class KittiError : public std::runtime_error {
public:
    explicit KittiError(const std::string& what) : std::runtime_error(what) {}
};

/// 真值要哪个系。默认 body——算法输出的是 body 轨迹，拿相机系真值去比是常见错误。
enum class GroundTruthFrame : std::uint8_t { kBody, kCamera };

/// 两种布局共用的选项。
struct CommonOptions {
    std::string lidar_name = "velodyne";
    /// 由方位角反推逐点时间。
    ///
    /// 反推出来的是 `kReconstructed`，不是测量：它假定匀速旋转、假定切帧方位、
    /// 假定旋向。对**已经补偿过**的扫描反推更是南辕北辙——补偿后的坐标已不对应
    /// 采集时的方位角。所以两种布局的默认值不同，见各自的选项。
    bool derive_point_times = true;
    /// 跳过前 N 帧 / 最多读 N 帧（0 = 不限）。调试时不必读完整个序列。
    std::size_t skip_scans = 0;
    std::size_t max_scans = 0;
};

struct OdometryOptions : CommonOptions {
    OdometryOptions() {
        // 基准序列的扫描已由数据集补偿过：不反推逐点时间，也就不会有人再补一次
        derive_point_times = false;
    }

    /// 一次扫描的时长。HDL-64E 是 10Hz。逐点时间与覆盖区间都按它算。
    fw::Duration scan_period = std::chrono::milliseconds(100);
    /// 真值路径。留空则按 `<sequence>/../../poses/<序列号>.txt` 找。
    std::string poses_path;
    GroundTruthFrame ground_truth_frame = GroundTruthFrame::kBody;
    /// 扫描的补偿状态。基准序列是补偿过的；如果你拿到的是自行导出的原始扫描，
    /// 改成 kRaw 并打开 derive_point_times。
    fw::MotionCompensation motion_compensation = fw::MotionCompensation::kCompensated;
};

struct RawOptions : CommonOptions {
    /// oxts 除了 IMU 还有 GNSS。发不发由使用者定：没人消费就是白占带宽。
    bool emit_gnss = false;
    /// 允许在缺少 `calib_imu_to_velo.txt` 且没有外部标定时把激光外参假设为单位阵。
    ///
    /// 默认不允许：body 是 IMU，激光在别处，猜一个单位阵等于凭空抹掉杆臂。
    /// 确实要在没有外参的情况下先跑通流程时显式打开，产物里会留下 kAssumed。
    bool allow_assumed_extrinsics = false;
    /// 算法要求的时间偏置余量（与打包策略的 post_support 一致）。惯性流的右括点
    /// 取"最后一帧终点加上它之后的第一条样本"，保证末帧也等得到。
    fw::Duration post_support{0};
    /// 修补"起点不早于终点"的扫描时间：按 100 ms 标称周期从终点往回推起点。
    ///
    /// 默认不修补而是拒绝读取：修补是容错，不是校验，修补过的帧的逐点时间与覆盖区间
    /// 都是推出来的。确实要跑这样的数据时显式打开；修补政策与受影响的帧写进来源描述，
    /// 进有效身份，报告里看得见。
    bool repair_scan_timing = false;
};

/// KITTI odometry：`<dataset>/sequences/XX/`，含 `velodyne/`、`times.txt`、`calib.txt`。
/// 没有 IMU，因此打包策略是"不要求 IMU"。
class OdometrySource final : public fw::ISensorSource {
public:
    explicit OdometrySource(const std::string& sequence_path, OdometryOptions options = {});
    ~OdometrySource() override;

    OdometrySource(OdometrySource&&) noexcept;
    OdometrySource& operator=(OdometrySource&&) noexcept;

    [[nodiscard]] std::optional<fw::SensorDatum> next() override;
    [[nodiscard]] std::string name() const override;
    [[nodiscard]] fw::SourceDescriptor descriptor() const override;

    [[nodiscard]] std::size_t frameCount() const;
    /// 数据集自带的标定。body 就是激光，因此它的外参是已知的恒等变换；
    /// 相机外参来自 calib.txt。
    [[nodiscard]] const fw::CalibrationData& calibration() const;
    [[nodiscard]] bool hasGroundTruth() const;
    /// 真值轨迹，时刻与发出的包的参考时刻逐帧一致。只覆盖选中的扫描窗口，
    /// 世界原点在第一帧选中扫描。
    [[nodiscard]] const fw::Trajectory& groundTruth() const;
    /// 与本数据源匹配的打包策略。不必自己拼。
    [[nodiscard]] fw::PacketizerPolicy packetizerPolicy() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// KITTI raw（synced+rectified）：`<date>/<date>_drive_XXXX_sync/`，
/// 含 `velodyne_points/`（data + timestamps_start/timestamps/timestamps_end）与 `oxts/`。
/// 标定文件在日期目录下（`calib_imu_to_velo.txt`、`calib_velo_to_cam.txt`）。
/// 有 IMU，因此打包策略是"要求 IMU"。
class RawSource final : public fw::ISensorSource {
public:
    explicit RawSource(const std::string& drive_path, RawOptions options = {});
    ~RawSource() override;

    RawSource(RawSource&&) noexcept;
    RawSource& operator=(RawSource&&) noexcept;

    [[nodiscard]] std::optional<fw::SensorDatum> next() override;
    [[nodiscard]] std::string name() const override;
    [[nodiscard]] fw::SourceDescriptor descriptor() const override;

    [[nodiscard]] std::size_t frameCount() const;
    [[nodiscard]] std::size_t imuSampleCount() const;
    /// oxts 的**实测**采样率与最大相邻间隔。`_sync` 目录下通常是约 10 Hz 的
    /// 导航状态，不是 100 Hz 惯性流，所以这两个数要看实测值。
    [[nodiscard]] double imuRateHz() const;
    [[nodiscard]] fw::Duration maxImuGap() const;
    /// body = IMU；激光外参来自 calib_imu_to_velo.txt。
    [[nodiscard]] const fw::CalibrationData& calibration() const;
    [[nodiscard]] bool hasGroundTruth() const;
    /// oxts 解出的 body 轨迹，只覆盖选中的扫描窗口（含左右括点）。
    ///
    /// 世界系是**第一帧选中扫描时刻的 body**（`T_origin⁻¹ · T_i`），不是 ENU：
    /// 原点处姿态里的 roll/pitch/yaw 都被一起消掉了，剩下的只是相对原点的位姿。
    /// 样本在 OXTS 原始时刻上，与扫描终点不同步，评测应使用插值关联。
    [[nodiscard]] const fw::Trajectory& groundTruth() const;
    [[nodiscard]] fw::PacketizerPolicy packetizerPolicy() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace simpleslam::plugin::kitti
