// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file config.hpp
/// 配置聚合体：配置的真源在 C++，不在 YAML。
///
/// 字段只写一次。`visitConfig` 是唯一的字段清单，加载、导出、哈希、schema 导出、
/// 推送到运行期参数存储，全部从它派生。加一个字段只需要改一处，
/// 不会出现"结构体加了、YAML 读取忘了、schema 没更新"这种三处不同步。
///
/// 两个概念不要混：
/// - **配置**是一次运行的完整输入，包含不可在运行期改的东西（线程数、算法选择）；
/// - **参数**是运行期可调的子集，走事务提交与版本（comm/parameters.hpp）。
/// 每个字段自己声明属于哪一类（Tunable），因此"改了为什么没生效"有确定答案。
///
/// **kYes 只给真的在运行期被重新读取的字段**：模块或算法每帧（每次求解）从当时的
/// 参数快照里取值。只在启动时读一次、存进成员的字段标成可调，结果上贴的参数版本号
/// 就是假的——版本变了，用的值没变。

#include <cstdint>
#include <string>

namespace simpleslam::config {

/// 字段是否允许在运行期改。不可调的字段要换会话才生效，这是声明不是惯例。
enum class Tunable : std::uint8_t { kNo, kYes };

/// 数据从哪来。装配期决定，不可运行期改。
struct DatasetSection {
    /// synthetic / kitti_odometry / kitti_raw / replay
    std::string kind = "synthetic";
    std::string path;
    /// 真值取 body 系还是相机系。KITTI odometry 的 poses 是相机系的，
    /// 拿它直接比激光轨迹是常见错误，所以这里默认 body。
    std::string groundtruth_frame = "body";
    std::int64_t skip_scans = 0;
    std::int64_t max_scans = 0;
    /// 数据集里时间异常的帧（扫描起点不早于终点）是否按标称周期修补。默认拒绝读取：
    /// 修补是容错，不是校验。打开后修补政策与受影响的帧进有效身份。
    bool repair_scan_timing = false;
};

/// 标定文件。标定本身不进这张扁平表（calibration_io.hpp 说明了为什么），
/// 这里只留一个指针。留空则用数据源自带的标定。
struct CalibrationSection {
    std::string file;
};

struct RuntimeSection {
    /// 离线：单线程步进 + 虚拟时钟 + 可逐位复现。在线：一个模块一个线程 + 单调时钟。
    bool offline = true;
    std::int64_t spin_budget = 4096;
};

struct PacketizerSection {
    /// 打包策略从哪来。
    ///
    /// "source" 用读取器按实测数据给出的推荐值——支撑区间要按实测的惯性采样间隔定，
    /// 写死一个数只在特定设备上成立。"config" 用下面这些字段。
    /// 默认跟着数据源走，但**用了哪一个必须写进产物**：
    /// 悄悄忽略用户填的字段，比忽略本身更糟。
    std::string source_policy = "source";  // source | config
    bool trigger_on_lidar = true;
    /// 发包是否必须等到覆盖区间的 IMU。纯 LiDAR 里程计置 false。
    bool imu_required = true;
    double post_support_s = 0.0;
    double pre_support_s = 0.0;
    /// 算法要求的相邻 IMU 样本最大间隔（秒）。零表示不查。
    double max_imu_gap_s = 0.5;
    std::int64_t max_imu_backlog = 100000;
    double attachment_tolerance_s = 0.05;
};

struct OdometrySection {
    std::string algorithm = "constant_velocity";
    double keyframe_distance_m = 1.0;
    double keyframe_rotation_deg = 15.0;
    double keyframe_interval_s = 1.0;
    std::int64_t max_iterations = 30;
    double voxel_size_m = 0.5;
};

struct BackendSection {
    std::string place_recognizer = "none";
    std::string graph_optimizer = "none";
    /// 算法没给跨关键帧约束时的兜底权重。如实标注为启发式，不冒充协方差之逆。
    double odometry_translation_weight = 100.0;
    double odometry_rotation_weight = 400.0;
    std::int64_t max_iterations = 20;
    double loop_score_threshold = 0.6;
};

struct EvaluationSection {
    std::string groundtruth;
    std::string output;
    std::string format = "tum";
    std::string alignment = "se3";
    /// nearest：最近邻，只在两边采样同步时成立；interpolate：把真值插值到估计时刻。
    std::string association = "nearest";
    double association_tolerance_s = 0.02;
    /// 插值关联时括点的最大间隔（秒）。超过它的帧不参与比较。
    double max_interpolation_gap_s = 0.15;
    /// 允许拿不完整的运行下结论（前缀分析）。默认不允许：严格模式判为无效。
    bool allow_incomplete = false;
    /// RPE 的间隔。按距离是 KITTI 的口径（100..800 米），按时间是 TUM 的口径。
    /// 两个都给时以时间为准；都为 0 则不算 RPE。
    double rpe_delta_m = 100.0;
    double rpe_delta_s = 0.0;
    std::string baselines;
    std::string report;
};

struct RecordingSection {
    bool enabled = false;
    std::string output;
    bool record_diagnostics = true;
};

struct SlamConfig {
    std::string name = "default";
    /// 随机种子。写进实验回放包，因此"当时那一次"可以重来。
    std::int64_t seed = 0;
    DatasetSection dataset;
    CalibrationSection calibration;
    RuntimeSection runtime;
    PacketizerSection packetizer;
    OdometrySection odometry;
    BackendSection backend;
    EvaluationSection evaluation;
    RecordingSection recording;
};

/// 唯一的字段清单。visit 的签名：visit(路径, 值引用, 说明, 可调性)。
///
/// Config 既可以是 SlamConfig 也可以是 const SlamConfig——值引用随之带上 const。
template <class Config, class Visitor>
void visitConfig(Config& c, Visitor&& visit) {
    visit("name", c.name, "实验名，进报告与回放包", Tunable::kNo);
    visit("seed", c.seed, "随机种子", Tunable::kNo);

    visit("dataset.kind", c.dataset.kind,
          "数据来源：synthetic / kitti_odometry / kitti_raw / replay", Tunable::kNo);
    visit("dataset.path", c.dataset.path, "数据集或录制文件路径", Tunable::kNo);
    visit("dataset.groundtruth_frame", c.dataset.groundtruth_frame,
          "真值坐标系：body / camera", Tunable::kNo);
    visit("dataset.skip_scans", c.dataset.skip_scans, "跳过前 N 帧", Tunable::kNo);
    visit("dataset.max_scans", c.dataset.max_scans, "最多读 N 帧（0 = 全部）", Tunable::kNo);
    visit("dataset.repair_scan_timing", c.dataset.repair_scan_timing,
          "按 100 ms 标称周期修补起点不早于终点的扫描（默认拒绝读取）", Tunable::kNo);

    visit("calibration.file", c.calibration.file,
          "标定文件路径（留空则用数据源自带的标定）", Tunable::kNo);

    visit("runtime.offline", c.runtime.offline, "离线步进（虚拟时钟、可复现）", Tunable::kNo);
    visit("runtime.spin_budget", c.runtime.spin_budget, "每轮步进的任务预算", Tunable::kNo);

    visit("packetizer.source_policy", c.packetizer.source_policy,
          "打包策略取自数据源推荐还是本节配置：source / config", Tunable::kNo);
    visit("packetizer.trigger_on_lidar", c.packetizer.trigger_on_lidar,
          "由激光触发（否则由相机触发）", Tunable::kNo);
    visit("packetizer.imu_required", c.packetizer.imu_required,
          "发包是否必须有 IMU 支撑", Tunable::kNo);
    visit("packetizer.post_support_s", c.packetizer.post_support_s,
          "参考时刻之后额外要等的时间偏置余量（秒）；零表示只等右括点", Tunable::kNo);
    visit("packetizer.pre_support_s", c.packetizer.pre_support_s,
          "参考时刻之前保留的历史（秒）", Tunable::kNo);
    visit("packetizer.max_imu_gap_s", c.packetizer.max_imu_gap_s,
          "相邻 IMU 样本的最大间隔（秒），超过即标缺口；零表示不查", Tunable::kNo);
    visit("packetizer.max_imu_backlog", c.packetizer.max_imu_backlog,
          "IMU 积压上限，超限报缺口", Tunable::kNo);
    visit("packetizer.attachment_tolerance_s", c.packetizer.attachment_tolerance_s,
          "附件的时间容差（秒）", Tunable::kNo);

    visit("odometry.algorithm", c.odometry.algorithm, "里程计算法名", Tunable::kNo);
    visit("odometry.keyframe_distance_m", c.odometry.keyframe_distance_m,
          "关键帧的平移阈值（米）", Tunable::kYes);
    visit("odometry.keyframe_rotation_deg", c.odometry.keyframe_rotation_deg,
          "关键帧的旋转阈值（度）", Tunable::kYes);
    visit("odometry.keyframe_interval_s", c.odometry.keyframe_interval_s,
          "关键帧的时间上限（秒）", Tunable::kNo);
    visit("odometry.max_iterations", c.odometry.max_iterations, "单帧最大迭代次数",
          Tunable::kNo);
    visit("odometry.voxel_size_m", c.odometry.voxel_size_m, "降采样体素边长（米）",
          Tunable::kNo);

    visit("backend.place_recognizer", c.backend.place_recognizer, "位置识别算法名", Tunable::kNo);
    visit("backend.graph_optimizer", c.backend.graph_optimizer, "图优化算法名", Tunable::kNo);
    visit("backend.odometry_translation_weight", c.backend.odometry_translation_weight,
          "里程计边的兜底平移权重（1/m²，启发式）", Tunable::kYes);
    visit("backend.odometry_rotation_weight", c.backend.odometry_rotation_weight,
          "里程计边的兜底旋转权重（1/rad²，启发式）", Tunable::kYes);
    visit("backend.max_iterations", c.backend.max_iterations, "图优化最大迭代次数",
          Tunable::kNo);
    visit("backend.loop_score_threshold", c.backend.loop_score_threshold,
          "回环接受阈值", Tunable::kNo);

    visit("evaluation.groundtruth", c.evaluation.groundtruth, "真值轨迹路径", Tunable::kNo);
    visit("evaluation.output", c.evaluation.output, "估计轨迹输出路径", Tunable::kNo);
    visit("evaluation.format", c.evaluation.format, "轨迹格式：tum / kitti", Tunable::kNo);
    visit("evaluation.alignment", c.evaluation.alignment,
          "对齐方式：none / first_pose / se3 / sim3", Tunable::kNo);
    visit("evaluation.association", c.evaluation.association,
          "关联方式：nearest / interpolate（两边采样不同步时用插值）", Tunable::kNo);
    visit("evaluation.association_tolerance_s", c.evaluation.association_tolerance_s,
          "最近邻关联容差（秒）", Tunable::kNo);
    visit("evaluation.max_interpolation_gap_s", c.evaluation.max_interpolation_gap_s,
          "插值关联的最大括点间隔（秒）", Tunable::kNo);
    visit("evaluation.allow_incomplete", c.evaluation.allow_incomplete,
          "允许拿不完整的运行下结论（前缀分析，只能与前缀基线比较）", Tunable::kNo);
    visit("evaluation.rpe_delta_m", c.evaluation.rpe_delta_m,
          "RPE 的距离间隔（米，KITTI 口径）", Tunable::kNo);
    visit("evaluation.rpe_delta_s", c.evaluation.rpe_delta_s,
          "RPE 的时间间隔（秒，TUM 口径；>0 时优先）", Tunable::kNo);
    visit("evaluation.baselines", c.evaluation.baselines, "精度基线文件路径", Tunable::kNo);
    visit("evaluation.report", c.evaluation.report, "评测报告输出路径", Tunable::kNo);

    visit("recording.enabled", c.recording.enabled, "是否录制原始数据流", Tunable::kNo);
    visit("recording.output", c.recording.output, "录制文件路径", Tunable::kNo);
    visit("recording.record_diagnostics", c.recording.record_diagnostics,
          "是否同时录制诊断流", Tunable::kNo);
}

}  // namespace simpleslam::config
