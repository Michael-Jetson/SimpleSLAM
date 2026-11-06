// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file config.hpp
/// 配置聚合体：配置的真源在 C++，不在 YAML。
///
/// 一个任务是一次完整的实验或部署：用一个 SLAM 系统，在线或离线地处理一份数据，
/// 列了评测函数就评测，没列就只跑。任务 YAML 引用一份系统 YAML，系统 YAML 写管线、模块与插件；
/// 加载后两者合成一棵 `TaskConfig`。
///
/// 字段只写一次。`visitConfig` 是唯一的字段清单，加载、导出、schema 导出、
/// 推送到运行期参数存储，全部从它派生。加一个字段只需要改一处。
///
/// 两个概念不要混：
/// - **配置**是一次运行的完整输入，包含不可在运行期改的东西（算法选择、运行方式）；
/// - **参数**是运行期可调的子集，走事务提交与版本（comm/parameters.hpp）。
/// 每个字段自己声明属于哪一类（Tunable），因此"改了为什么没生效"有确定答案。
///
/// **kYes 只给真的在运行期被重新读取的字段**：模块每次用到时从当时的参数快照里取值。
/// 插件节点里的参数第一版一律不可调。

#include <SimpleSLAM/config/plugin.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace simpleslam::config {

/// 字段是否允许在运行期改。不可调的字段要换会话才生效，这是声明不是惯例。
enum class Tunable : std::uint8_t { kNo, kYes };

/// 任务本身。
struct TaskSection {
    std::string name = "default";
    /// offline：单线程步进、虚拟时钟、可逐位复现；online：一个模块一个线程、单调时钟。
    std::string mode = "offline";
};

struct RuntimeSection {
    std::int64_t spin_budget = 4096;
    /// 在线时入口允许多少包在途。队列只为突发准备，冗余给足：200 包是 10 Hz 激光的 20 秒。
    std::int64_t online_max_in_flight = 200;
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

struct OdometryModuleSection {
    /// 里程计方案。没有内置实现：算法由用户写，在入口登记。
    PluginSpec plugin;
};

struct PlaceRecognizerModuleSection {
    PluginSpec plugin{.type = "none", .params = {}, .children = {}};
};

struct GraphOptimizerModuleSection {
    PluginSpec plugin{.type = "none", .params = {}, .children = {}};
    /// 算法没给跨关键帧约束时的兜底权重。如实标注为启发式，不冒充协方差之逆。
    double odometry_translation_weight = 100.0;
    double odometry_rotation_weight = 400.0;
};

struct ModulesSection {
    OdometryModuleSection odometry;
    PlaceRecognizerModuleSection place_recognizer;
    GraphOptimizerModuleSection graph_optimizer;
};

/// 一个 SLAM 系统：工作方式、模块与插件。系统 YAML 的全部内容。
struct SystemConfig {
    /// 管线按工作方式分：odometry 只跑里程计 / mapping 建图 / localization 定位。
    std::string pipeline = "odometry";
    RuntimeSection runtime;
    PacketizerSection packetizer;
    ModulesSection modules;
};

struct DataSection {
    /// 数据集读取器：类型加参数。读取器一次给出来源、自带的标定、真值与推荐的打包策略。
    PluginSpec source;
    /// 外部标定文件。给了就取代数据集自带的标定。
    std::string calibration;
    /// 多序列：每个序列一个子进程，覆盖 `data.source.sequence`；读取器要有这个字段。
    /// 序列名要能当目录名，只用字母、数字与 . _ -。
    std::vector<std::string> sequences;
};

struct EvaluationSection {
    /// 这次跑哪些评测函数：名字 → 插件节点。空表示只跑不评测。名字只用字母、数字与
    /// `_`：它是结果里指标名的前缀，也是逐样本表格的文件名。
    PluginMap evaluators;
    /// 真值轨迹文件。空表示用数据集自带的真值。
    std::string groundtruth;
};

/// 日志的等级。只决定记下哪些、显示哪些，不影响结果。
struct LogOutputs {
    std::string console = "info";  ///< 终端显示的最低等级
    std::string file = "debug";    ///< 任务记录里 run.log 记录的最低等级
    /// 按来源单独设，终端与 run.log 都照它：每条写成 `<来源的键>=<等级>`，键与配置路径
    /// 同一种写法，按段前缀匹配，最长的生效。如 `odometry.plugin.map=debug`。
    std::vector<std::string> levels;
};

struct OutputsSection {
    /// 任务记录目录。空表示 runs/<task.name>。
    std::string dir;
    /// 另写一份 KITTI 格式的轨迹（给 KITTI 官方评测程序用）。TUM 格式的总会写。
    bool kitti_trajectories = false;
    LogOutputs log;
};

/// 一个任务：系统、数据、运行方式、评测、产出。
struct TaskConfig {
    TaskSection task;
    SystemConfig system;
    DataSection data;
    EvaluationSection evaluation;
    OutputsSection outputs;
};

/// 唯一的字段清单。visit 的签名：visit(路径, 值引用, 说明, 可调性)。
///
/// 值的类型有七种：bool、int64、double、string、字符串列表、插件节点（PluginSpec）、
/// 插件节点的映射（PluginMap）。
/// Config 既可以是 TaskConfig 也可以是 const TaskConfig——值引用随之带上 const。
template <class Config, class Visitor>
void visitConfig(Config& c, Visitor&& visit) {
    visit("task.name", c.task.name, "任务名，是任务记录的目录名，也进结果", Tunable::kNo);
    visit("task.mode", c.task.mode, "运行方式：offline 离线 / online 在线", Tunable::kNo);

    auto& s = c.system;
    visit("system.pipeline", s.pipeline, "管线：odometry / mapping / localization", Tunable::kNo);
    visit("system.runtime.spin_budget", s.runtime.spin_budget, "每轮步进的任务预算",
          Tunable::kNo);
    visit("system.runtime.online_max_in_flight", s.runtime.online_max_in_flight,
          "在线时入口允许在途的包数（0 表示不限）", Tunable::kNo);

    visit("system.packetizer.source_policy", s.packetizer.source_policy,
          "打包策略取自数据源推荐还是本节配置：source / config", Tunable::kNo);
    visit("system.packetizer.trigger_on_lidar", s.packetizer.trigger_on_lidar,
          "由激光触发（否则由相机触发）", Tunable::kNo);
    visit("system.packetizer.imu_required", s.packetizer.imu_required, "发包是否必须有 IMU 支撑",
          Tunable::kNo);
    visit("system.packetizer.post_support_s", s.packetizer.post_support_s,
          "参考时刻之后额外要等的时间偏置余量（秒）；零表示只等右括点", Tunable::kNo);
    visit("system.packetizer.pre_support_s", s.packetizer.pre_support_s,
          "参考时刻之前保留的历史（秒）", Tunable::kNo);
    visit("system.packetizer.max_imu_gap_s", s.packetizer.max_imu_gap_s,
          "相邻 IMU 样本的最大间隔（秒），超过即标缺口；零表示不查", Tunable::kNo);
    visit("system.packetizer.max_imu_backlog", s.packetizer.max_imu_backlog,
          "IMU 积压上限，超限报缺口", Tunable::kNo);
    visit("system.packetizer.attachment_tolerance_s", s.packetizer.attachment_tolerance_s,
          "附件的时间容差（秒）", Tunable::kNo);

    visit("system.modules.odometry.plugin", s.modules.odometry.plugin, "里程计方案",
          Tunable::kNo);
    visit("system.modules.place_recognizer.plugin", s.modules.place_recognizer.plugin,
          "位置识别（建图管线用；none 表示不找回环）", Tunable::kNo);
    visit("system.modules.graph_optimizer.plugin", s.modules.graph_optimizer.plugin,
          "图优化（建图管线用）", Tunable::kNo);
    visit("system.modules.graph_optimizer.odometry_translation_weight",
          s.modules.graph_optimizer.odometry_translation_weight,
          "里程计边的兜底平移权重（1/m²，启发式）", Tunable::kYes);
    visit("system.modules.graph_optimizer.odometry_rotation_weight",
          s.modules.graph_optimizer.odometry_rotation_weight,
          "里程计边的兜底旋转权重（1/rad²，启发式）", Tunable::kYes);

    visit("data.source", c.data.source, "数据集读取器：类型与参数", Tunable::kNo);
    visit("data.calibration", c.data.calibration, "外部标定文件（留空则用数据集自带的标定）",
          Tunable::kNo);
    visit("data.sequences", c.data.sequences, "多序列：每个序列一个子进程", Tunable::kNo);

    visit("evaluation.evaluators", c.evaluation.evaluators,
          "这次跑哪些评测函数：名字 → 插件节点；没有就不评测", Tunable::kNo);
    visit("evaluation.groundtruth", c.evaluation.groundtruth, "真值轨迹路径（留空用数据集的）",
          Tunable::kNo);

    visit("outputs.dir", c.outputs.dir, "任务记录目录（留空为 runs/<task.name>）",
          Tunable::kNo);
    visit("outputs.kitti_trajectories", c.outputs.kitti_trajectories,
          "另写一份 KITTI 格式的轨迹（TUM 格式的总会写）", Tunable::kNo);
    visit("outputs.log.console", c.outputs.log.console,
          "终端显示的最低等级：debug / info / warn / error", Tunable::kNo);
    visit("outputs.log.file", c.outputs.log.file, "run.log 记录的最低等级", Tunable::kNo);
    visit("outputs.log.levels", c.outputs.log.levels,
          "按来源设等级：<来源的键>=<等级>，如 odometry.plugin.map=debug", Tunable::kNo);
}

}  // namespace simpleslam::config
