// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 任务执行器：插件节点由目录还原并补齐默认值、参数桥接到模块的私有名、配置命令、
/// 在一份 KITTI 夹具上从任务文件跑到任务记录、度量与事件、单独评测、在线评测、多序列与
/// 按权重的汇总、运行之前拒绝还不支持的组合。

#include "count_evaluator.hpp"
#include "still_odometry.hpp"

#include <SimpleSLAM/app/bindings.hpp>
#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/run.hpp>
#include <SimpleSLAM/app/task.hpp>
#include <SimpleSLAM/comm/parameters.hpp>
#include <SimpleSLAM/config/calibration_io.hpp>
#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/eval/evaluator.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>
#include <SimpleSLAM/plugins/odometry.hpp>
#include <SimpleSLAM/plugins/sources/kitti_dataset.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unistd.h>
#include <utility>
#include <vector>

using namespace simpleslam;

namespace {

namespace fs = std::filesystem;

/// 用户自己写的评测函数：终点漂移，走完时估计与真值差多远，占真值走过路程的百分比。
/// 与内置的同样登记、同样在任务里选、同样进结论。
class FinalDrift final : public eval::EvaluatorBase {
public:
    static constexpr std::string_view kTypeName = "final_drift";
    struct Config {
        std::string estimate = "odometry";

        template <class V>
        void visit(V& v) {
            v("estimate", estimate, "评哪条估计");
        }
    };
    explicit FinalDrift(const Config& config) : config_(config) {}

    [[nodiscard]] eval::Needs needs() const override { return {.groundtruth = true}; }
    [[nodiscard]] eval::Evaluation evaluate(const eval::IEvaluationInputs& inputs) const override {
        const auto pair = inputs.pair(config_.estimate);
        eval::Evaluation out;
        const double length = pair.reference.pathLength();
        if (pair.estimate.empty() || pair.reference.empty() || !(length > 0.0)) return out;
        const auto& estimate = pair.estimate.at(pair.estimate.size() - 1).pose;
        const auto& truth = pair.reference.at(pair.reference.size() - 1).pose;
        out.samples = 1;
        out.add("drift", 100.0 * (estimate.translation() - truth.translation()).norm() / length,
                "%", "drift");
        return out;
    }

private:
    Config config_;
};

/// 给的结果不合格的评测函数：`mistake` 选错在哪一样。框架在写任何文件之前核对。
class Misnamed final : public eval::EvaluatorBase {
public:
    static constexpr std::string_view kTypeName = "misnamed";
    struct Config {
        std::string mistake = "table";

        template <class V>
        void visit(V& v) {
            v("mistake", mistake, "错在哪一样：table / column / scalar / duplicate / row");
        }
    };
    explicit Misnamed(const Config& config) : config_(config) {}

    [[nodiscard]] eval::Needs needs() const override { return {}; }
    [[nodiscard]] eval::Evaluation evaluate(
        const eval::IEvaluationInputs& /*inputs*/) const override {
        const auto& mistake = config_.mistake;
        eval::Evaluation out;
        out.samples = 1;
        out.add(mistake == "scalar" ? "rmse (m)" : "rmse", 1.0, "m");
        if (mistake == "duplicate") out.add("rmse", 2.0, "m");
        const std::string column = mistake == "column" ? "a,b" : "time_ns";
        eval::Table table{.name = mistake == "table" ? "a/b" : "errors",
                          .columns = {{.name = column, .unit = ""}},
                          .rows = {{eval::Cell{std::int64_t{1}}}}};
        if (mistake == "row") table.rows.push_back({});
        out.tables.push_back(std::move(table));
        return out;
    }

private:
    Config config_;
};

/// 没有 sequence 参数的读取器：多序列用不了它。核对在打开数据之前，所以不必真能读。
class SingleFileReader final : public plugin::DatasetReaderBase {
public:
    static constexpr std::string_view kTypeName = "single_file";
    struct Config {
        std::string path;

        template <class V>
        void visit(V& v) {
            v("path", path, "数据文件");
        }
    };
    explicit SingleFileReader(const Config& config) : config_(config) {}
    [[nodiscard]] std::string name() const override { return "SingleFileReader"; }

private:
    [[nodiscard]] plugin::Dataset doOpen(const plugin::DatasetRequest& /*request*/) const override {
        throw std::runtime_error("测试用的读取器不读数据: " + config_.path);
    }
    Config config_;
};

/// 用到参数的里程计：每帧读重力（共享、运行中不改），并把跑过的帧数写进一个运行中可改的参数。
class ParamOdometry final : public plugin::OdometryBase {
public:
    static constexpr std::string_view kTypeName = "param_odometry";
    static constexpr comm::Param<double> kGravity{.name = "gravity",
                                                  .fallback = 9.81,
                                                  .doc = "重力加速度（m/s²）",
                                                  .min = 9.0,
                                                  .max = 10.5};
    static constexpr comm::Param<std::int64_t> kSteps{
        .name = "odometry.steps", .fallback = 0, .doc = "跑过的帧数", .runtime = true};
    static constexpr std::array kParams{comm::reads(kGravity), comm::writes(kSteps)};

    struct Config {
        template <class V>
        void visit(V& /*v*/) {}
    };
    explicit ParamOdometry(const Config& /*config*/) {}

    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }
    [[nodiscard]] std::string name() const override { return "param_odometry"; }

    static inline double seen_gravity = 0.0;

protected:
    plugin::TrackResult track(const fw::SensorPacket& /*packet*/) override {
        seen_gravity = param(kGravity);
        setParam(kSteps, param(kSteps) + 1);
        return {};
    }
};

config::Catalogs catalogs() {
    auto result = app::builtinCatalogs();
    result.of<plugin::DatasetReaderBase>().add<SingleFileReader>();
    result.of<plugin::OdometryBase>().add<test::StillOdometry>();
    result.of<plugin::OdometryBase>().add<ParamOdometry>();
    result.of<eval::EvaluatorBase>().add<FinalDrift>();
    result.of<eval::EvaluatorBase>().add<Misnamed>();
    result.of<eval::EvaluatorBase>().add<test::CountEvaluator>();
    return result;
}

/// 一次性目录，析构即删。
class TempDir {
public:
    explicit TempDir(const char* tag) {
        path_ = fs::temp_directory_path()
                / ("simpleslam_" + std::string(tag) + std::to_string(::getpid()));
        fs::remove_all(path_);
        fs::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ignored;
        fs::remove_all(path_, ignored);
    }
    [[nodiscard]] const fs::path& path() const { return path_; }

private:
    fs::path path_;
};

void writeText(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream(path) << text;
}

std::string readText(const fs::path& path) {
    std::ifstream file(path);
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

/// 一帧 velodyne .bin：四个点，方位角各差 90°。
void writeScan(const fs::path& path) {
    fs::create_directories(path.parent_path());
    const std::array<std::array<float, 4>, 4> points = {{{1.0F, 0.0F, 0.0F, 0.5F},
                                                         {0.0F, 1.0F, 0.0F, 0.6F},
                                                         {-1.0F, 0.0F, 0.0F, 0.7F},
                                                         {0.0F, -1.0F, 0.0F, 0.8F}}};
    std::ofstream file(path, std::ios::binary);
    for (const auto& point : points) {
        file.write(reinterpret_cast<const char*>(point.data()), sizeof(point));
    }
}

/// odometry 布局：<root>/sequences/<序列> 与 <root>/poses/<序列>.txt，相机系真值每帧前进 1 米。
void makeOdometryDataset(const fs::path& root, int frames, const std::string& name = "00") {
    const fs::path sequence = root / "sequences" / name;
    std::ostringstream times;
    std::ostringstream poses;
    for (int i = 0; i < frames; ++i) {
        const std::string index = std::to_string(i);
        writeScan(sequence / "velodyne" / (std::string(6 - index.size(), '0') + index + ".bin"));
        times << std::fixed << (0.1 * i) << '\n';
        poses << "1 0 0 " << i << " 0 1 0 0 0 0 1 0\n";
    }
    writeText(sequence / "times.txt", times.str());
    writeText(root / "poses" / (name + ".txt"), poses.str());
    writeText(sequence / "calib.txt",
              "P0: 1 0 0 0 0 1 0 0 0 0 1 0\nTr: 1 0 0 0 0 1 0 0 0 0 1 0\n");
}

/// 任务文件里的评测一节：对真值算 ATE。
constexpr const char* kEvaluateAte = "evaluation:\n  evaluators:\n    ate: {type: ate}\n";

struct Fixture {
    TempDir dir{"app_task"};
    fs::path task_file;
    fs::path run_dir;

    /// evaluation 是任务文件里评测一节的原文，空串表示只跑不评测。
    explicit Fixture(const std::string& evaluation = kEvaluateAte) {
        makeOdometryDataset(dir.path() / "kitti", 10);
        writeText(dir.path() / "systems" / "still.yaml",
                  "pipeline: mapping\nmodules:\n  odometry:\n    plugin: {type: still}\n");
        run_dir = dir.path() / "runs" / "still";
        task_file = dir.path() / "tasks" / "still.yaml";
        writeText(task_file, "task:\n  name: still\n"
                             "system: ../systems/still.yaml\n"
                             "data:\n  source:\n    type: kitti_odometry\n"
                             "    path: \""
                                 + (dir.path() / "kitti").string()
                                 + "\"\n    sequence: \"00\"\n"
                                   "outputs:\n  dir: \""
                                 + run_dir.string() + "\"\n" + evaluation);
    }

    int run(std::vector<std::string> overrides = {}, bool overwrite = false) {
        out.str("");
        err.str("");
        const app::RunOptions options{.task_file = task_file.string(),
                                      .overrides = std::move(overrides),
                                      .overwrite = overwrite};
        return app::runTask(options, catalogs(), out, err);
    }

    /// 多序列：子进程由登记了同一个测试实现的入口跑。
    int runSequences(std::vector<std::string> overrides, std::size_t jobs = 1,
                     bool overwrite = false) {
        out.str("");
        err.str("");
        const app::RunOptions options{.task_file = task_file.string(),
                                      .overrides = std::move(overrides),
                                      .overwrite = overwrite,
                                      .jobs = jobs,
                                      .executable = SIMPLESLAM_TEST_TASK_RUNNER};
        return app::runTask(options, catalogs(), out, err);
    }

    std::ostringstream out;
    std::ostringstream err;
};

app::ResolvedTask resolve(const std::string& text) {
    return app::resolveTask(config::loadTaskString(text, "<string>"), catalogs());
}

}  // namespace

TEST_CASE("插件节点由目录还原并补齐默认值：写不写默认值，导出相同", "[app][task]") {
    const std::string data = "data:\n  source: {type: kitti_odometry, path: /data/x}\n";
    const auto bare =
        resolve("system:\n  modules:\n    odometry:\n      plugin: {type: still}\n" + data);
    const auto full = resolve(
        "system:\n  modules:\n    odometry:\n      plugin: {type: still, keyframe_every: 5}\n"
        + data);
    CHECK(config::dumpConfig(bare.config) == config::dumpConfig(full.config));

    // 还原后的节点补齐了每个字段
    const auto& source = bare.config.data.source;
    CHECK(std::get<std::string>(source.params.at("groundtruth_frame")) == "body");
    CHECK(std::get<std::int64_t>(source.params.at("max_scans")) == 0);
    CHECK(bare.config.system.modules.place_recognizer.plugin.type == "none");
    CHECK(bare.odometry.type() == "still");
}

TEST_CASE("选错了在运行之前报错，带完整路径与可选的实现", "[app][task]") {
    const std::string data = "data:\n  source: {type: kitti_odometry, path: /data/x}\n";
    const auto message = [&](const std::string& plugin) {
        try {
            const std::string system =
                "system:\n  modules:\n    odometry:\n      plugin: " + plugin + "\n";
            (void)resolve(system + data);
        } catch (const config::ConfigError& e) {
            return std::string(e.what());
        }
        return std::string();
    };
    // 别的接口的实现：说清这里要什么、它是什么
    const auto wrong_interface = message("{type: voxel_point_map}");
    CHECK(wrong_interface.find("system.modules.odometry.plugin.type") != std::string::npos);
    CHECK(wrong_interface.find("OdometryBase") != std::string::npos);
    CHECK(wrong_interface.find("still") != std::string::npos);
    // 名字拼错、参数拼错、参数值不合法
    CHECK(message("{type: stil}").find("still") != std::string::npos);
    CHECK(message("{type: still, keyframe_evry: 3}").find("keyframe_evry") != std::string::npos);
    CHECK(message("{type: still, keyframe_every: 0}").find("keyframe_every") != std::string::npos);
    // 数据节点同样严格
    CHECK_THROWS_AS(resolve("data:\n  source: {type: kitti_odometry, path: \"\"}\n"
                            "system:\n  modules:\n    odometry:\n      plugin: {type: still}\n"),
                    config::ConfigError);
}

TEST_CASE("参数：按声明核对、补齐默认值，覆盖项照常写", "[app][params]") {
    const std::string data = "data:\n  source: {type: kitti_odometry, path: /data/x}\n";
    const std::string mapping =
        "system:\n  pipeline: mapping\n  modules:\n    odometry:\n"
        "      plugin: {type: param_odometry}\n";
    const auto task = resolve(mapping + data);
    // 选中的插件与模块声明的参数都补上了默认值
    const auto& params = task.config.system.params;
    CHECK(std::get<double>(params.at("gravity")) == 9.81);
    CHECK(std::get<std::int64_t>(params.at("odometry.steps")) == 0);
    CHECK(std::get<double>(params.at("graph.odometry_translation_weight")) == 100.0);
    // 插件写的参数，写者是它所属的模块
    const auto steps = std::find_if(task.params.begin(), task.params.end(),
                                    [](const auto& d) { return d.name == "odometry.steps"; });
    REQUIRE(steps != task.params.end());
    CHECK(steps->writers == std::vector<std::string>{"/slam/odometry"});
    // task.yaml 写出完整的 params，原样能重跑
    const std::string dumped = config::dumpConfig(task.config);
    CHECK(dumped.find("  params:\n    \"graph.odometry_rotation_weight\": 400\n")
          != std::string::npos);
    CHECK(config::dumpConfig(resolve(dumped).config) == dumped);

    // 值写在 params 里，覆盖项照常写
    const auto written = resolve(mapping + "  params:\n    gravity: 9.79\n" + data);
    CHECK(std::get<double>(written.config.system.params.at("gravity")) == 9.79);
    const auto overridden = app::resolveTask(
        config::loadTaskString(mapping + data, "<string>", ".", {"system.params.gravity=9.78"}),
        catalogs());
    CHECK(std::get<double>(overridden.config.system.params.at("gravity")) == 9.78);

    // 写错了在运行之前报：没人声明的名字列出声明过的，类型不符，超出范围
    const auto message = [&](const std::string& system) {
        try {
            (void)resolve(system + data);
        } catch (const config::ConfigError& e) {
            return std::string(e.what());
        }
        return std::string();
    };
    const auto unknown = message(mapping + "  params:\n    gravty: 9.8\n");
    CHECK(unknown.find("未知参数 gravty") != std::string::npos);
    CHECK(unknown.find("gravity") != std::string::npos);
    CHECK(message(mapping + "  params:\n    gravity: true\n").find("要 double")
          != std::string::npos);
    CHECK(message(mapping + "  params:\n    gravity: 11\n").find("不能大于") != std::string::npos);
    // 只跑里程计的管线没有图优化模块：它的参数这次没人声明
    const std::string odometry =
        "system:\n  modules:\n    odometry:\n      plugin: {type: param_odometry}\n";
    CHECK(message(odometry + "  params:\n    graph.odometry_translation_weight: 120\n")
              .find("未知参数") != std::string::npos);
}

TEST_CASE("插件在回调里写参数：所属模块回调结束后提交，每次修改记进 run.log", "[app][params]") {
    Fixture fixture;
    writeText(fixture.dir.path() / "systems" / "still.yaml",
              "pipeline: mapping\nmodules:\n  odometry:\n    plugin: {type: param_odometry}\n"
              "params:\n  gravity: 9.79\n");
    REQUIRE(fixture.run() == 0);
    INFO(fixture.out.str() << fixture.err.str());
    CHECK(ParamOdometry::seen_gravity == 9.79);  // 读的是配置给的值
    const std::string log = readText(fixture.run_dir / "run.log");
    // 写者、旧值与新值、版本
    CHECK(log.find("参数 odometry.steps：0 → 1（第 1 版，/slam/odometry）") != std::string::npos);
    CHECK(log.find("参数 odometry.steps：9 → 10（第 10 版，/slam/odometry）") != std::string::npos);
    // task.yaml 只有初值，变化看 run.log
    CHECK(readText(fixture.run_dir / "task.yaml").find("\"odometry.steps\": 0")
          != std::string::npos);
}

TEST_CASE("配置命令：check、dump、schema 都由目录还原插件节点", "[app][task]") {
    Fixture fixture;
    const auto path = fixture.task_file.string();
    std::ostringstream out;
    std::ostringstream err;

    REQUIRE(app::configCommand({"dump", path}, catalogs(), out, err) == 0);
    const std::string dumped = out.str();
    CHECK(dumped.find("type: \"still\"") != std::string::npos);
    CHECK(dumped.find("keyframe_every: 5") != std::string::npos);  // 默认值已补齐

    out.str("");
    REQUIRE(app::configCommand({"schema"}, catalogs(), out, err) == 0);
    CHECK(out.str().find("\"system.modules.odometry.plugin\": \"OdometryBase\"")
          != std::string::npos);
    CHECK(out.str().find("\"type\": \"still\"") != std::string::npos);
    CHECK(out.str().find("\"type\": \"kitti_odometry\"") != std::string::npos);

    CHECK(app::configCommand({"check", path, "system.modules.odometry.plugin.type=nope"},
                             catalogs(), out, err)
          == 1);
    CHECK(app::configCommand({path}, catalogs(), out, err) == -1);  // 不是配置命令
}

TEST_CASE("从任务文件跑到任务记录：结果、出处、结局、结论在一个目录里，可以复现",
          "[app][task]") {
    Fixture fixture;
    REQUIRE(fixture.run() == 0);
    INFO(fixture.out.str() << fixture.err.str());

    for (const char* name :
         {"task.yaml", "outcome.yaml", "calibration.yaml", "result.json",
          "run.log", "trajectory/odometry.tum", "trajectory/online.tum",
          "trajectory/keyframes.tum", "trajectory/groundtruth.tum", "evaluation/ate.errors.csv"}) {
        CHECK(fs::exists(fixture.run_dir / name));
    }
    CHECK(readText(fixture.run_dir / "outcome.yaml").find("complete: true") != std::string::npos);
    // 机器读的结论：操作台读它，不解析 YAML
    const std::string result = readText(fixture.run_dir / "result.json");
    CHECK(result.find("\"schema\": 3,") != std::string::npos);
    CHECK(result.find("\"kind\": \"run\"") != std::string::npos);
    CHECK(result.find("\"complete\": true,\n  \"exit_code\": 0,") != std::string::npos);
    // 每个评测函数一段：测出了东西、读了哪些文件
    CHECK(result.find("\"status\": \"ok\"") != std::string::npos);
    CHECK(result.find("\"inputs\": [\"trajectory/odometry.tum\", \"trajectory/groundtruth.tum\"]")
          != std::string::npos);
    CHECK(result.find("\"translation.rmse\": {\"value\": ") != std::string::npos);
    CHECK(result.find("\"label\": \"ATE_t\"") != std::string::npos);
    CHECK(result.find("\"file\": \"evaluation/ate.errors.csv\"") != std::string::npos);
    CHECK(fixture.out.str().find("结论: ") != std::string::npos);
    // 评测读的轨迹带着语义：坐标系、跟踪的传感器、时钟
    CHECK(readText(fixture.run_dir / "trajectory" / "odometry.tum")
              .starts_with("# simpleslam frame=odom tracked="));
    CHECK(readText(fixture.run_dir / "trajectory" / "groundtruth.tum")
              .starts_with("# simpleslam frame=world tracked="));

    // 任务记录里的任务就是这次运行的唯一输入：重新加载、还原，导出与它逐字节相同
    const auto again = app::loadTask((fixture.run_dir / "task.yaml").string(), {}, catalogs());
    CHECK(config::dumpConfig(again.config) == readText(fixture.run_dir / "task.yaml"));
    // 原始配置原样存一份：任务文件与系统文件同名，系统文件前面加上它的角色
    CHECK(readText(fixture.run_dir / "source" / "still.yaml") == readText(fixture.task_file));
    CHECK(readText(fixture.run_dir / "source" / "system_still.yaml")
          == readText(fixture.dir.path() / "systems" / "still.yaml"));
    CHECK_FALSE(fs::exists(fixture.run_dir / "source" / "overrides.txt"));  // 没有覆盖项
    CHECK(fixture.out.str().starts_with("任务: still（"));
    const std::string outcome = readText(fixture.run_dir / "outcome.yaml");
    // 运行的事实：写它的软件版本、实际生效的输入；目标窗口在里面，事后补评从这里读
    CHECK(outcome.find("software_version:") != std::string::npos);
    CHECK(outcome.find("\"source.evaluation_window_end_ns\": \"900000000\"")
          != std::string::npos);

    // 目录已有内容：拒绝，免得两次任务的结果混在一起；显式覆盖才写
    CHECK(fixture.run() == 1);
    CHECK(fixture.err.str().find("已有内容") != std::string::npos);
    CHECK(fixture.run({}, /*overwrite=*/true) == 0);

    // 覆盖项写进 overrides.txt；操作台渲染之前的源文件（如 .py）用 --source 另存一份
    const fs::path script = fixture.dir.path() / "tasks" / "still.py";
    writeText(script, "config = None  # 渲染之前的原文\n");
    std::ostringstream out;
    std::ostringstream err;
    const app::RunOptions options{.task_file = fixture.task_file.string(),
                                  .overrides = {"outputs.kitti_trajectories=false"},
                                  .overwrite = true,
                                  .source_file = script.string()};
    REQUIRE(app::runTask(options, catalogs(), out, err) == 0);
    CHECK(readText(fixture.run_dir / "source" / "still.py") == readText(script));
    CHECK(readText(fixture.run_dir / "source" / "overrides.txt")
          == "outputs.kitti_trajectories=false\n");
}

TEST_CASE("只跑不评测：结论里只有运行的事实；还不支持的组合在运行之前拒绝", "[app][task]") {
    {
        Fixture fixture("");
        REQUIRE(fixture.run() == 0);
        CHECK_FALSE(fs::exists(fixture.run_dir / "evaluation"));
        const std::string result = readText(fixture.run_dir / "result.json");
        CHECK(result.find("\"exit_code\": 0,") != std::string::npos);
        CHECK(result.find("\"evaluations\": {}") != std::string::npos);
        CHECK(fs::exists(fixture.run_dir / "trajectory" / "odometry.tum"));
        // 数据自带真值就照样写进任务记录：事后补评不必回头找数据
        CHECK(fs::exists(fixture.run_dir / "trajectory" / "groundtruth.tum"));
    }
    {
        Fixture fixture;
        CHECK(fixture.run({"system.pipeline=localization"}) == 1);
        CHECK(fixture.err.str().find("定位管线") != std::string::npos);
        // 都在运行之前拒绝：没有留下半个任务记录
        CHECK_FALSE(fs::exists(fixture.run_dir));
    }
}

TEST_CASE("单独评测：补评与重评只读任务记录，数字与跑完立刻评相同", "[app][task][evaluate]") {
    Fixture fixture("");  // 先只跑不评测
    REQUIRE(fixture.run() == 0);
    const fs::path& dir = fixture.run_dir;
    std::ostringstream out;
    std::ostringstream err;

    const auto refused = [&](const std::string& target, const std::vector<std::string>& overrides,
                             const std::string& reason) {
        err.str("");
        CHECK(app::evaluateTask(target, overrides, catalogs(), out, err) == 1);
        CHECK(err.str().find(reason) != std::string::npos);
    };
    // 任务里没有评测函数，覆盖项也没加：没什么可评
    refused(dir.string(), {}, "没有评测函数");
    // 单独评测只改评测；不是任务记录的目录不评
    refused(dir.string(), {"outputs.kitti_trajectories=true"}, "覆盖项要以 evaluation. 开头");
    refused(fixture.dir.path().string(), {}, "不是任务记录");

    // 补评：用覆盖项加评测函数，评测一节随之写回任务记录里的任务
    const std::string outcome_before = readText(dir / "outcome.yaml");
    REQUIRE(app::evaluateTask(dir.string(), {"evaluation.evaluators.ate.type=ate"}, catalogs(),
                              out, err)
            == 0);
    INFO(out.str() << err.str());
    std::string result = readText(dir / "result.json");
    CHECK(result.find("\"complete\": true,\n  \"exit_code\": 0,") != std::string::npos);
    CHECK(result.find("\"status\": \"ok\"") != std::string::npos);
    const std::string table = readText(dir / "evaluation" / "ate.errors.csv");
    CHECK_FALSE(table.empty());
    // 单独评测不改运行：任务里的评测一节更新，运行的出处一个字不变
    auto again = app::loadTask((dir / "task.yaml").string(), {}, catalogs());
    CHECK(again.evaluators.count("ate") == 1);
    CHECK(readText(dir / "outcome.yaml") == outcome_before);

    // 重评：加一个评测函数，原来的照评；原地不动的估计做不了 sim3，结论里写明实际用的对齐
    REQUIRE(app::evaluateTask(dir.string(),
                              {"evaluation.evaluators.ate_sim3.type=ate",
                               "evaluation.evaluators.ate_sim3.alignment=sim3"},
                              catalogs(), out, err)
            == 0);
    result = readText(dir / "result.json");
    CHECK(result.find("\"ate_sim3\": {") != std::string::npos);
    CHECK(result.find("请求的对齐方式是 sim3，实际用的是 se3") != std::string::npos);
    CHECK(fs::exists(dir / "evaluation" / "ate_sim3.errors.csv"));
    CHECK(readText(dir / "evaluation" / "ate.errors.csv") == table);

    // 重评中途出错（真值文件不见了）：上一次的结论与表格原样留着，任务里的评测一节也不改
    const std::string before = readText(dir / "result.json");
    const fs::path truth = dir / "trajectory" / "groundtruth.tum";
    fs::rename(truth, fixture.dir.path() / "groundtruth.tum");
    refused(dir.string(), {"evaluation.evaluators.rpe.type=rpe"}, "没有真值");
    CHECK(readText(dir / "result.json") == before);
    CHECK(readText(dir / "evaluation" / "ate.errors.csv") == table);
    CHECK(app::loadTask((dir / "task.yaml").string(), {}, catalogs()).evaluators.count("rpe") == 0);
    fs::rename(fixture.dir.path() / "groundtruth.tum", truth);

    // 跑完立刻评：同一份任务记录，同样的数字
    REQUIRE(fixture.run({"evaluation.evaluators.ate.type=ate"}, /*overwrite=*/true) == 0);
    CHECK(readText(dir / "evaluation" / "ate.errors.csv") == table);
}

TEST_CASE("单独评测只读评测要用的几节：系统一节的写法变了也能评，别的节原样保留",
          "[app][task][evaluate]") {
    Fixture fixture("");
    REQUIRE(fixture.run() == 0);
    const fs::path& dir = fixture.run_dir;
    // 模拟系统文件的写法变了：系统一节多一个这个入口不认识的键，顶层另有一节入口不认识
    std::string text = readText(dir / "task.yaml");
    const auto system = text.find("system:\n");
    REQUIRE(system != std::string::npos);
    text.insert(system + 8, "  retired_section:\n    anything: 1\n");
    text += "\nslam:\n  scheme: retired_lio\n";
    {
        std::ofstream file(dir / "task.yaml");
        file << text;
    }
    std::ostringstream out;
    std::ostringstream err;
    REQUIRE(app::evaluateTask(dir.string(), {"evaluation.evaluators.ate.type=ate"}, catalogs(), out,
                              err)
            == 0);
    INFO(out.str() << err.str());
    CHECK(fs::exists(dir / "evaluation" / "ate.errors.csv"));
    // 评测一节换成这次的，前后的节一个字不变
    const std::string after = readText(dir / "task.yaml");
    const auto head = [](const std::string& s) { return s.substr(0, s.find("\nevaluation:\n")); };
    const auto tail = [](const std::string& s) { return s.substr(s.find("\noutputs:\n")); };
    CHECK(head(after) == head(text));
    CHECK(tail(after) == tail(text));
    const std::string section = after.substr(after.find("\nevaluation:\n"));
    CHECK(section.substr(0, section.find("\noutputs:\n")).find("ate:\n      type: \"ate\"")
          != std::string::npos);
}

TEST_CASE("自己写的评测函数与内置的一样登记、一样选、一样进结论", "[app][task][evaluate]") {
    Fixture fixture("evaluation:\n  evaluators:\n    drift: {type: final_drift}\n");
    REQUIRE(fixture.run({"outputs.kitti_trajectories=true"}) == 0);
    INFO(fixture.out.str() << fixture.err.str());
    const std::string result = readText(fixture.run_dir / "result.json");
    CHECK(result.find("\"drift\": {\n      \"type\": \"final_drift\"") != std::string::npos);
    CHECK(result.find("\"drift\": {\"value\": 100, \"unit\": \"%\", \"label\": \"drift\"}")
          != std::string::npos);  // 原地不动，真值走了 9 米
    CHECK(fixture.out.str().find("评测 drift（final_drift）: 1 个样本") != std::string::npos);
    // 要的话另写一份 KITTI 格式的轨迹，给官方评测程序
    CHECK(fs::exists(fixture.run_dir / "trajectory" / "odometry.txt"));

    // 参数同样按字段清单严格还原，写错了在运行之前报
    CHECK(fixture.run({"evaluation.evaluators.drift.estimat=online"}, /*overwrite=*/true) == 1);
    CHECK(fixture.err.str().find("estimat") != std::string::npos);
}

TEST_CASE("评测函数给的名字进文件名与表头：写错了在写任何文件之前报", "[app][task][evaluate]") {
    Fixture fixture;  // 跑完立刻评 ate
    REQUIRE(fixture.run() == 0);
    const fs::path& dir = fixture.run_dir;
    const std::string before = readText(dir / "result.json");
    const std::vector<std::pair<std::string, std::string>> mistakes = {
        {"table", "表名 a/b"}, {"column", "表 errors 的列名 a,b"}, {"scalar", "标量名 rmse (m)"},
        {"duplicate", "标量名 rmse 重复"}, {"row", "表 errors 有一行的格数是 0，列数是 1"}};
    for (const auto& [mistake, reason] : mistakes) {
        std::ostringstream out;
        std::ostringstream err;
        CHECK(app::evaluateTask(dir.string(),
                                {"evaluation.evaluators.bad.type=misnamed",
                                 "evaluation.evaluators.bad.mistake=" + mistake},
                                catalogs(), out, err)
              == 1);
        INFO(mistake << ": " << err.str());
        CHECK(err.str().find("评测函数 bad 给的结果不合格：" + reason) != std::string::npos);
    }
    // 上一次的结论、表格与任务原样留着
    CHECK(readText(dir / "result.json") == before);
    CHECK(fs::exists(dir / "evaluation" / "ate.errors.csv"));
    CHECK(app::loadTask((dir / "task.yaml").string(), {}, catalogs()).evaluators.count("bad") == 0);
}

TEST_CASE("数据没有真值而所选的评测函数要真值：在运行之前拒绝", "[app][task][evaluate]") {
    Fixture fixture;
    makeOdometryDataset(fixture.dir.path() / "kitti", 4, "11");
    fs::remove(fixture.dir.path() / "kitti" / "poses" / "11.txt");  // KITTI 11 号以后没有真值
    CHECK(fixture.run({"data.source.sequence=\"11\""}) == 1);
    CHECK(fixture.err.str().find("所选的评测函数要真值") != std::string::npos);
    CHECK_FALSE(fs::exists(fixture.run_dir));  // 不留下半个任务记录
}

TEST_CASE("单独评测外部轨迹：写成一份最小的任务记录，只评测、不判通过与否", "[app][evaluate]") {
    Fixture fixture;
    REQUIRE(fixture.run() == 0);
    const fs::path& run = fixture.run_dir;
    app::TrajectoryEvaluation request;
    request.estimate = (run / "trajectory" / "odometry.tum").string();
    request.groundtruth = (run / "trajectory" / "groundtruth.tum").string();
    request.out = (fixture.dir.path() / "external").string();
    request.evaluators = {{"ate", config::PluginSpec{.type = "ate", .params = {}, .children = {}}}};
    std::ostringstream out;
    std::ostringstream err;

    // 只有两条轨迹，说不出产生它的运行是否完整：照样给出误差，完整性记为 null
    REQUIRE(app::evaluateTrajectories(request, catalogs(), out, err) == 0);
    INFO(out.str() << err.str());
    const fs::path external = request.out;
    CHECK(fs::exists(external / "trajectory" / "odometry.tum"));
    CHECK(fs::exists(external / "trajectory" / "groundtruth.tum"));
    const std::string result = readText(external / "result.json");
    CHECK(result.find("\"complete\": null,\n  \"exit_code\": 0,") != std::string::npos);
    // 没有任务就不编：名字取估计轨迹的文件名
    CHECK(result.find("\"task\": \"odometry\"") != std::string::npos);
    CHECK(result.find("\"translation.rmse\": {\"value\": ") != std::string::npos);
    // 数字与任务记录里跑完立刻评的相同
    CHECK(readText(external / "evaluation" / "ate.errors.csv")
          == readText(run / "evaluation" / "ate.errors.csv"));

    // KITTI 格式不带时刻：没给时刻文件就报错，给了就按它评
    const fs::path kitti = fixture.dir.path() / "kitti";
    fs::create_directories(kitti);
    const auto odometry = eval::readTrajectory((run / "trajectory" / "odometry.tum").string(),
                                               eval::TrajectoryFormat::kTum);
    eval::writeTrajectory((kitti / "odometry.txt").string(), odometry,
                          eval::TrajectoryFormat::kKitti);
    {
        std::ofstream times(kitti / "times.txt");
        times << std::setprecision(17);
        for (std::size_t i = 0; i < odometry.size(); ++i) {
            times << static_cast<double>(odometry.at(i).time.count()) * 1e-9 << '\n';
        }
    }
    request.estimate = (kitti / "odometry.txt").string();
    request.out = (fixture.dir.path() / "kitti_without_times").string();
    CHECK(app::evaluateTrajectories(request, catalogs(), out, err) == 1);
    CHECK(err.str().find("times.txt") != std::string::npos);
    CHECK_FALSE(fs::exists(request.out));
    request.times = (kitti / "times.txt").string();
    request.out = (fixture.dir.path() / "kitti_with_times").string();
    CHECK(app::evaluateTrajectories(request, catalogs(), out, err) == 0);

    // 外部真值要自带时刻：KITTI 格式的真值在任务里报错
    CHECK(fixture.run({"evaluation.groundtruth=" + (kitti / "odometry.txt").string()}, true) == 1);
    CHECK(fixture.err.str().find("TUM") != std::string::npos);

    // 读不了的轨迹：报错，不留下半份任务记录
    request.estimate = (fixture.dir.path() / "missing.tum").string();
    request.out = (fixture.dir.path() / "rejected").string();
    CHECK(app::evaluateTrajectories(request, catalogs(), out, err) == 1);
    CHECK_FALSE(fs::exists(request.out));
}

TEST_CASE("多序列：每个序列一个子进程、一份子任务记录，最后汇总成一张表",
          "[app][task][sequences]") {
    Fixture fixture;  // 评测 ATE；夹具里已有序列 00
    makeOdometryDataset(fixture.dir.path() / "kitti", 6, "01");
    const std::vector<std::string> sequences = {"data.source.sequence=\"\"",
                                                "data.sequences=[\"00\", \"01\"]"};
    REQUIRE(fixture.runSequences(sequences, /*jobs=*/2) == 0);
    INFO(fixture.out.str() << fixture.err.str());

    for (const std::string name : {"00", "01"}) {
        const fs::path child = fixture.run_dir / name;
        CHECK(fs::exists(child / "evaluation" / "ate.errors.csv"));
        CHECK(fs::exists(child / "trajectory" / "odometry.tum"));
        CHECK(fs::exists(fixture.run_dir / "logs" / (name + ".log")));
        // 子任务记录是一份独立的单序列任务，原样就能重跑
        const auto task = app::loadTask((child / "task.yaml").string(), {}, catalogs());
        CHECK(task.config.task.name == "still-" + name);
        CHECK(task.config.data.sequences.empty());
        CHECK(std::get<std::string>(task.config.data.source.params.at("sequence")) == name);
        CHECK(config::dumpConfig(task.config) == readText(child / "task.yaml"));
        CHECK(fs::exists(child / "source" / "still.yaml"));
    }
    const std::string result = readText(fixture.run_dir / "result.json");
    CHECK(result.find("\"kind\": \"sequences\"") != std::string::npos);
    CHECK(result.find("\"exit_code\": 0,") != std::string::npos);
    CHECK(result.find("{\"name\": \"01\"") != std::string::npos);
    CHECK(result.find("\"scalars\": {\"ate.translation.rmse\": {\"value\": ")
          != std::string::npos);
    // 每个序列都有结果，才有平均
    CHECK(result.find("\"ate.translation.rmse\"", result.find("\"mean\"")) != std::string::npos);
    CHECK(readText(fixture.run_dir / "task.yaml").find("sequences: [\"00\", \"01\"]")
          != std::string::npos);
    CHECK(fs::exists(fixture.run_dir / "source" / "still.yaml"));  // 顶层也存原始配置
    CHECK(fixture.out.str().find("平均") != std::string::npos);

    // 任务记录已有内容：照样拒绝；显式覆盖时一个一个跑，结果与同时跑相同
    const std::string parallel = readText(fixture.run_dir / "01" / "trajectory" / "odometry.tum");
    CHECK(fixture.runSequences(sequences) == 1);
    CHECK(fixture.err.str().find("已有内容") != std::string::npos);
    REQUIRE(fixture.runSequences(sequences, 1, /*overwrite=*/true) == 0);
    CHECK(readText(fixture.run_dir / "01" / "trajectory" / "odometry.tum") == parallel);

    // 单独评测多序列的任务记录：每个子任务记录各自重评，汇总随之重写
    std::ostringstream out;
    std::ostringstream err;
    REQUIRE(app::evaluateTask(fixture.run_dir.string(),
                              {"evaluation.evaluators.ate_sim3.type=ate",
                               "evaluation.evaluators.ate_sim3.alignment=sim3"},
                              catalogs(), out, err)
            == 0);
    INFO(out.str() << err.str());
    CHECK(fs::exists(fixture.run_dir / "01" / "evaluation" / "ate_sim3.errors.csv"));
    const std::string evaluated = readText(fixture.run_dir / "result.json");
    CHECK(evaluated.find("\"ate_sim3.translation.rmse\"", evaluated.find("\"mean\""))
          != std::string::npos);
    // 两个评测函数用了同一个短名：表头带上评测函数的名字，免得两列看着一样
    CHECK(out.str().find("ate_sim3:ATE_t") != std::string::npos);
    CHECK(app::loadTask((fixture.run_dir / "task.yaml").string(), {}, catalogs())
              .evaluators.count("ate_sim3")
          == 1);
}

TEST_CASE("多序列：一个序列出错不影响别的序列，退出码取最严重的", "[app][task][sequences]") {
    Fixture fixture;  // 夹具里没有序列 07
    REQUIRE(fixture.runSequences({"data.source.sequence=\"\"", "data.sequences=[\"00\", \"07\"]"},
                                 /*jobs=*/2)
            == 1);
    CHECK(fs::exists(fixture.run_dir / "00" / "result.json"));  // 00 照常跑完
    CHECK_FALSE(fs::exists(fixture.run_dir / "07"));  // 07 在建任务记录之前就出错了
    CHECK(readText(fixture.run_dir / "logs" / "07.log").find("错误") != std::string::npos);
    const std::string result = readText(fixture.run_dir / "result.json");
    CHECK(result.find("\"exit_code\": 1,\n  \"sequences\"") != std::string::npos);
    CHECK(result.find("\"exit_code\": 1, \"complete\": null") != std::string::npos);
    CHECK(result.find("\"problem\": \"目录不存在") != std::string::npos);  // 原因取自日志
    CHECK(result.find("\"mean\": {}") != std::string::npos);  // 有序列没出结果，不算平均
}

TEST_CASE("多序列：读取器不支持、与 data.source.sequence 同时写，都在运行之前拒绝",
          "[app][task][sequences]") {
    Fixture fixture;  // 任务文件里写了 data.source.sequence: "00"
    CHECK(fixture.runSequences({"data.sequences=[\"00\"]"}) == 1);
    CHECK(fixture.err.str().find("只能写一个") != std::string::npos);
    CHECK(fixture.runSequences({"data.sequences=[\"00\"]",
                                "data.source={type: single_file, path: missing.dat}"})
          == 1);
    CHECK(fixture.err.str().find("不支持多序列") != std::string::npos);
    CHECK_FALSE(fs::exists(fixture.run_dir));

    // --jobs 要是正整数
    std::vector<std::string> words = {"simpleslam", "--jobs=0", fixture.task_file.string()};
    std::vector<char*> argv;
    for (auto& word : words) argv.push_back(word.data());
    CHECK(app::run(static_cast<int>(argv.size()), argv.data(), catalogs()) == 2);
}

TEST_CASE("日志：任务记录里一份全量的 run.log，终端照旧，按来源设的等级管得住插件",
          "[app][task][log]") {
    Fixture fixture;
    REQUIRE(fixture.run() == 0);
    INFO(fixture.out.str() << fixture.err.str());
    const std::string log = readText(fixture.run_dir / "run.log");
    // 执行器的输出都在里面；终端上只印正文
    CHECK(log.find("I 0.000 - task  任务: ") != std::string::npos);
    CHECK(fixture.out.str().starts_with("任务: "));
    CHECK(log.find("task  任务记录: ") != std::string::npos);
    // 插件的调试日志只进文件，带帧序号、完整来源与代码位置
    CHECK(log.find("#1 /slam/odometry › still (still_odometry.hpp:") != std::string::npos);
    CHECK(log.find("原地不动，第 0 帧") != std::string::npos);
    CHECK(fixture.out.str().find("原地不动") == std::string::npos);
    CHECK(readText(fixture.run_dir / "outcome.yaml").find("log_dropped: 0") != std::string::npos);

    // 按来源调等级：里程计只记 warn 以上
    REQUIRE(fixture.run({"outputs.log.levels=[odometry=warn]"}, /*overwrite=*/true) == 0);
    CHECK(readText(fixture.run_dir / "run.log").find("原地不动") == std::string::npos);
    // 写错了在运行之前报错
    CHECK(fixture.run({"outputs.log.levels=[odometry]"}, /*overwrite=*/true) == 1);
    CHECK(fixture.err.str().find("outputs.log.levels") != std::string::npos);
}

TEST_CASE("在线：按数据时间的节奏回放，跑完写任务记录与队列统计", "[app][task][online]") {
    Fixture fixture;  // 10 帧，数据跨 0.9 秒
    const auto start = std::chrono::steady_clock::now();
    REQUIRE(fixture.run({"task.mode=online"}) == 0);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    INFO(fixture.out.str() << fixture.err.str());

    CHECK(elapsed >= std::chrono::milliseconds(900));  // 不提前放行
    CHECK(fixture.out.str().find("在线，按数据时间的节奏回放") != std::string::npos);
    CHECK(readText(fixture.run_dir / "queues.txt").find("/slam/sensor_packet -> /slam/odometry")
          != std::string::npos);
    CHECK(readText(fixture.run_dir / "outcome.yaml").find("complete: true") != std::string::npos);
    CHECK(fs::exists(fixture.run_dir / "evaluation" / "ate.errors.csv"));

    // 离线不写队列统计：没有并发，统计没有意义
    REQUIRE(fixture.run({"task.mode=offline"}, /*overwrite=*/true) == 0);
    CHECK_FALSE(fs::exists(fixture.run_dir / "queues.txt"));
}

namespace {

/// metrics.csv 里来源与键都对得上的那些行。
std::vector<std::string> metricRows(const fs::path& run_dir, const std::string& source,
                                    const std::string& key) {
    std::vector<std::string> rows;
    std::istringstream text(readText(run_dir / "metrics.csv"));
    for (std::string line; std::getline(text, line);) {
        if (line.find("," + source + "," + key + ",") != std::string::npos) rows.push_back(line);
    }
    return rows;
}

}  // namespace

TEST_CASE("度量与事件进任务记录：评测函数读它们，旧的任务记录没有就明确报错",
          "[app][task][evaluate]") {
    Fixture fixture;
    REQUIRE(fixture.run() == 0);
    INFO(fixture.out.str() << fixture.err.str());
    CHECK(readText(fixture.run_dir / "metrics.csv")
              .starts_with("frame,data_time_ns,source,key,value,unit\n"));
    CHECK(fs::exists(fixture.run_dir / "events.jsonl"));
    // 里程计插件的固定骨架每帧记一条耗时：帧序号是包序号，时刻是那一包的数据时刻
    const auto durations = metricRows(fixture.run_dir, "odometry.plugin", "track.duration");
    REQUIRE(durations.size() == 10);
    CHECK(durations.front().starts_with("1,0,odometry.plugin,track.duration,"));
    CHECK(durations.front().ends_with(",ms"));

    // 没有 metrics.csv 的任务记录（复制一份、去掉它）：ate 照样能评，timing 明确报错
    const fs::path trimmed = fixture.dir.path() / "no_metrics_run";
    fs::copy(fixture.run_dir, trimmed, fs::copy_options::recursive);
    fs::remove(trimmed / "metrics.csv");
    std::ostringstream out;
    std::ostringstream err;
    CHECK(app::evaluateTask(trimmed.string(), {}, catalogs(), out, err) == 0);
    CHECK(app::evaluateTask(trimmed.string(), {"evaluation.evaluators.t.type=timing"}, catalogs(),
                            out, err)
          == 1);
    CHECK(err.str().find("没有 metrics.csv") != std::string::npos);

    // 单独评测换成 timing：只读度量，不要真值
    out.str("");
    err.str("");
    REQUIRE(app::evaluateTask(fixture.run_dir.string(), {"evaluation.evaluators.t.type=timing"},
                              catalogs(), out, err)
            == 0);
    INFO(out.str() << err.str());
    const std::string result = readText(fixture.run_dir / "result.json");
    CHECK(result.find("\"duration.p95\": {\"value\": ") != std::string::npos);
    CHECK(result.find("\"inputs\": [\"metrics.csv\"]") != std::string::npos);
    CHECK(fs::exists(fixture.run_dir / "evaluation" / "t.samples.csv"));
    CHECK(out.str().find("T_mean") != std::string::npos);
}

TEST_CASE("在线评测：跑着按数据时间每隔一段评一次，记成度量、终端一行，不进结论",
          "[app][task][online-evaluation]") {
    Fixture fixture;
    const std::vector<std::string> online = {"evaluation.online.evaluators=[\"ate\"]",
                                             "evaluation.online.interval_s=0.3"};
    REQUIRE(fixture.run(online) == 0);
    INFO(fixture.out.str() << fixture.err.str());
    CHECK(fixture.out.str().find("在线评测: ate，每 0.3 秒（数据时间）") != std::string::npos);
    CHECK(fixture.out.str().find("在线评测 ate（ate）: ") != std::string::npos);
    // 结果记成度量：来源是在线评测模块，键是 <评测函数的名字>.<标量名>
    const auto rows = metricRows(fixture.run_dir, "evaluation", "ate.translation.rmse");
    REQUIRE_FALSE(rows.empty());
    CHECK(rows.front().ends_with(",m"));
    // 结论只认跑完后对任务记录的评测
    const std::string result = readText(fixture.run_dir / "result.json");
    CHECK(result.find("\"ate\": {") != std::string::npos);
    CHECK(result.find("evaluation.") == std::string::npos);

    // 离线按数据时间触发：再跑一遍，在线评测的度量逐字节相同
    REQUIRE(fixture.run(online, /*overwrite=*/true) == 0);
    CHECK(metricRows(fixture.run_dir, "evaluation", "ate.translation.rmse") == rows);

    // 在线评的必须是上面列出的评测函数；间隔必须为正。都在运行之前报错
    CHECK(fixture.run({"evaluation.online.evaluators=[\"rpe\"]"}, true) == 1);
    CHECK(fixture.err.str().find("不在 evaluation.evaluators 里") != std::string::npos);
    CHECK(fixture.run({"evaluation.online.evaluators=[\"ate\"]",
                       "evaluation.online.interval_s=0"},
                      true)
          == 1);
    CHECK(fixture.err.str().find("evaluation.online.interval_s 必须为正") != std::string::npos);
}

TEST_CASE("多序列：给了权重的量按权重平均，平均的权重是权重之和", "[app][task][sequences]") {
    Fixture fixture("evaluation:\n  evaluators:\n    count: {type: count}\n");
    makeOdometryDataset(fixture.dir.path() / "kitti", 6, "01");  // 序列 00 有 10 帧
    REQUIRE(fixture.runSequences({"data.source.sequence=\"\"", "data.sequences=[\"00\", \"01\"]"})
            == 0);
    INFO(fixture.out.str() << fixture.err.str());
    const std::string result = readText(fixture.run_dir / "result.json");
    const auto mean = result.find("\"mean\"");
    REQUIRE(mean != std::string::npos);
    // (10·10 + 6·6) / (10 + 6) = 8.5；按序列平均会是 8
    CHECK(result.find("\"count.frames\": {\"value\": 8.5, \"unit\": \"\", \"label\": \"N\", "
                      "\"weight\": 16}",
                      mean)
          != std::string::npos);
}
