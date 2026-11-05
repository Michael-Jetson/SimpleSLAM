// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 任务执行器：插件节点由目录还原并补齐默认值、参数桥接到模块的私有名、配置命令、
/// 在一份 KITTI 夹具上从任务文件跑到任务档案、运行之前拒绝还不支持的组合。

#include <SimpleSLAM/app/bindings.hpp>
#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/run.hpp>
#include <SimpleSLAM/app/task.hpp>
#include <SimpleSLAM/comm/parameters.hpp>
#include <SimpleSLAM/config/calibration_io.hpp>
#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/plugins/odometry.hpp>
#include <SimpleSLAM/plugins/sources/kitti_dataset.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unistd.h>
#include <vector>

using namespace simpleslam;

namespace {

namespace fs = std::filesystem;

/// 测试用的里程计：原地不动，每 keyframe_every 帧给一个关键帧建议。
class StillOdometry final : public plugin::OdometryBase {
public:
    static constexpr std::string_view kTypeName = "still";

    struct Config {
        std::int64_t keyframe_every = 5;

        template <class V>
        void visit(V& v) {
            v("keyframe_every", keyframe_every, "每几帧一个关键帧");
        }
    };

    explicit StillOdometry(const Config& config) : config_(config) {
        if (config_.keyframe_every < 1) throw std::invalid_argument("keyframe_every 至少为 1");
    }

    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }
    [[nodiscard]] std::string name() const override { return "still"; }

protected:
    plugin::TrackResult track(const fw::SensorPacket& /*packet*/) override {
        plugin::TrackResult result;
        result.suggest_keyframe = frames_++ % config_.keyframe_every == 0;
        return result;
    }

private:
    Config config_;
    std::int64_t frames_ = 0;
};

config::Catalogs catalogs() {
    auto result = app::builtinCatalogs();
    result.of<plugin::OdometryBase>().add<StillOdometry>();
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

/// odometry 布局：<root>/sequences/00 与 <root>/poses/00.txt，相机系真值每帧前进 1 米。
void makeOdometryDataset(const fs::path& root, int frames) {
    const fs::path sequence = root / "sequences" / "00";
    std::ostringstream times;
    std::ostringstream poses;
    for (int i = 0; i < frames; ++i) {
        const std::string index = std::to_string(i);
        writeScan(sequence / "velodyne" / (std::string(6 - index.size(), '0') + index + ".bin"));
        times << std::fixed << (0.1 * i) << '\n';
        poses << "1 0 0 " << i << " 0 1 0 0 0 0 1 0\n";
    }
    writeText(sequence / "times.txt", times.str());
    writeText(root / "poses" / "00.txt", poses.str());
    writeText(sequence / "calib.txt",
              "P0: 1 0 0 0 0 1 0 0 0 0 1 0\nTr: 1 0 0 0 0 1 0 0 0 0 1 0\n");
}

struct Fixture {
    TempDir dir{"app_task"};
    fs::path task_file;
    fs::path archive;

    explicit Fixture(const std::string& task_section = "purpose: experiment",
                     const std::string& system_extra = "") {
        makeOdometryDataset(dir.path() / "kitti", 10);
        writeText(dir.path() / "systems" / "still.yaml",
                  "pipeline: mapping\nmodules:\n  odometry:\n    plugin: {type: still}\n"
                      + system_extra);
        archive = dir.path() / "runs" / "still";
        task_file = dir.path() / "tasks" / "still.yaml";
        writeText(task_file, "task:\n  name: still\n  " + task_section
                                 + "\nsystem: ../systems/still.yaml\n"
                                   "data:\n  source:\n    type: kitti_odometry\n"
                                   "    path: \""
                                 + (dir.path() / "kitti").string()
                                 + "\"\n    sequence: \"00\"\n"
                                   "outputs:\n  dir: \""
                                 + archive.string() + "\"\n");
    }

    int run(std::vector<std::string> overrides = {}, bool overwrite = false) {
        out.str("");
        err.str("");
        const app::RunOptions options{.task_file = task_file.string(),
                                      .overrides = std::move(overrides),
                                      .overwrite = overwrite};
        return app::runTask(options, catalogs(), out, err);
    }

    std::ostringstream out;
    std::ostringstream err;
};

app::ResolvedTask resolve(const std::string& text) {
    return app::resolveTask(config::loadTaskString(text, "<string>"), catalogs());
}

}  // namespace

TEST_CASE("插件节点由目录还原并补齐默认值：写不写默认值，导出与哈希相同", "[app][task]") {
    const std::string data = "data:\n  source: {type: kitti_odometry, path: /data/x}\n";
    const auto bare =
        resolve("system:\n  modules:\n    odometry:\n      plugin: {type: still}\n" + data);
    const auto full = resolve(
        "system:\n  modules:\n    odometry:\n      plugin: {type: still, keyframe_every: 5}\n"
        + data);
    CHECK(config::dumpConfig(bare.config) == config::dumpConfig(full.config));
    CHECK(config::configHash(bare.config) == config::configHash(full.config));

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

TEST_CASE("可调字段进参数存储，名字是模块的私有名；不可调的不进", "[app][params]") {
    CHECK(app::configParamName("system.modules.graph_optimizer.odometry_translation_weight")
          == "/slam/graph_optimizer/odometry_translation_weight");
    CHECK(app::configParamName("evaluation.rpe_delta_m") == "/config/evaluation/rpe_delta_m");

    auto config = config::loadTaskString(
                      "system:\n  modules:\n    graph_optimizer:\n"
                      "      odometry_translation_weight: 2.5\n",
                      "<string>")
                      .config;
    comm::ParameterStore store;
    app::applyToParameters(config, store);
    const std::string name =
        app::configParamName("system.modules.graph_optimizer.odometry_translation_weight");
    CHECK(store.snapshot()->get<double>(name) == 2.5);

    // 模块随后按代码默认值声明：配置给的值优先
    store.declare<double>(name, 1.0);
    CHECK(store.snapshot()->get<double>(name) == 2.5);
    // 不可调字段不进参数存储：改了也不会生效，不如根本不提供这个口子
    CHECK_FALSE(store.snapshot()->has(app::configParamName("system.runtime.spin_budget")));

    // 运行期改过的值读得回配置
    REQUIRE(store.commit({{name, 3.5}}).ok());
    app::readFromParameters(config, *store.snapshot());
    CHECK(config.system.modules.graph_optimizer.odometry_translation_weight == 3.5);
}

TEST_CASE("配置命令：check、dump、hash、schema 都由目录还原插件节点", "[app][task]") {
    Fixture fixture;
    const auto path = fixture.task_file.string();
    std::ostringstream out;
    std::ostringstream err;

    REQUIRE(app::configCommand({"dump", path}, catalogs(), out, err) == 0);
    const std::string dumped = out.str();
    CHECK(dumped.find("type: \"still\"") != std::string::npos);
    CHECK(dumped.find("keyframe_every: 5") != std::string::npos);  // 默认值已补齐

    out.str("");
    REQUIRE(app::configCommand({"hash", path}, catalogs(), out, err) == 0);
    const auto task = app::loadTask(path, {}, catalogs());
    CHECK(out.str() == config::configHash(task.config) + "\n");

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

TEST_CASE("从任务文件跑到任务档案：结果、出处、结局、报告在一个目录里，可以复现",
          "[app][task]") {
    Fixture fixture;
    REQUIRE(fixture.run() == 0);
    INFO(fixture.out.str() << fixture.err.str());

    for (const char* name : {"task.yaml", "identity.yaml", "outcome.yaml", "report.yaml",
                             "trajectory/odometry.tum", "trajectory/online.tum",
                             "trajectory/keyframes.tum"}) {
        CHECK(fs::exists(fixture.archive / name));
    }
    CHECK(readText(fixture.archive / "outcome.yaml").find("complete: true") != std::string::npos);

    // 档案里的任务就是这次运行的唯一输入：重新加载、还原，得到同一个哈希
    const auto again = app::loadTask((fixture.archive / "task.yaml").string(), {}, catalogs());
    const std::string identity = readText(fixture.archive / "identity.yaml");
    CHECK(identity.find("task_hash: \"" + config::configHash(again.config) + "\"")
          != std::string::npos);
    CHECK(identity.find("software_version:") != std::string::npos);
    CHECK(identity.find("still.yaml") != std::string::npos);  // 出处记的是系统文件

    // 目录已有内容：拒绝，免得两次任务的结果混在一起；显式覆盖才写
    CHECK(fixture.run() == 1);
    CHECK(fixture.err.str().find("已有内容") != std::string::npos);
    CHECK(fixture.run({}, /*overwrite=*/true) == 0);
}

TEST_CASE("录制进档案；回放这份录制，跑出同一条轨迹", "[app][task][replay]") {
    Fixture fixture("purpose: run");
    REQUIRE(fixture.run({"outputs.record=true"}) == 0);
    const fs::path recording = fixture.archive / "recording.ssrec";
    REQUIRE(fs::exists(recording));

    // 录制里只有传感器数据：标定按外部标定文件给，打包策略按系统配置
    const fs::path calibration = fixture.dir.path() / "calibration.yaml";
    plugin::kitti::OdometryReader::Config kitti;
    kitti.path = (fixture.dir.path() / "kitti").string();
    kitti.sequence = "00";
    config::writeCalibrationFile(calibration.string(),
                                 plugin::kitti::OdometryReader(kitti).open({}).calibration);

    const fs::path replayed = fixture.dir.path() / "runs" / "replayed";
    const std::vector<std::string> replay = {
        "data.source={type: replay, path: \"" + recording.string() + "\"}",
        "data.calibration=" + calibration.string(), "system.packetizer.imu_required=false",
        "outputs.dir=" + replayed.string()};
    REQUIRE(fixture.run(replay) == 0);
    INFO(fixture.out.str() << fixture.err.str());
    CHECK(fixture.out.str().find("ReplaySource(") != std::string::npos);
    CHECK(fixture.out.str().find("帧数: 未知") != std::string::npos);  // 回放不先读一遍
    CHECK(readText(replayed / "trajectory" / "odometry.tum")
          == readText(fixture.archive / "trajectory" / "odometry.tum"));

    // 录制里没有真值：拿它做实验，在运行之前就拒绝，不留下档案
    auto experiment = replay;
    experiment.back() = "outputs.dir=" + (fixture.dir.path() / "runs" / "experiment").string();
    experiment.emplace_back("task.purpose=experiment");
    CHECK(fixture.run(experiment) == 1);
    CHECK(fixture.err.str().find("实验要有真值") != std::string::npos);
    CHECK_FALSE(fs::exists(fixture.dir.path() / "runs" / "experiment"));
}

TEST_CASE("只出结果的任务不评测；还不支持的组合在运行之前拒绝", "[app][task]") {
    {
        Fixture fixture("purpose: run");
        REQUIRE(fixture.run() == 0);
        CHECK_FALSE(fs::exists(fixture.archive / "report.yaml"));
        CHECK(fs::exists(fixture.archive / "trajectory" / "odometry.tum"));
    }
    {
        Fixture fixture;
        CHECK(fixture.run({"system.pipeline=localization"}) == 1);
        CHECK(fixture.err.str().find("定位管线") != std::string::npos);
        CHECK(fixture.run({"task.mode=online"}) == 1);
        CHECK(fixture.err.str().find("在线运行") != std::string::npos);
        CHECK(fixture.run({"data.sequences=[\"00\"]"}) == 1);
        CHECK(fixture.err.str().find("多段序列") != std::string::npos);
        // 都在运行之前拒绝：没有留下半个档案
        CHECK_FALSE(fs::exists(fixture.archive));
    }
}
