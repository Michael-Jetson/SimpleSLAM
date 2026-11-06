// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 任务配置：未知键与类型不符一律报错、缺省即默认、哈希只随内容变、系统文件按任务文件
/// 所在目录解析、插件节点读成"类型 + 参数"、改 type 回到新实现的默认参数、
/// 系统哈希只看 system 子树、schema 从同一份字段清单导出。

#include <SimpleSLAM/config/loader.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

using namespace simpleslam;
using namespace simpleslam::config;
using Catch::Approx;

namespace {

TaskConfig load(const std::string& text, const std::vector<std::string>& overrides = {}) {
    return loadTaskString(text, "<string>", ".", overrides).config;
}

/// 一次性目录，析构即删。
class TempDir {
public:
    explicit TempDir(const char* tag) {
        path_ = std::filesystem::temp_directory_path()
                / ("simpleslam_" + std::string(tag) + std::to_string(::getpid()));
        std::filesystem::remove_all(path_);
        std::filesystem::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }
    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

void writeText(const std::filesystem::path& path, const std::string& text) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path) << text;
}

}  // namespace

TEST_CASE("缺省即默认值", "[config]") {
    const auto config = load("system:\n  runtime:\n    spin_budget: 7\n");
    CHECK(config.system.runtime.spin_budget == 7);
    CHECK(config.system.runtime.online_max_in_flight == 200);  // 没写的保持默认
    CHECK(config.system.pipeline == "odometry");
    CHECK(config.task.name == "default");
    CHECK(config.task.purpose == "experiment");
    CHECK(config.system.modules.place_recognizer.plugin.type == "none");
}

TEST_CASE("未知键是错误而不是忽略", "[config]") {
    // 少一个字母的键静默跑完，比报错难查一百倍
    CHECK_THROWS_AS(load("systm:\n  pipeline: mapping\n"), ConfigError);
    CHECK_THROWS_AS(load("system:\n  runtim:\n    spin_budget: 7\n"), ConfigError);
    CHECK_THROWS_AS(load("task:\n  nmae: x\n"), ConfigError);
}

TEST_CASE("类型不符是错误，不做隐式转换", "[config]") {
    CHECK_THROWS_AS(load("system:\n  runtime:\n    spin_budget: abc\n"), ConfigError);
    CHECK_THROWS_AS(load("system:\n  runtime:\n    spin_budget: 1.5\n"), ConfigError);
    CHECK_THROWS_AS(load("outputs:\n  record: 1\n"), ConfigError);
    CHECK_THROWS_AS(load("system:\n  runtime:\n    spin_budget: \"4\"\n"), ConfigError);
    CHECK_THROWS_AS(load("data:\n  sequences: \"00\"\n"), ConfigError);  // 要列表
    CHECK_THROWS_AS(load("data:\n  source: kitti_odometry\n"), ConfigError);  // 插件节点是映射
}

TEST_CASE("配置哈希只随内容变，不随注释与键序变", "[config]") {
    const auto a = load("# 注释\nsystem:\n  runtime:\n    spin_budget: 9\ntask:\n  name: run\n");
    const auto b = load("task:\n  name: run\nsystem:\n  runtime:\n    spin_budget: 9\n");
    CHECK(configHash(a) == configHash(b));

    const auto c = load("task:\n  name: run\nsystem:\n  runtime:\n    spin_budget: 10\n");
    CHECK(configHash(a) != configHash(c));
}

TEST_CASE("规范化导出可以再加载回来，插件节点与列表也一样", "[config]") {
    auto config = load(R"(
task:
  name: demo
system:
  modules:
    odometry:
      plugin:
        type: my_lo
        max_range_m: 100
        tolerance: 0.25
        verbose: true
        tag: "00"
        map: {type: voxel_point_map, voxel_size_m: 1.0}
data:
  sequences: ["00", "01"]
)");
    config.evaluation.rpe_delta_m = 50.0;
    const auto again = load(dumpConfig(config));
    CHECK(again.task.name == "demo");
    const auto& plugin = again.system.modules.odometry.plugin;
    CHECK(plugin.type == "my_lo");
    CHECK(std::get<std::int64_t>(plugin.params.at("max_range_m")) == 100);
    CHECK(std::get<double>(plugin.params.at("tolerance")) == Approx(0.25));
    CHECK(std::get<bool>(plugin.params.at("verbose")));
    CHECK(std::get<std::string>(plugin.params.at("tag")) == "00");  // 带引号的是字符串
    CHECK(plugin.children.at("map").type == "voxel_point_map");
    CHECK(again.data.sequences == std::vector<std::string>{"00", "01"});
    CHECK(again.evaluation.rpe_delta_m == Approx(50.0));
    CHECK(configHash(again) == configHash(config));
}

TEST_CASE("规范化导出对刁钻字符串也无损", "[config]") {
    // `name: "a # b"` 不加引号导出，再加载只剩 `a`。
    for (const std::string nasty : {"a # b", "x: y", "\"quoted\"", "tail  ", "back\\slash",
                                    "line\nbreak", "true", "42", ""}) {
        TaskConfig config;
        config.task.name = nasty.empty() ? "x" : nasty;  // 任务名不能为空
        config.outputs.dir = nasty;
        config.data.sequences = {"00", "true", "1.5", "-"};  // 不加引号就会读成别的类型
        config.system.modules.odometry.plugin.params["note"] = nasty;
        const auto again = load(dumpConfig(config));
        CHECK(again.task.name == config.task.name);
        CHECK(again.outputs.dir == nasty);
        CHECK(again.data.sequences == config.data.sequences);
        CHECK(std::get<std::string>(again.system.modules.odometry.plugin.params.at("note"))
              == nasty);
        CHECK(configHash(again) == configHash(config));
    }
}

TEST_CASE("未知的空映射也是未知键", "[config]") {
    // 空映射没有叶子，按叶子遍历的未知键检查看不见它
    CHECK_THROWS_AS(load("misspelled: {}\n"), ConfigError);
    CHECK_THROWS_AS(load("system:\n  nope: {}\n"), ConfigError);
}

TEST_CASE("类型对了不等于值有意义", "[config]") {
    CHECK_THROWS_AS(load("system:\n  runtime:\n    spin_budget: 0\n"), ConfigError);
    CHECK_THROWS_AS(load("system:\n  packetizer:\n    post_support_s: -0.1\n"), ConfigError);
    CHECK_THROWS_AS(load("system:\n  packetizer:\n    post_support_s: nan\n"), ConfigError);
    CHECK_THROWS_AS(load("system:\n  packetizer:\n    post_support_s: 1e400\n"), ConfigError);
    CHECK_THROWS_AS(load("system:\n  pipeline: slam\n"), ConfigError);
    CHECK_THROWS_AS(load("task:\n  purpose: benchmark\n"), ConfigError);
    CHECK_THROWS_AS(load("task:\n  mode: realtime\n"), ConfigError);
    CHECK_THROWS_AS(load("task:\n  name: \"\"\n"), ConfigError);
    CHECK_THROWS_AS(load("evaluation:\n  alignment: umeyama\n"), ConfigError);
    CHECK_THROWS_AS(
        load("system:\n  modules:\n    graph_optimizer:\n      odometry_rotation_weight: 0\n"),
        ConfigError);
    CHECK_THROWS_AS(load("data:\n  sequences: [\"00\", \"00\"]\n"), ConfigError);
    // 序列名是子档案的目录名：越出档案目录、要转义的都不收
    for (const char* name : {"a/b", "..", ".", "a b", "\"q\"", ""}) {
        CHECK_THROWS_AS(load(std::string("data:\n  sequences: [\"") + name + "\"]\n"),
                        ConfigError);
    }
    CHECK(load("data:\n  sequences: [\"2011_09_26_drive_0027_sync\", \"MH-01.v2\"]\n")
              .data.sequences.size() == 2);
    // 覆盖项走同一套校验
    TaskConfig config;
    CHECK_THROWS_AS(applyOverrides(config, {"system.runtime.spin_budget=0"}), ConfigError);
}

TEST_CASE("日志的等级：写法不对在运行之前报错，不进任务哈希", "[config][log]") {
    CHECK_THROWS_AS(load("outputs:\n  log:\n    console: verbose\n"), ConfigError);
    CHECK_THROWS_AS(load("outputs:\n  log:\n    file: INFO\n"), ConfigError);
    for (const char* rule : {"odometry", "odometry=loud", "=debug", "odometry..map=debug",
                             "odometry.map.=debug", "odo metry=debug"}) {
        CHECK_THROWS_AS(load(std::string("outputs:\n  log:\n    levels: [\"") + rule + "\"]\n"),
                        ConfigError);
    }
    CHECK_THROWS_AS(
        load("outputs:\n  log:\n    levels: [\"odometry=debug\", \"odometry=warn\"]\n"),
        ConfigError);
    const auto config =
        load("outputs:\n  log:\n    console: warn\n    levels: [\"odometry.plugin.map=debug\"]\n");
    CHECK(config.outputs.log.console == "warn");
    CHECK(config.outputs.log.levels == std::vector<std::string>{"odometry.plugin.map=debug"});
    CHECK(configHash(config) == configHash(load("")));  // 日志不影响结果
}

TEST_CASE("覆盖项按 schema 的类型解析", "[config]") {
    TaskConfig config;
    applyOverrides(config, {"system.runtime.spin_budget=40", "task.mode=online",
                            "outputs.dir=/tmp/run", "data.sequences=[\"00\", \"05\"]"});
    CHECK(config.system.runtime.spin_budget == 40);
    CHECK(config.task.mode == "online");
    CHECK(config.outputs.dir == "/tmp/run");
    CHECK(config.data.sequences == std::vector<std::string>{"00", "05"});

    CHECK_THROWS_AS(applyOverrides(config, {"system.runtime.nope=1"}), ConfigError);
    CHECK_THROWS_AS(applyOverrides(config, {"system.runtime.spin_budget=abc"}), ConfigError);
    CHECK_THROWS_AS(applyOverrides(config, {"没有等号"}), ConfigError);
}

TEST_CASE("插件节点的覆盖：改参数、改子节点、换实现回到新实现的默认参数", "[config]") {
    TaskConfig config;
    applyOverrides(config, {"system.modules.odometry.plugin={type: my_lo, a: 1}",
                            "system.modules.odometry.plugin.b=2.5",
                            "system.modules.odometry.plugin.map.type=voxel_point_map",
                            "system.modules.odometry.plugin.map.voxel_size_m=0.8"});
    auto& plugin = config.system.modules.odometry.plugin;
    CHECK(plugin.type == "my_lo");
    CHECK(std::get<std::int64_t>(plugin.params.at("a")) == 1);
    CHECK(std::get<double>(plugin.params.at("b")) == Approx(2.5));
    CHECK(plugin.children.at("map").type == "voxel_point_map");
    CHECK(std::get<double>(plugin.children.at("map").params.at("voxel_size_m")) == Approx(0.8));

    // 换实现：原来的参数作废，A/B 对比不会混进上一个实现的参数
    applyOverrides(config, {"system.modules.odometry.plugin.type=other_lo"});
    CHECK(plugin.type == "other_lo");
    CHECK(plugin.params.empty());
    CHECK(plugin.children.empty());
    // 同名不算换：参数保留
    applyOverrides(config, {"system.modules.odometry.plugin.c=3",
                            "system.modules.odometry.plugin.type=other_lo"});
    CHECK(plugin.params.count("c") == 1);
    // 插件节点本身只接受映射
    CHECK_THROWS_AS(applyOverrides(config, {"system.modules.odometry.plugin=lo"}), ConfigError);
}

TEST_CASE("覆盖全部到位之后才整体校验：文件里写错、命令行改对照样成立", "[config]") {
    const auto loaded =
        loadTaskString("task:\n  mode: bogus\n", "<string>", ".", {"task.mode=offline"});
    CHECK(loaded.config.task.mode == "offline");
    CHECK_THROWS_AS(load("task:\n  mode: bogus\n"), ConfigError);
}

TEST_CASE("系统文件按任务文件所在目录解析，挂到 system 下；文件里的覆盖项先于命令行",
          "[config]") {
    const TempDir dir("task_config");
    writeText(dir.path() / "systems" / "lo.yaml",
              "pipeline: mapping\nmodules:\n  odometry:\n    plugin: {type: my_lo, a: 1}\n");
    writeText(dir.path() / "tasks" / "run.yaml",
              "task:\n  name: run\nsystem: ../systems/lo.yaml\n"
              "overrides:\n  - system.modules.odometry.plugin.a=2\n  - task.name=from_file\n");

    const auto loaded = loadTaskFile((dir.path() / "tasks" / "run.yaml").string(),
                                     {"task.name=from_cli"});
    CHECK(loaded.config.system.pipeline == "mapping");
    CHECK(std::get<std::int64_t>(loaded.config.system.modules.odometry.plugin.params.at("a")) == 2);
    CHECK(loaded.config.task.name == "from_cli");  // 命令行排在文件之后
    CHECK(loaded.source.system_file
          == (dir.path() / "systems" / "lo.yaml").lexically_normal().string());
    REQUIRE(loaded.source.overrides.size() == 3);
    CHECK(loaded.source.overrides.back() == "task.name=from_cli");

    // 导出不含出处与覆盖项：导出的是覆盖之后的结果，再加载不会再覆盖一遍
    const auto dumped = dumpConfig(loaded.config);
    CHECK(dumped.find("overrides") == std::string::npos);
    CHECK(load(dumped).task.name == "from_cli");

    // 系统文件里拼错的键，报错指向系统文件
    writeText(dir.path() / "systems" / "lo.yaml", "pipelin: mapping\n");
    try {
        (void)loadTaskFile((dir.path() / "tasks" / "run.yaml").string());
        FAIL("应当报未知键");
    } catch (const ConfigError& e) {
        CHECK(std::string(e.what()).find("lo.yaml") != std::string::npos);
        CHECK(std::string(e.what()).find("system.pipelin") != std::string::npos);
    }
}

TEST_CASE("系统哈希只看 system 子树：换数据、评测、产出不变，换算法参数就变", "[config]") {
    const auto base = load("system:\n  modules:\n    odometry:\n      plugin: {type: lo, a: 1}\n");
    auto other_data = base;
    applyOverrides(other_data, {"data.source={type: kitti_odometry, path: /x}",
                                "evaluation.alignment=sim3", "outputs.dir=/tmp/y",
                                "task.name=another"});
    CHECK(systemHash(other_data) == systemHash(base));
    CHECK(configHash(other_data) != configHash(base));

    // 产出放在哪里不影响结果：只改档案目录，任务哈希不变
    auto other_outputs = base;
    applyOverrides(other_outputs, {"outputs.dir=/tmp/elsewhere", "outputs.record=true"});
    CHECK(configHash(other_outputs) == configHash(base));
    CHECK(dumpConfig(other_outputs) != dumpConfig(base));  // 导出照样完整

    auto other_system = base;
    applyOverrides(other_system, {"system.modules.odometry.plugin.a=2"});
    CHECK(systemHash(other_system) != systemHash(base));
}

TEST_CASE("schema 从同一份字段清单导出", "[config]") {
    const auto fields = schema();
    CHECK(fields.size() > 25);

    bool saw_tunable = false;
    bool saw_slot = false;
    bool saw_list = false;
    for (const auto& field : fields) {
        CHECK_FALSE(field.path.empty());
        CHECK_FALSE(field.doc.empty());  // 每个字段都要有说明，否则 Python 侧只能猜
        if (field.path == "system.modules.graph_optimizer.odometry_translation_weight") {
            CHECK(field.type == "double");
            CHECK(field.tunable);  // 模块每次加边时从当时的参数快照里读它
            saw_tunable = true;
        }
        if (field.path == "system.runtime.spin_budget") {
            CHECK(field.type == "int");
            CHECK(field.default_value == "4096");
            CHECK_FALSE(field.tunable);  // 步进预算改了要重启，不是"改了没生效"
        }
        if (field.path == "system.modules.odometry.plugin") {
            CHECK(field.type == "slot");
            CHECK_FALSE(field.tunable);  // 插件参数第一版不可调
            saw_slot = true;
        }
        if (field.path == "data.sequences") {
            CHECK(field.type == "list");
            saw_list = true;
        }
    }
    CHECK(saw_tunable);
    CHECK(saw_slot);
    CHECK(saw_list);

    const PluginSchema plugin{.interface_name = "OdometryBase",
                              .type = "my_lo",
                              .fields = {PluginField{.name = "a",
                                                     .kind = "int",
                                                     .default_value = "1",
                                                     .doc = "说明"}}};
    const auto json =
        schemaJson({{"system.modules.odometry.plugin", "OdometryBase"}}, {plugin});
    CHECK(json.find("\"system.modules.odometry.plugin\": \"OdometryBase\"") != std::string::npos);
    CHECK(json.find("\"type\": \"my_lo\"") != std::string::npos);
    CHECK(json.find("\"tunable\": true") != std::string::npos);
    CHECK(json.find("default_spec") == std::string::npos);  // 没有子槽就没有默认节点

    // 子槽的默认节点整个导出：按 schema 还原的一方（Python）照抄它，不另行推算
    const PluginSpec leaf{.type = "leaf", .params = {}, .children = {}};
    const PluginSpec voxel{.type = "voxel",
                           .params = {{"size", 0.5},
                                      {"label", std::string("a\"b")},
                                      {"on", true},
                                      {"n", std::int64_t{3}}},
                           .children = {{"inner", leaf}}};
    const PluginSchema nested{.interface_name = "OdometryBase",
                              .type = "nested",
                              .fields = {PluginField{.name = "map",
                                                     .kind = "slot:Map",
                                                     .default_value = "voxel",
                                                     .doc = "地图",
                                                     .default_spec = voxel}}};
    CHECK(schemaJson({}, {nested}).find(
              R"("default_spec": {"type": "voxel", "params": {"label": "a\"b", "n": 3, )"
              R"("on": true, "size": 0.5}, "children": {"inner": {"type": "leaf", )"
              R"("params": {}, "children": {}}}})") != std::string::npos);
}

TEST_CASE("配置文件读写往返", "[config]") {
    const TempDir dir("config_roundtrip");
    const auto path = dir.path() / "task.yaml";
    TaskConfig config;
    config.task.name = "roundtrip";
    config.system.packetizer.imu_required = false;
    config.system.packetizer.post_support_s = 0.05;
    writeConfigFile(path.string(), config);

    const auto loaded = loadTaskFile(path.string()).config;
    CHECK(loaded.task.name == "roundtrip");
    CHECK_FALSE(loaded.system.packetizer.imu_required);
    CHECK(loaded.system.packetizer.post_support_s == Approx(0.05));
    CHECK(configHash(loaded) == configHash(config));
}
