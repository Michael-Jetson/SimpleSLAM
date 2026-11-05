// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 配置：未知键与类型不符一律报错、缺省即默认、哈希只随内容变、
/// 可调字段进参数存储而声明不覆盖它、schema 从同一份字段清单导出。

#include <SimpleSLAM/config/loader.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>

using namespace simpleslam;
using namespace simpleslam::config;
using Catch::Approx;

TEST_CASE("缺省即默认值", "[config]") {
    const auto config = loadConfigString("odometry:\n  max_iterations: 7\n");
    CHECK(config.odometry.max_iterations == 7);
    CHECK(config.odometry.voxel_size_m == Approx(0.5));  // 没写的保持默认
    CHECK(config.runtime.worker_threads == 2);
    CHECK(config.name == "default");
}

TEST_CASE("未知键是错误而不是忽略", "[config]") {
    // 少一个字母的键静默跑完，比报错难查一百倍
    CHECK_THROWS_AS(loadConfigString("odometr:\n  max_iterations: 7\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("odometry:\n  max_iteration: 7\n"), ConfigError);
}

TEST_CASE("类型不符是错误，不做隐式转换", "[config]") {
    CHECK_THROWS_AS(loadConfigString("odometry:\n  max_iterations: abc\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("odometry:\n  max_iterations: 1.5\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("runtime:\n  offline: 1\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("runtime:\n  worker_threads: \"4\"\n"), ConfigError);
}

TEST_CASE("配置哈希只随内容变，不随注释与键序变", "[config]") {
    const auto a = loadConfigString("# 注释\nodometry:\n  max_iterations: 9\nname: run\n");
    const auto b = loadConfigString("name: run\nodometry:\n  max_iterations: 9\n");
    CHECK(configHash(a) == configHash(b));

    const auto c = loadConfigString("name: run\nodometry:\n  max_iterations: 10\n");
    CHECK(configHash(a) != configHash(c));
}

TEST_CASE("规范化导出可以再加载回来", "[config]") {
    auto config = loadConfigString("name: demo\nodometry:\n  algorithm: kiss\n");
    config.backend.loop_score_threshold = 0.85;
    const auto text = dumpConfig(config);
    const auto again = loadConfigString(text, "<dump>");
    CHECK(again.name == "demo");
    CHECK(again.odometry.algorithm == "kiss");
    CHECK(again.backend.loop_score_threshold == Approx(0.85));
    CHECK(configHash(again) == configHash(config));
}

TEST_CASE("规范化导出对刁钻字符串也无损", "[config]") {
    // `name: "a # b"` 不加引号导出，再加载只剩 `a`。
    for (const std::string nasty : {"a # b", "x: y", "\"quoted\"", "tail  ", "back\\slash",
                                    "line\nbreak", "true", "42", ""}) {
        SlamConfig config;
        config.name = nasty;
        config.evaluation.output = nasty;
        const auto again = loadConfigString(dumpConfig(config), "<dump>");
        CHECK(again.name == nasty);
        CHECK(again.evaluation.output == nasty);
        CHECK(configHash(again) == configHash(config));
    }
}

TEST_CASE("未知的空映射也是未知键", "[config]") {
    // 空映射没有叶子，按叶子遍历的未知键检查看不见它
    CHECK_THROWS_AS(loadConfigString("misspelled: {}\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("odometry:\n  nope: {}\n"), ConfigError);
}

TEST_CASE("类型对了不等于值有意义", "[config]") {
    CHECK_THROWS_AS(loadConfigString("runtime:\n  worker_threads: -1\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("runtime:\n  spin_budget: 0\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("packetizer:\n  post_support_s: -0.1\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("packetizer:\n  post_support_s: nan\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("packetizer:\n  post_support_s: 1e400\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("dataset:\n  kind: kitti\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("evaluation:\n  alignment: umeyama\n"), ConfigError);
    CHECK_THROWS_AS(loadConfigString("backend:\n  odometry_rotation_weight: 0\n"), ConfigError);
    // 命令行覆盖走同一套校验
    SlamConfig config;
    CHECK_THROWS_AS(applyOverrides(config, {"runtime.worker_threads=0"}), ConfigError);
}

TEST_CASE("命令行覆盖按 schema 的类型解析", "[config]") {
    SlamConfig config;
    applyOverrides(config, {"odometry.max_iterations=40", "runtime.offline=false",
                            "evaluation.output=/tmp/est.tum"});
    CHECK(config.odometry.max_iterations == 40);
    CHECK_FALSE(config.runtime.offline);
    CHECK(config.evaluation.output == "/tmp/est.tum");

    CHECK_THROWS_AS(applyOverrides(config, {"odometry.nope=1"}), ConfigError);
    CHECK_THROWS_AS(applyOverrides(config, {"odometry.max_iterations=abc"}), ConfigError);
    CHECK_THROWS_AS(applyOverrides(config, {"没有等号"}), ConfigError);
}

TEST_CASE("跨字段约束看全部覆盖到位之后的配置：先写开关、后补路径照样成立", "[config]") {
    SlamConfig config;
    applyOverrides(config, {"recording.enabled=true", "recording.output=/tmp/run.ssrec"});
    CHECK(config.recording.enabled);
    CHECK(config.recording.output == "/tmp/run.ssrec");

    // 文件里只写了开关，路径由命令行补上
    const auto loaded = loadConfigString("recording:\n  enabled: true\n", "<string>",
                                         {"recording.output=/tmp/run.ssrec"});
    CHECK(loaded.recording.output == "/tmp/run.ssrec");

    // 最终仍没有路径：拒绝，不悄悄不录
    CHECK_THROWS_AS(loadConfigString("recording:\n  enabled: true\n"), ConfigError);
    SlamConfig missing;
    CHECK_THROWS_AS(applyOverrides(missing, {"recording.enabled=true"}), ConfigError);
}

TEST_CASE("schema 从同一份字段清单导出", "[config]") {
    const auto fields = schema();
    CHECK(fields.size() > 20);

    bool saw_tunable = false;
    bool saw_fixed = false;
    for (const auto& field : fields) {
        CHECK_FALSE(field.path.empty());
        CHECK_FALSE(field.doc.empty());  // 每个字段都要有说明，否则 Python 侧只能猜
        if (field.path == "odometry.keyframe_distance_m") {
            CHECK(field.type == "double");
            CHECK(field.tunable);  // 算法每帧从本帧参数快照里读它
            saw_tunable = true;
        }
        if (field.path == "odometry.max_iterations") {
            CHECK(field.type == "int");
            CHECK(field.default_value == "30");
            // 没有任何代码在运行期重新读它：标成可调的话，结果上的参数版本就是假的
            CHECK_FALSE(field.tunable);
        }
        if (field.path == "runtime.worker_threads") {
            CHECK_FALSE(field.tunable);  // 线程数改了要重启，不是"改了没生效"
            saw_fixed = true;
        }
    }
    CHECK(saw_tunable);
    CHECK(saw_fixed);

    const auto json = schemaJson();
    CHECK(json.find("\"odometry.keyframe_distance_m\"") != std::string::npos);
    CHECK(json.find("\"tunable\": true") != std::string::npos);
}

TEST_CASE("可调字段进参数存储，声明不覆盖它", "[config][params]") {
    const auto config = loadConfigString("odometry:\n  keyframe_distance_m: 2.5\n");
    comm::ParameterStore store;
    applyToParameters(config, store);

    const std::string name = configParamName("odometry.keyframe_distance_m");
    CHECK(name == "/config/odometry/keyframe_distance_m");
    CHECK(store.snapshot()->get<double>(name) == 2.5);

    // 模块随后按代码默认值声明：配置给的值优先
    CHECK(store.declare<double>(name, 1.0) == comm::ParamDeclare::kAlreadyPresent);
    CHECK(store.snapshot()->get<double>(name) == 2.5);

    // 不可调字段不进参数存储：改了也不会生效，不如根本不提供这个口子
    CHECK_FALSE(store.snapshot()->has(configParamName("runtime.worker_threads")));
}

TEST_CASE("运行期改过的可调字段可以读回配置", "[config][params]") {
    auto config = loadConfigString("odometry:\n  keyframe_distance_m: 2.5\n");
    comm::ParameterStore store;
    applyToParameters(config, store);

    const auto result = store.commit(
        {{configParamName("odometry.keyframe_distance_m"), 3.5}});
    REQUIRE(result.ok());

    readFromParameters(config, *store.snapshot());
    CHECK(config.odometry.keyframe_distance_m == 3.5);
}

TEST_CASE("配置文件读写往返", "[config]") {
    const auto path = std::filesystem::temp_directory_path() / "simpleslam_config.yaml";
    SlamConfig config;
    config.name = "roundtrip";
    config.packetizer.imu_required = false;
    config.packetizer.post_support_s = 0.05;
    writeConfigFile(path.string(), config);

    const auto loaded = loadConfigFile(path.string());
    CHECK(loaded.name == "roundtrip");
    CHECK_FALSE(loaded.packetizer.imu_required);
    CHECK(loaded.packetizer.post_support_s == Approx(0.05));
    std::filesystem::remove(path);
}
