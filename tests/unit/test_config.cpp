/// @file test_config.cpp
/// 配置系统测试——验证 YAML 加载、合并、schema 校验

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <SimpleSLAM/core/infra/config.hpp>

#include <fstream>

using namespace simpleslam;

// 测试数据根目录（由 CMake 传入）
static const std::string kTestDataDir = SIMPLESLAM_TEST_DATA_DIR;

TEST_CASE("加载 defaults.yaml", "[config]") {
    auto cfg = Config::load(kTestDataDir + "/configs/defaults.yaml");
    REQUIRE(cfg.schemaVersion() == 1);
}

TEST_CASE("dot-path 访问标量值", "[config]") {
    auto cfg = Config::load(kTestDataDir + "/configs/defaults.yaml");

    // bool
    REQUIRE(cfg.get<bool>("logging.async") == true);

    // int
    REQUIRE(cfg.get<int>("logging.async_queue_size") == 8192);

    // string
    REQUIRE(cfg.get<std::string>("logging.console_level") == "info");

    // 带默认值的访问
    REQUIRE(cfg.get<int>("nonexistent.key", 42) == 42);
}

TEST_CASE("has() 路径检测", "[config]") {
    auto cfg = Config::load(kTestDataDir + "/configs/defaults.yaml");

    REQUIRE(cfg.has("logging"));
    REQUIRE(cfg.has("logging.async"));
    REQUIRE(cfg.has("timing.enabled"));
    REQUIRE_FALSE(cfg.has("nonexistent"));
    REQUIRE_FALSE(cfg.has("logging.nonexistent"));
}

TEST_CASE("不存在的路径抛出异常", "[config]") {
    auto cfg = Config::load(kTestDataDir + "/configs/defaults.yaml");
    REQUIRE_THROWS_AS(cfg.get<int>("nonexistent.key"), ConfigError);
}

TEST_CASE("YAML 深度合并", "[config]") {
    // 创建临时覆盖层文件
    std::string overlay_path = "/tmp/test_overlay.yaml";
    {
        std::ofstream out(overlay_path);
        out << "schema_version: 1\n"
            << "logging:\n"
            << "  console_level: debug\n"          // 覆盖
            << "  new_field: hello\n"              // 新增
            << "timing:\n"
            << "  enabled: false\n";               // 覆盖
    }

    auto cfg = Config::loadWithOverlays(
        kTestDataDir + "/configs/defaults.yaml", {overlay_path});

    // 被覆盖的值应该是 overlay 的值
    REQUIRE(cfg.get<std::string>("logging.console_level") == "debug");
    REQUIRE(cfg.get<bool>("timing.enabled") == false);

    // base 中有但 overlay 中没有的值应保留
    REQUIRE(cfg.get<int>("logging.async_queue_size") == 8192);
    REQUIRE(cfg.get<bool>("logging.async") == true);

    // overlay 新增的字段
    REQUIRE(cfg.get<std::string>("logging.new_field") == "hello");

    // 清理
    std::remove(overlay_path.c_str());
}

TEST_CASE("缺少 schema_version 抛出异常", "[config]") {
    // 创建无 schema_version 的临时文件
    std::string path = "/tmp/test_no_schema.yaml";
    {
        std::ofstream out(path);
        out << "logging:\n  async: true\n";
    }

    auto cfg = Config::load(path);
    REQUIRE_THROWS_AS(cfg.schemaVersion(), ConfigError);

    std::remove(path.c_str());
}

TEST_CASE("node() 返回子树", "[config]") {
    auto cfg = Config::load(kTestDataDir + "/configs/defaults.yaml");
    auto logging_node = cfg.node("logging");

    REQUIRE(logging_node.IsMap());
    REQUIRE(logging_node["async"].as<bool>() == true);
}

TEST_CASE("空路径返回根节点", "[config]") {
    auto cfg = Config::load(kTestDataDir + "/configs/defaults.yaml");
    auto root = cfg.node("");
    REQUIRE(root.IsMap());
    REQUIRE(root["schema_version"].as<int>() == 1);
}
