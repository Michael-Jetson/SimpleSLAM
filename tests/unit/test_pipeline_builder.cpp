/// @file test_pipeline_builder.cpp
/// PipelineBuilder 配置驱动管道组装器测试

#include <SimpleSLAM/runner/pipeline_builder.hpp>
#include <SimpleSLAM/core/infra/registry.hpp>
#include <SimpleSLAM/odometry/odometry_base.hpp>
#include <SimpleSLAM/sensor_io/sensor_source.hpp>
#include <SimpleSLAM/backend/service_base.hpp>

#include <catch2/catch_test_macros.hpp>

#include <fstream>

using namespace simpleslam;

namespace {

// ── Mock 数据源 ──
class MockSource final : public ISensorSource {
public:
    [[nodiscard]] bool hasNext() const override { return false; }
    std::optional<LidarScan> nextScan() override { return std::nullopt; }
    [[nodiscard]] Timestamp currentTimestamp() const override { return 0.0; }
};

class MockSourceWithConfig final : public ISensorSource {
public:
    std::string path;
    explicit MockSourceWithConfig(const YAML::Node& cfg)
        : path(cfg["path"].as<std::string>("")) {}
    [[nodiscard]] bool hasNext() const override { return false; }
    std::optional<LidarScan> nextScan() override { return std::nullopt; }
    [[nodiscard]] Timestamp currentTimestamp() const override { return 0.0; }
};

// ── Mock 里程计 ──
class MockOdom final : public OdometryBase {
public:
    OdometryResult processLidar(const LidarScan&) override { return {}; }
    void reset() override {}
    [[nodiscard]] std::string_view name() const override { return "MockOdom"; }
};

// ── Mock 后端服务 ──
class MockService final : public ServiceBase {
public:
    MockService() : ServiceBase("MockService") {}
};

/// 写 YAML 字符串到临时文件并返回路径
std::string writeTempYaml(const std::string& filename, const YAML::Node& root) {
    std::string path = "/tmp/" + filename;
    std::ofstream ofs(path);
    ofs << root;
    ofs.close();
    return path;
}

}  // namespace

// ── 注册 Mock 组件 ──
SIMPLESLAM_REGISTER_MODULE(ISensorSource, "pb_mock_source", MockSource);
SIMPLESLAM_REGISTER_MODULE(ISensorSource, "pb_mock_source_cfg", MockSourceWithConfig);
SIMPLESLAM_REGISTER_MODULE(OdometryBase, "pb_mock_odom", MockOdom);
SIMPLESLAM_REGISTER_MODULE(ServiceBase, "pb_mock_service", MockService);

// ── 测试 ──

TEST_CASE("PipelineBuilder fromConfig 创建完整 Runner", "[pipeline_builder]") {
    YAML::Node root;
    root["schema_version"] = 1;
    root["pipeline"]["source"]["type"] = "pb_mock_source";
    root["pipeline"]["odometry"]["type"] = "pb_mock_odom";
    root["pipeline"]["services"][0]["type"] = "pb_mock_service";

    auto config = Config::load(writeTempYaml("test_pb_full.yaml", root));
    auto runner = PipelineBuilder::fromConfig(config);
    REQUIRE(runner != nullptr);
}

TEST_CASE("PipelineBuilder fromConfig 无 services 也可运行", "[pipeline_builder]") {
    YAML::Node root;
    root["schema_version"] = 1;
    root["pipeline"]["source"]["type"] = "pb_mock_source";
    root["pipeline"]["odometry"]["type"] = "pb_mock_odom";

    auto config = Config::load(writeTempYaml("test_pb_nosvc.yaml", root));
    auto runner = PipelineBuilder::fromConfig(config);
    REQUIRE(runner != nullptr);
}

TEST_CASE("PipelineBuilder fromConfig 未注册 type 抛异常", "[pipeline_builder]") {
    YAML::Node root;
    root["schema_version"] = 1;
    root["pipeline"]["source"]["type"] = "nonexistent_source";
    root["pipeline"]["odometry"]["type"] = "pb_mock_odom";

    auto config = Config::load(writeTempYaml("test_pb_bad.yaml", root));
    REQUIRE_THROWS_AS(PipelineBuilder::fromConfig(config), std::runtime_error);
}

TEST_CASE("PipelineBuilder fromConfig 传递配置参数", "[pipeline_builder]") {
    YAML::Node root;
    root["schema_version"] = 1;
    root["pipeline"]["source"]["type"] = "pb_mock_source_cfg";
    root["pipeline"]["source"]["path"] = "/data/test";
    root["pipeline"]["odometry"]["type"] = "pb_mock_odom";

    auto config = Config::load(writeTempYaml("test_pb_cfg.yaml", root));
    auto runner = PipelineBuilder::fromConfig(config);
    REQUIRE(runner != nullptr);
}
