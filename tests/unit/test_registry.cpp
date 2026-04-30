/// @file test_registry.cpp
/// Registry 插件/模块注册表测试
///
/// 测试覆盖：
///   - 类型擦除插件注册与创建（有参/无参）
///   - 继承模块注册与创建（有参/无参）
///   - 查询 API（has, registered, registeredNames）
///   - 未注册名称抛异常
///   - 同类型多名（别名）行为
///   - 不同 Registry 实例隔离

#include <SimpleSLAM/core/infra/registry.hpp>
#include <SimpleSLAM/core/concepts/any_loop_detector.hpp>
#include <SimpleSLAM/odometry/odometry_base.hpp>
#include <SimpleSLAM/backend/service_base.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <string>

using namespace simpleslam;

// ═══════════════════════════════════════════════════════════════════
// Mock 类型定义（匿名命名空间，仅本 TU 可见）
// ═══════════════════════════════════════════════════════════════════

namespace {

// ── 插件 Mock：满足 LoopDetector concept ──

/// 无参构造的回环检测器 Mock
struct MockDetectorA {
    void addKeyframe(const KeyframeData&) {}
    std::optional<LoopCandidate> detect(const KeyframeData&) {
        return std::nullopt;
    }
};

/// 带 YAML 配置构造的回环检测器 Mock
/// 从配置中读取 threshold 参数，用于验证配置传递
struct MockDetectorB {
    int threshold = 0;

    explicit MockDetectorB(const YAML::Node& config)
        : threshold(config["threshold"].as<int>(10)) {}

    void addKeyframe(const KeyframeData&) {}
    std::optional<LoopCandidate> detect(const KeyframeData&) {
        return std::nullopt;
    }
};

// ── 模块 Mock：继承 OdometryBase ──

/// 无参构造的里程计 Mock
class MockOdometryA final : public OdometryBase {
public:
    OdometryResult processLidar(const LidarScan&) override { return {}; }
    void reset() override {}
    [[nodiscard]] std::string_view name() const override {
        return "MockOdometryA";
    }
};

/// 带 YAML 配置构造的里程计 Mock
/// 从配置中读取 max_iterations 参数
class MockOdometryB final : public OdometryBase {
public:
    int iters = 0;

    explicit MockOdometryB(const YAML::Node& config)
        : iters(config["max_iterations"].as<int>(10)) {}

    OdometryResult processLidar(const LidarScan&) override { return {}; }
    void reset() override {}
    [[nodiscard]] std::string_view name() const override {
        return "MockOdometryB";
    }
};

// ── 模块 Mock：继承 ServiceBase ──

/// 无参构造的后端服务 Mock
class MockServiceA final : public ServiceBase {
public:
    MockServiceA() : ServiceBase("MockServiceA") {}
};

}  // namespace

// ═══════════════════════════════════════════════════════════════════
// 文件作用域注册——静态初始化时自动执行
// ═══════════════════════════════════════════════════════════════════

// 插件注册（类型擦除）
SIMPLESLAM_REGISTER_PLUGIN(AnyLoopDetector, "mock_detector_a", MockDetectorA);
SIMPLESLAM_REGISTER_PLUGIN(AnyLoopDetector, "mock_detector_b", MockDetectorB);

// 模块注册（继承体系）
SIMPLESLAM_REGISTER_MODULE(OdometryBase, "mock_odom_a", MockOdometryA);
SIMPLESLAM_REGISTER_MODULE(OdometryBase, "mock_odom_b", MockOdometryB);
SIMPLESLAM_REGISTER_MODULE(ServiceBase, "mock_service_a", MockServiceA);

// 别名注册——同一类型以不同名注册（应产生 warning，但不 abort）
SIMPLESLAM_REGISTER_PLUGIN(AnyLoopDetector, "alias_detector_a", MockDetectorA);

// ═══════════════════════════════════════════════════════════════════
// 测试用例
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("Plugin 无参创建", "[registry]") {
    // MockDetectorA 默认构造，create 时不传配置
    auto det = Registry<AnyLoopDetector>::create("mock_detector_a");
    REQUIRE(det.valid());
}

TEST_CASE("Plugin YAML 配置创建", "[registry]") {
    // MockDetectorB 从 YAML 读取 threshold 参数
    YAML::Node config;
    config["threshold"] = 42;

    auto det = Registry<AnyLoopDetector>::create("mock_detector_b", config);
    REQUIRE(det.valid());

    // 验证 detect 正常工作（间接验证对象构造成功）
    KeyframeData kf;
    auto result = det.detect(kf);
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("Module OdometryBase 无参创建", "[registry]") {
    auto odom = Registry<std::unique_ptr<OdometryBase>>::create("mock_odom_a");
    REQUIRE(odom != nullptr);
    REQUIRE(odom->name() == "MockOdometryA");
}

TEST_CASE("Module OdometryBase YAML 配置创建", "[registry]") {
    YAML::Node config;
    config["max_iterations"] = 20;

    auto odom = Registry<std::unique_ptr<OdometryBase>>::create(
        "mock_odom_b", config);
    REQUIRE(odom != nullptr);
    REQUIRE(odom->name() == "MockOdometryB");

    // 向下转型验证配置传递
    auto* concrete = dynamic_cast<MockOdometryB*>(odom.get());
    REQUIRE(concrete != nullptr);
    REQUIRE(concrete->iters == 20);
}

TEST_CASE("Module ServiceBase 无参创建", "[registry]") {
    auto svc = Registry<std::unique_ptr<ServiceBase>>::create("mock_service_a");
    REQUIRE(svc != nullptr);
    REQUIRE(svc->name() == "MockServiceA");
}

TEST_CASE("has() 已注册返回 true", "[registry]") {
    REQUIRE(Registry<AnyLoopDetector>::has("mock_detector_a"));
    REQUIRE(Registry<AnyLoopDetector>::has("mock_detector_b"));
    REQUIRE(Registry<std::unique_ptr<OdometryBase>>::has("mock_odom_a"));
}

TEST_CASE("has() 未注册返回 false", "[registry]") {
    REQUIRE_FALSE(Registry<AnyLoopDetector>::has("nonexistent"));
    REQUIRE_FALSE(Registry<std::unique_ptr<OdometryBase>>::has("nonexistent"));
}

TEST_CASE("registered() 包含全部条目", "[registry]") {
    auto entries = Registry<AnyLoopDetector>::registered();
    // 至少包含 mock_detector_a, mock_detector_b, alias_detector_a
    REQUIRE(entries.size() >= 3);

    auto hasName = [&](const std::string& name) {
        return std::any_of(entries.begin(), entries.end(),
                           [&](const RegistryEntry& e) { return e.name == name; });
    };
    REQUIRE(hasName("mock_detector_a"));
    REQUIRE(hasName("mock_detector_b"));
    REQUIRE(hasName("alias_detector_a"));
}

TEST_CASE("registeredNames() 返回名称列表", "[registry]") {
    auto names = Registry<AnyLoopDetector>::registeredNames();
    REQUIRE(names.size() >= 3);

    auto contains = [&](const std::string& name) {
        return std::find(names.begin(), names.end(), name) != names.end();
    };
    REQUIRE(contains("mock_detector_a"));
    REQUIRE(contains("mock_detector_b"));
}

TEST_CASE("create 未注册名称 throw runtime_error", "[registry]") {
    REQUIRE_THROWS_AS(
        Registry<AnyLoopDetector>::create("not_registered"),
        std::runtime_error);

    // 验证错误信息包含已注册名称列表
    try {
        Registry<AnyLoopDetector>::create("not_registered");
    } catch (const std::runtime_error& e) {
        std::string msg = e.what();
        REQUIRE(msg.find("not_registered") != std::string::npos);
        REQUIRE(msg.find("mock_detector_a") != std::string::npos);
    }
}

TEST_CASE("同类型多名（别名）都能创建", "[registry]") {
    // MockDetectorA 注册了 "mock_detector_a" 和 "alias_detector_a"
    auto det1 = Registry<AnyLoopDetector>::create("mock_detector_a");
    auto det2 = Registry<AnyLoopDetector>::create("alias_detector_a");
    REQUIRE(det1.valid());
    REQUIRE(det2.valid());
}

TEST_CASE("不同 Registry 实例隔离", "[registry]") {
    // AnyLoopDetector 注册表和 unique_ptr<OdometryBase> 注册表互不影响
    REQUIRE(Registry<AnyLoopDetector>::has("mock_detector_a"));
    REQUIRE_FALSE(Registry<AnyLoopDetector>::has("mock_odom_a"));

    REQUIRE(Registry<std::unique_ptr<OdometryBase>>::has("mock_odom_a"));
    REQUIRE_FALSE(Registry<std::unique_ptr<OdometryBase>>::has("mock_detector_a"));
}

TEST_CASE("registered() 包含正确的 type_index", "[registry]") {
    auto entries = Registry<AnyLoopDetector>::registered();

    // 找到 mock_detector_a，验证 type_index 匹配 MockDetectorA
    auto it = std::find_if(entries.begin(), entries.end(),
                           [](const RegistryEntry& e) {
                               return e.name == "mock_detector_a";
                           });
    REQUIRE(it != entries.end());
    REQUIRE(it->concrete_type == std::type_index(typeid(MockDetectorA)));

    // alias_detector_a 的 type_index 也应该是 MockDetectorA
    auto alias_it = std::find_if(entries.begin(), entries.end(),
                                 [](const RegistryEntry& e) {
                                     return e.name == "alias_detector_a";
                                 });
    REQUIRE(alias_it != entries.end());
    REQUIRE(alias_it->concrete_type == std::type_index(typeid(MockDetectorA)));
}

// ═══════════════════════════════════════════════════════════════════
// Phase 7: 命名与日志测试
// ═══════════════════════════════════════════════════════════════════

TEST_CASE("Plugin create 后 name 等于注册名", "[registry]") {
    auto det = Registry<AnyLoopDetector>::create("mock_detector_a");
    REQUIRE(det.name() == "mock_detector_a");
}

TEST_CASE("Plugin 手动构造默认 name 等于类名", "[registry]") {
    AnyLoopDetector det(MockDetectorA{});
    REQUIRE(det.name() == "MockDetectorA");
}

TEST_CASE("Plugin setOwner 设置归属模块", "[registry]") {
    auto det = Registry<AnyLoopDetector>::create("mock_detector_a");
    det.setOwner("TestService");
    REQUIRE(det.owner() == "TestService");
    REQUIRE(det.name() == "mock_detector_a");
}

TEST_CASE("Plugin setName 覆盖名称", "[registry]") {
    AnyLoopDetector det(MockDetectorA{});
    REQUIRE(det.name() == "MockDetectorA");
    det.setName("custom_name");
    REQUIRE(det.name() == "custom_name");
}
