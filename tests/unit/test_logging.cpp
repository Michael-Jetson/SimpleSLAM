// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

// 日志：来源跟着装配路径走、帧与时刻自动带上、按来源设等级、限频、后台线程的缓冲与先后、
// run.log 稍后接上、事件与度量交给诊断。

#include <SimpleSLAM/config/plugin.hpp>
#include <SimpleSLAM/logging/construction.hpp>
#include <SimpleSLAM/logging/router.hpp>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace lg = simpleslam::logging;
namespace cfg = simpleslam::config;
using namespace std::chrono_literals;

namespace {

/// 测试用的日志组件：记录收进向量，终端写进字符串流。
struct Capture {
    std::ostringstream out;
    std::ostringstream err;
    std::mutex mutex;
    std::vector<lg::Record> records;
    std::shared_ptr<lg::Router> router;

    explicit Capture(lg::LogOptions options = {}) {
        options.out = &out;
        options.err = &err;
        options.tap = [this](const lg::Record& record) {
            std::lock_guard lock(mutex);
            records.push_back(record);
        };
        router = lg::Router::create(std::move(options));
    }

    std::vector<lg::Record> snapshot() {
        router->flush();
        std::lock_guard lock(mutex);
        return records;
    }
};

/// 一个插件接口与两个实现：父插件构造时建一个子插件，跑起来以后再建一个。
class Step : public lg::Loggable {
public:
    static constexpr std::string_view kInterfaceName = "TestStep";
    virtual ~Step() = default;
    void say(std::string_view what) const { log_.info("{}", what); }
    [[nodiscard]] const lg::Logger& logger() const { return log_; }
};

class Leaf final : public Step {
public:
    static constexpr std::string_view kTypeName = "leaf";
    struct Config {
        int value = 0;
        template <class V>
        void visit(V& v) {
            v("value", value, "值");
        }
    };
    explicit Leaf(const Config& config) { log_.info("构造 leaf {}", config.value); }
};

class Parent final : public Step {
public:
    static constexpr std::string_view kTypeName = "parent";
    struct Config {
        cfg::Slot<Step> child = cfg::Slot<Step>::of<Leaf>();
        cfg::Slot<Step> later = cfg::Slot<Step>::of<Leaf>();
        template <class V>
        void visit(V& v) {
            v("child", child, "构造时建");
            v("later", later, "跑起来以后才建");
        }
    };
    explicit Parent(const Config& config) : config_(config), child_(config.child.make()) {
        log_.info("构造 parent");
    }
    void grow() { later_ = config_.later.make(); }
    [[nodiscard]] const Step& child() const { return *child_; }
    [[nodiscard]] const Step& later() const { return *later_; }

private:
    Config config_;
    std::unique_ptr<Step> child_;
    std::unique_ptr<Step> later_;
};

std::vector<std::string> texts(const std::vector<lg::Record>& records) {
    std::vector<std::string> out;
    for (const auto& record : records) out.push_back(record.text);
    return out;
}

}  // namespace

TEST_CASE("等级的写法", "[logging]") {
    for (const lg::Level level :
         {lg::Level::kDebug, lg::Level::kInfo, lg::Level::kWarn, lg::Level::kError}) {
        CHECK(lg::parseLevel(lg::toString(level)) == level);
    }
    CHECK_FALSE(lg::parseLevel("verbose").has_value());
    CHECK_FALSE(lg::parseLevel("INFO").has_value());
}

TEST_CASE("来源跟着装配路径走：模块 › 实现类型 › 子槽(实现类型)", "[logging][source]") {
    Capture capture;
    const lg::Logger module = capture.router->root("/slam/odometry");
    auto parent = cfg::Slot<Step>::of<Parent>().make(module, "plugin");
    auto& concrete = static_cast<Parent&>(*parent);
    concrete.grow();

    CHECK(module.source() == "/slam/odometry");
    CHECK(module.key() == "odometry");
    CHECK(parent->logger().source() == "/slam/odometry › parent");
    CHECK(parent->logger().key() == "odometry.plugin");
    // 构造函数里建的子插件挂在父插件下面
    CHECK(concrete.child().logger().source() == "/slam/odometry › parent › child(leaf)");
    CHECK(concrete.child().logger().key() == "odometry.plugin.child");
    // 跑起来以后才建的也一样：父插件交来的槽记着父插件
    CHECK(concrete.later().logger().source() == "/slam/odometry › parent › later(leaf)");
    CHECK(concrete.later().logger().key() == "odometry.plugin.later");

    // 构造函数里写的日志也带着完整来源
    const auto records = capture.snapshot();
    REQUIRE(records.size() == 3);
    CHECK(records[0].text == "构造 leaf 0");
    CHECK(records[0].source == "/slam/odometry › parent › child(leaf)");
    CHECK(records[1].text == "构造 parent");
    CHECK(records[1].source == "/slam/odometry › parent");
    CHECK(records[2].source == "/slam/odometry › parent › later(leaf)");
}

TEST_CASE("试构造不记日志，真正构造时记一份", "[logging][source]") {
    Capture capture;
    cfg::Catalogs catalogs;
    catalogs.of<Step>().add<Parent>();
    catalogs.of<Step>().add<Leaf>();
    const cfg::PluginSpec spec{.type = "parent", .params = {}, .children = {}};
    const auto slot = catalogs.resolve<Step>(spec, "system.modules.odometry.plugin");
    CHECK(capture.snapshot().empty());  // 还原时试构造了一次，日志一条没有

    auto parent = slot.make(capture.router->root("/slam/odometry"), "plugin");
    CHECK(texts(capture.snapshot()) == std::vector<std::string>{"构造 leaf 0", "构造 parent"});
}

TEST_CASE("手工构造的组件交给模块之后由模块接上", "[logging][source]") {
    Capture capture;
    Leaf leaf(Leaf::Config{});  // 不经槽：没接上，构造时那条写 stderr
    CHECK(leaf.logger().source().empty());

    const lg::Logger module = capture.router->root("/slam/odometry");
    lg::adopt(leaf, module, "plugin", "Leaf");
    CHECK(leaf.logger().source() == "/slam/odometry › Leaf");
    CHECK(leaf.logger().key() == "odometry.plugin");
    leaf.say("接上了");
    const auto records = capture.snapshot();
    REQUIRE(records.size() == 1);
    CHECK(records[0].source == "/slam/odometry › Leaf");

    // 已经接上的不动
    lg::adopt(leaf, capture.router->root("/slam/other"), "plugin", "X");
    CHECK(leaf.logger().source() == "/slam/odometry › Leaf");
}

TEST_CASE("帧序号与数据时刻由模块设上，插件沿来源读到", "[logging][frame]") {
    Capture capture;
    std::int64_t now = 5'000'000'000;
    capture.router->setClock([&] { return now; });
    const lg::Logger module = capture.router->root("/slam/odometry");
    auto leaf = cfg::Slot<Step>::of<Leaf>().make(module, "plugin");

    module.setFrame({.sequence = 7, .data_time_ns = 123, .session = 1, .parameter_revision = 2});
    leaf->logger().setFrame({.sequence = 99, .data_time_ns = 0, .session = 0,
                             .parameter_revision = 0});  // 插件设不了
    leaf->say("这一帧");
    const auto records = capture.snapshot();
    REQUIRE(records.size() == 2);
    const auto& record = records[1];
    REQUIRE(record.frame.has_value());
    CHECK(record.frame->sequence == 7);
    CHECK(record.frame->data_time_ns == 123);
    CHECK(record.time_ns == now);
    CHECK_FALSE(records[0].frame.has_value());  // 构造时还没有帧

    // 同一个名字拿到同一个根，帧也是同一份
    CHECK(capture.router->root("/slam/odometry").key() == "odometry");
}

TEST_CASE("按来源设等级：最长的前缀生效，终端与 run.log 都照它", "[logging][level]") {
    lg::LogOptions options;
    options.rules = {{.key = "odometry", .level = lg::Level::kWarn},
                     {.key = "odometry.plugin.child", .level = lg::Level::kDebug}};
    Capture capture(std::move(options));
    const lg::Logger module = capture.router->root("/slam/odometry");
    auto parent = cfg::Slot<Step>::of<Parent>().make(module, "plugin");
    const auto& concrete = static_cast<const Parent&>(*parent);

    CHECK_FALSE(module.enabled(lg::Level::kInfo));
    CHECK(module.enabled(lg::Level::kWarn));
    CHECK_FALSE(parent->logger().enabled(lg::Level::kInfo));       // 管到 odometry 下面的全部
    CHECK(concrete.child().logger().enabled(lg::Level::kDebug));   // 更长的规则胜出
    // 段前缀：odometry 管不到 odometry2；没有规则时 run.log 收 debug
    CHECK(capture.router->root("/slam/odometry2").enabled(lg::Level::kDebug));

    // 默认：终端 info，run.log debug
    const lg::Logger other = capture.router->root("/slam/other");
    other.debug("只进文件");
    other.info("两边都有");
    const auto records = capture.snapshot();
    REQUIRE(records.size() == 3);  // 子插件构造时的那条 info 加上这两条
    CHECK_FALSE(records[1].to_console);
    CHECK(records[1].to_file);
    CHECK(records[2].to_console);
    CHECK(capture.out.str().find("只进文件") == std::string::npos);
}

TEST_CASE("限频：同一行每隔一段至多一条，下一条写明合并了几条", "[logging][throttle]") {
    Capture capture;
    std::int64_t now = 0;
    capture.router->setClock([&] { return now; });
    const lg::Logger module = capture.router->root("/slam/odometry");
    const auto burst = [&](int count) {
        for (int i = 0; i < count; ++i) module.every(1s).warn("第 {} 次", i);
    };
    burst(5);
    now += 500'000'000;
    burst(3);  // 还没到 1 秒
    now += 600'000'000;
    burst(1);
    const auto records = capture.snapshot();
    REQUIRE(records.size() == 2);
    CHECK(records[0].text == "第 0 次");
    CHECK(records[1].text == "第 0 次（期间合并了 7 条）");
}

TEST_CASE("执行器自己的输出在终端上只印正文，别的带等级、时刻、帧、来源", "[logging][format]") {
    Capture capture;
    capture.router->setClock([] { return std::int64_t{1'317'402'000'000}; });
    capture.router->root("task", "", true).info("跑完: {} 包", 30);
    const lg::Logger module = capture.router->root("/slam/odometry");
    module.setFrame({.sequence = 153, .data_time_ns = 0, .session = 0, .parameter_revision = 0});
    module.warn("内点只有 {} 个", 31);
    capture.router->flush();
    CHECK(capture.out.str() == "跑完: 30 包\n");
    CHECK(capture.err.str() == "W 1317.402 #153 /slam/odometry  内点只有 31 个\n");
}

TEST_CASE("run.log 稍后接上：之前该进文件的记录接上时补写，带代码位置", "[logging][file]") {
    const auto path = std::filesystem::temp_directory_path() / "simpleslam_test_run.log";
    std::filesystem::remove(path);
    {
        lg::LogOptions options;
        options.defer_file = true;
        Capture capture(std::move(options));
        const lg::Logger module = capture.router->root("/slam/odometry");
        module.debug("接上之前");
        capture.router->flush();
        capture.router->openFile(path);
        module.info("接上之后");
        capture.router->root("task", "", true).info("执行器");
        capture.router->close();
    }
    std::ifstream in(path);
    std::vector<std::string> lines;
    for (std::string line; std::getline(in, line);) lines.push_back(line);
    REQUIRE(lines.size() == 3);
    CHECK(lines[0].starts_with("D 0.000 - /slam/odometry (test_logging.cpp:"));
    CHECK(lines[0].ends_with("  接上之前"));
    CHECK(lines[1].ends_with("  接上之后"));
    CHECK(lines[2] == "I 0.000 - task  执行器");  // 执行器的行不带代码位置
    std::filesystem::remove(path);
}

TEST_CASE("缓冲满了丢最旧的 debug、info；warn 与执行器的输出不丢，先后不乱", "[logging][buffer]") {
    std::mutex mutex;
    std::condition_variable changed;
    bool entered = false;
    bool released = false;
    std::vector<std::string> delivered;
    lg::LogOptions options;
    options.capacity = 3;
    std::ostringstream out;
    std::ostringstream err;
    options.out = &out;
    options.err = &err;
    // 第一条交到接收端时卡住后台线程，让后面的记录在缓冲里排队
    options.tap = [&](const lg::Record& record) {
        std::unique_lock lock(mutex);
        delivered.push_back(record.text);
        if (!entered) {
            entered = true;
            changed.notify_all();
            changed.wait(lock, [&] { return released; });
        }
    };
    auto router = lg::Router::create(std::move(options));
    const lg::Logger module = router->root("/slam/odometry");
    const lg::Logger runner = router->root("task", "", true);
    module.info("i0");
    {
        std::unique_lock lock(mutex);
        changed.wait(lock, [&] { return entered; });
    }
    for (int i = 1; i <= 6; ++i) {
        module.info("i{}", i);
        if (i == 2) module.warn("w");
        if (i == 4) runner.info("执行器");
    }
    {
        std::lock_guard lock(mutex);
        released = true;
    }
    changed.notify_all();
    router->flush();
    CHECK(router->dropped() == 3);  // i1、i2、i3 被挤掉
    std::lock_guard lock(mutex);
    CHECK(delivered == std::vector<std::string>{"i0", "w", "i4", "执行器", "i5", "i6"});
}

TEST_CASE("事件与度量交给诊断：一条不丢，带模块、来源与帧", "[logging][event]") {
    Capture capture;
    std::vector<lg::Event> events;
    capture.router->setEventSink([&](const lg::Event& event) { events.push_back(event); });
    const lg::Logger module = capture.router->root("/slam/odometry");
    auto leaf = cfg::Slot<Step>::of<Leaf>().make(module, "plugin");
    module.setFrame({.sequence = 4, .data_time_ns = 40, .session = 1, .parameter_revision = 3});
    leaf->logger().event("degenerate", "走廊");
    leaf->logger().metric("icp.iterations", 12);
    {
        const auto timer = leaf->logger().timer("deskew");
    }
    REQUIRE(events.size() == 3);  // 在调用线程上同步交出
    CHECK(events[0].kind == lg::Event::Kind::kEvent);
    CHECK(events[0].module == "/slam/odometry");
    CHECK(events[0].source == "odometry.plugin");
    CHECK(events[0].detail == "走廊");
    REQUIRE(events[0].frame.has_value());
    CHECK(events[0].frame->sequence == 4);
    CHECK(events[0].frame->parameter_revision == 3);
    CHECK(events[1].kind == lg::Event::Kind::kMetric);
    CHECK(events[1].value == 12);
    CHECK(events[2].key == "deskew.duration");
    CHECK(events[2].unit == "ms");
    CHECK(events[2].value >= 0.0);
}

TEST_CASE("几个线程同时写：一条不少，各线程内的先后不乱", "[logging][concurrency]") {
    Capture capture;
    constexpr int kThreads = 4;
    constexpr int kPerThread = 500;
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&, t] {
            const lg::Logger module = capture.router->root("/slam/m" + std::to_string(t));
            for (int i = 0; i < kPerThread; ++i) module.warn("{}", i);  // warn 不丢
        });
    }
    for (auto& thread : threads) thread.join();
    const auto records = capture.snapshot();
    REQUIRE(records.size() == static_cast<std::size_t>(kThreads * kPerThread));
    std::vector<int> next(kThreads, 0);
    bool ordered = true;
    for (const auto& record : records) {
        const int t = record.key.back() - '0';
        ordered = ordered && std::stoi(record.text) == next[t]++;
    }
    CHECK(ordered);
}

TEST_CASE("同步模式与关掉之后：在调用线程上直接写，不另开线程", "[logging][sync]") {
    lg::LogOptions options;
    options.asynchronous = false;
    Capture capture(std::move(options));
    const lg::Logger module = capture.router->root("/slam/odometry");
    module.info("同步");
    CHECK(capture.out.str() == "I 0.000 - /slam/odometry  同步\n");  // 不用等
    capture.router->close();
    module.info("关掉之后");  // 写 stderr，不崩、不进别的去向
    CHECK(capture.records.size() == 1);
}
