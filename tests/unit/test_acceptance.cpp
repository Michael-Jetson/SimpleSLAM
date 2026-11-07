// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 验收场景：按"系统在这种情形下必须怎样"来写，不按"多了几个字段"来写。
///
/// 每个用例描述一个会造成丢帧、错误身份、持续积压或错误通过的边界，断言的是正确行为。
/// 通信与生命周期、打包与确认、标定身份、评测，分节排列。

#include <SimpleSLAM/config/calibration_io.hpp>
#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/eval/metrics.hpp>
#include <SimpleSLAM/framework/deskew.hpp>
#include <SimpleSLAM/modules/slam_system.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <thread>
#include <typeindex>
#include <vector>

using namespace simpleslam;
using Catch::Approx;
using namespace std::chrono_literals;
namespace comm = simpleslam::comm;
namespace fw = simpleslam::fw;
namespace modules = simpleslam::modules;
namespace plugin = simpleslam::plugin;
namespace cfg = simpleslam::config;
namespace eval = simpleslam::eval;

namespace {

/// 一个运行中可改、只有外部写的整数参数 x。
void defineX(comm::Runtime& runtime, std::int64_t initial) {
    runtime.params().define({comm::ParamDefinition{.name = "x",
                                                   .type = comm::ParamType::kInt,
                                                   .fallback = std::int64_t{0},
                                                   .doc = "",
                                                   .min = std::nullopt,
                                                   .max = std::nullopt,
                                                   .runtime = true,
                                                   .writers = {}}},
                            {{"x", initial}});
}

/// 直接把内置模块挂到运行时上时，参数要先定义：按内置模块的声明，取默认值。
void defineModuleParams(comm::Runtime& runtime) {
    auto resolved = comm::resolveParams(modules::builtinModuleParams(true), {});
    runtime.params().define(std::move(resolved.definitions), std::move(resolved.values));
}

struct Provider final : comm::Module {
    int handle(const int& n) { return n + 1; }
    void onInit() override { advertiseService<int, int>("/svc", &Provider::handle); }
};

struct Client final : comm::Module {
    comm::ServiceClient<int, int> cli;
    void onInit() override { cli = serviceClient<int, int>("/svc"); }
};

struct ParamWatcher final : comm::Module {
    [[nodiscard]] std::uint64_t revision() const { return paramSnapshot()->revision; }
};

/// 每帧建议关键帧的里程计；可在 track 中注入一段钩子。
/// 什么都不改、直接报收敛的优化器：只用来触发求解路径。
struct IdleOptimizer final : plugin::GraphOptimizerBase {
    int calls = 0;
    Report onOptimize(fw::GraphProblem&) override {
        ++calls;
        return Report{};
    }
};

struct HookOdometry final : plugin::OdometryBase {
    std::function<void()> hook;
    explicit HookOdometry(std::function<void()> fn = {}) : hook(std::move(fn)) {}
    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }
    plugin::TrackResult track(const fw::SensorPacket&) override {
        if (hook) hook();
        plugin::TrackResult r;
        r.suggest_keyframe = true;
        return r;
    }
};

void settle(comm::Runtime& runtime) {
    while (runtime.spinOnce() != 0) {
    }
}

fw::Ptr<fw::LidarScan> scanAt(fw::Timestamp t) {
    auto s = std::make_shared<fw::LidarScan>();
    s->reference_time = t;
    s->coverage = fw::TimeRange{t - 100ms, t};
    return s;
}

fw::Ptr<fw::ImuSample> imuAt(fw::Timestamp t, fw::ClockDomainId domain = fw::kReferenceClock) {
    auto s = std::make_shared<fw::ImuSample>();
    s->time = t;
    s->domain = domain;
    return s;
}

fw::CalibrationData validCalibration() {
    fw::CalibrationData calibration;
    calibration.device = "acceptance";
    fw::Extrinsic lidar;
    lidar.source = fw::CalibrationSource::kFile;
    calibration.extrinsics["lidar"] = lidar;
    return calibration;
}

}  // namespace

// ─────────────────────────── 通信与生命周期 ───────────────────────────

TEST_CASE("收件箱已关时发布不留下无人认领的消息", "[acceptance][comm]") {
    auto hub = std::make_shared<comm::Hub>();
    auto box = std::make_shared<comm::Mailbox>("/x", hub, comm::ErrorReporter{});
    auto topic = std::make_shared<comm::Topic>("/x", std::type_index(typeid(int)), hub, comm::ErrorReporter{}, 0);
    auto sub = topic->subscribe(box, 4, [](const comm::AnyMsg&) {});
    box->close();
    comm::Publisher<int> pub(topic, comm::PublishOptions{});
    pub.publish(1);
    CHECK(sub->stats().depth == 0);
    CHECK(hub->idle());
}

TEST_CASE("同批到期的一次性定时器里，先执行的能取消后一个", "[acceptance][comm]") {
    comm::TimerQueue timers(std::make_shared<std::atomic<std::uint64_t>>(0));
    comm::TimerHandle second;
    int fired = 0;
    (void)timers.schedule(comm::Time{1ns}, comm::Duration{0}, [&] { second.cancel(); });
    second = timers.schedule(comm::Time{1ns}, comm::Duration{0}, [&] { ++fired; });
    comm::fireDueTimers({&timers}, comm::Time{1ns});
    CHECK(fired == 0);
    CHECK_FALSE(timers.nextDue().has_value());
}

TEST_CASE("每个受理的服务调用恰好结案一次，超时不会再结案第二次", "[acceptance][comm]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    runtime.addModule<Provider>("/p");
    auto& client = runtime.addModule<Client>("/c");
    runtime.init();

    int completions = 0;
    for (int i = 0; i < 5000; ++i) {
        REQUIRE(client.cli.asyncCall(i, 1h, [&](auto) { ++completions; }) == comm::ServiceStatus::kOk);
    }
    settle(runtime);
    CHECK(completions == 5000);

    runtime.virtualClock()->advanceTo(2h);
    settle(runtime);
    CHECK(completions == 5000);
}

TEST_CASE("受理后调用方关闭：不在已关闭的模块上回调，丢弃可见", "[acceptance][comm]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    runtime.addModule<Provider>("/p");
    auto& client = runtime.addModule<Client>("/c");
    runtime.init();

    int completions = 0;
    const auto status = client.cli.asyncCall(1, 0ns, [&](auto) { ++completions; });
    REQUIRE(status == comm::ServiceStatus::kOk);
    auto box = client.mailbox();
    box->close();
    settle(runtime);

    CHECK(completions == 0);
    CHECK(box->stats().discarded >= 1);
}

TEST_CASE("关闭之后 spinFor 立即返回", "[acceptance][comm]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    runtime.shutdown();
    CHECK(runtime.spinFor(10ns) == 0);
}

TEST_CASE("装配阶段之外不能加模块，init 只能一次", "[acceptance][comm]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    runtime.addModule<Provider>("/p");
    runtime.init();
    CHECK_THROWS_AS(runtime.addModule<Client>("/c"), comm::WiringError);
    CHECK_THROWS_AS(runtime.init(), comm::WiringError);
}

TEST_CASE("关闭开始后参数写入被拒，存储不变", "[acceptance][comm]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    defineX(runtime, 1);
    runtime.addModule<Client>("/c");
    runtime.init();
    runtime.shutdown();
    const auto result = runtime.setParams({{"x", std::int64_t{2}}});
    CHECK(result.status == comm::ParamCommit::kRejectedStopped);
    CHECK(runtime.params().snapshot()->revision == 0);
}

TEST_CASE("提交只进了存储，模块在自己的收件箱里才换上", "[acceptance][comm]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    defineX(runtime, 1);
    auto& watcher = runtime.addModule<ParamWatcher>("/c");
    runtime.init();

    const auto result = runtime.setParams({{"x", std::int64_t{2}}});
    REQUIRE(result.ok());
    CHECK(watcher.revision() == 0);  // 模块还没被驱动

    settle(runtime);
    CHECK(watcher.revision() == result.revision);
}

TEST_CASE("并发发布到同一订阅端：每个生产者自己的顺序保持", "[acceptance][comm]") {
    // 排格在订阅端锁内：同一订阅端里，进队列的次序就是格子的次序
    auto hub = std::make_shared<comm::Hub>();
    auto box = std::make_shared<comm::Mailbox>("/m", hub, comm::ErrorReporter{});
    auto topic = std::make_shared<comm::Topic>("/t", std::type_index(typeid(int)), hub, comm::ErrorReporter{}, 0);
    std::vector<int> seen;
    auto sub = topic->subscribe(box, 0, [&](const comm::AnyMsg& m) { seen.push_back(*static_cast<const int*>(m.get())); });
    constexpr int kPerProducer = 2000;
    auto produce = [&](int base) {
        comm::Publisher<int> pub(topic, comm::PublishOptions{});
        for (int i = 0; i < kPerProducer; ++i) pub.publish(base + i);
    };
    std::thread a(produce, 0);
    std::thread b(produce, 100000);
    a.join();
    b.join();
    while (auto ready = hub->popReady()) ready->drainSome();

    REQUIRE(seen.size() == 2 * kPerProducer);
    int last_a = -1;
    int last_b = 99999;
    for (const int v : seen) {
        if (v < 100000) {
            CHECK(v == last_a + 1);
            last_a = v;
        } else {
            CHECK(v == last_b + 1);
            last_b = v;
        }
    }
}

// ─────────────────────────── 打包与确认 ───────────────────────────

TEST_CASE("不在参考时钟域的 IMU 被拒收并标缺口", "[acceptance][packetizer]") {
    fw::Packetizer packetizer;
    packetizer.feed(imuAt(fw::Timestamp{100ms}, fw::ClockDomainId{2}));
    auto packet = packetizer.feed(scanAt(fw::Timestamp{100ms}));
    // 没有可用的 IMU：包还在等括点
    CHECK_FALSE(packet.has_value());
    CHECK(packetizer.foreignClockDrops() == 1);
    packet = packetizer.flush();
    REQUIRE(packet.has_value());
    CHECK(packet->imu.view().empty());
    CHECK(packet->gap == fw::GapReason::kClockDomain);
}

TEST_CASE("两端有样本不等于中间连续", "[acceptance][packetizer]") {
    fw::Packetizer packetizer;
    packetizer.feed(imuAt(fw::Timestamp{0}));
    packetizer.feed(imuAt(fw::Timestamp{10s}));
    auto packet = packetizer.feed(scanAt(fw::Timestamp{10s}));
    REQUIRE(packet.has_value());
    CHECK(packet->imu_max_gap == fw::Duration{10s});
    CHECK(packet->gap == fw::GapReason::kImuGap);
}

TEST_CASE("时间倒流的 IMU 被丢弃，不进取区间的算法", "[acceptance][packetizer]") {
    fw::Packetizer packetizer;
    packetizer.feed(imuAt(fw::Timestamp{100ms}));
    packetizer.feed(imuAt(fw::Timestamp{50ms}));   // 倒流
    packetizer.feed(imuAt(fw::Timestamp{100ms}));  // 重复
    CHECK(packetizer.outOfOrderImu() == 2);
    CHECK(packetizer.imuBacklog() == 1);
}

TEST_CASE("低频惯性流的不同相位下每一帧都发得出来", "[acceptance][packetizer]") {
    // 10 Hz 扫描、10 Hz 惯性流、惯性样本落在扫描中段：每一帧等到终点之后第一条样本
    // （右括点）就发，不会被下一帧顶掉
    fw::PacketizerPolicy policy;
    policy.max_imu_gap = 150ms;
    fw::Packetizer packetizer(policy);
    int packets = 0;
    int bracketed = 0;
    auto consume = [&](std::optional<fw::SensorPacket> packet) {
        while (packet) {
            ++packets;
            const auto samples = packet->imu.view();
            if (!samples.empty() && samples.front().time <= packet->imu_coverage.begin
                && samples.back().time >= packet->reference_time) {
                ++bracketed;
            }
            packet = packetizer.poll();
        }
    };
    consume(packetizer.feed(imuAt(fw::Timestamp{50ms})));
    for (int i = 1; i <= 8; ++i) {
        consume(packetizer.feed(scanAt(fw::Timestamp{i * 100ms})));
        consume(packetizer.feed(imuAt(fw::Timestamp{i * 100ms + 50ms})));
    }
    CHECK(packets == 8);
    CHECK(bracketed == 8);
    CHECK(packetizer.droppedTriggers() == 0);
}

TEST_CASE("附件队列有界", "[acceptance][packetizer]") {
    fw::PacketizerPolicy policy;
    policy.imu = fw::ImuRequirement::kNotRequired;
    policy.max_pending_attachments = 3;
    fw::Packetizer packetizer(policy);
    for (int i = 0; i < 10; ++i) {
        auto fix = std::make_shared<fw::GnssFix>();
        fix->time = fw::Timestamp{50s + i * 1ms};
        packetizer.feed(fw::Ptr<fw::GnssFix>(fix));
    }
    CHECK(packetizer.pendingAttachments() <= 3);
}

// ─────────────────────────── 标定身份 ───────────────────────────

TEST_CASE("帧处理中改标定：结果与关键帧都用帧开始时那一份", "[acceptance][calibration]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    modules::SlamResources resources;
    resources.setCalibration(validCalibration());
    bool changed = false;
    auto& odometry = runtime.addModule<modules::OdometryModule>(
        "/o", std::make_unique<HookOdometry>([&] {
            if (changed) return;
            changed = true;
            resources.modifyCalibration([](fw::CalibrationData& data) { data.imu.gravity = 9.7; });
        }),
        resources);
    runtime.init();

    comm::Publisher<fw::SensorPacket> packets(
        runtime.core().topic<fw::SensorPacket>(modules::topics::kSensorPacket),
        comm::PublishOptions{false, true});
    fw::SensorPacket first;
    first.sequence = 1;
    first.reference_time = fw::Timestamp{100ms};
    packets.publish(first);
    settle(runtime);

    const auto frame = resources.keyframes.snapshot()->find(fw::KeyframeId{1});
    REQUIRE(frame != nullptr);
    CHECK(odometry.algorithm().lastResult().calibration_revision == 1);
    CHECK(frame->calibration_revision == 1);  // 不是帧处理中途换上的 2

    // 下一帧才切换
    fw::SensorPacket second;
    second.sequence = 2;
    second.reference_time = fw::Timestamp{200ms};
    packets.publish(second);
    settle(runtime);
    CHECK(odometry.algorithm().lastResult().calibration_revision == 2);
}

TEST_CASE("多线程并发改标定：不丢更新，版本与资源一致", "[acceptance][calibration]") {
    modules::SlamResources resources;
    resources.setCalibration(validCalibration());
    constexpr int kRounds = 100;
    std::thread a([&] {
        for (int i = 0; i < kRounds; ++i) {
            resources.modifyCalibration([](fw::CalibrationData& d) { d.imu.gravity += 0.001; });
        }
    });
    std::thread b([&] {
        for (int i = 0; i < kRounds; ++i) {
            resources.modifyCalibration([](fw::CalibrationData& d) { d.imu.rate_hz += 1.0; });
        }
    });
    a.join();
    b.join();
    const auto snapshot = resources.calibrationSnapshot();
    CHECK(snapshot->imu.gravity == Approx(9.81 + kRounds * 0.001));
    CHECK(snapshot->imu.rate_hz == Approx(200.0 + kRounds));
    CHECK(snapshot->revision == 1 + 2 * kRounds);
    CHECK(resources.calibration.version() == snapshot->revision);
}

TEST_CASE("坏的标定进不了真源", "[acceptance][calibration]") {
    modules::SlamResources resources;
    auto bad = validCalibration();
    bad.imu.gravity = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS(resources.setCalibration(bad));
    CHECK(resources.calibrationRevision() == 0);

    for (const char* yaml : {"extrinsics:\n  lidar: {}\n",
                             "imu:\n  gravity: .nan\n",
                             "imu:\n  rate_hz: -1\n",
                             "lidars:\n  lidar: {min_range_m: 100, max_range_m: 1}\n"}) {
        CHECK_THROWS_AS(cfg::loadCalibrationString(yaml), cfg::CalibrationError);
    }
}

TEST_CASE("很小的已知协方差也能无损往返", "[acceptance][calibration]") {
    auto without = validCalibration();
    auto with = validCalibration();
    with.extrinsics["lidar"].covariance_known = true;
    with.extrinsics["lidar"].covariance(0, 0) = 1e-14;
    CHECK(cfg::dumpCalibration(with) != cfg::dumpCalibration(without));
    const auto roundtrip = cfg::loadCalibrationString(cfg::dumpCalibration(with));
    CHECK(roundtrip.extrinsics.at("lidar").covariance_known);
    CHECK(roundtrip.extrinsics.at("lidar").covariance(0, 0) == 1e-14);
}

TEST_CASE("必需外参由装配核对：外部标定补上即可", "[acceptance][calibration]") {
    struct NeedsLidar final : fw::ISensorSource {
        std::optional<fw::SensorDatum> next() override { return std::nullopt; }
        fw::SourceDescriptor descriptor() const override {
            fw::SourceDescriptor d;
            d.declared = true;
            d.name = "needs-lidar";
            d.required_extrinsics = {"lidar"};
            return d;
        }
    };
    fw::PacketizerPolicy policy;
    policy.imu = fw::ImuRequirement::kNotRequired;
    {
        modules::SlamSystem system;
        system.setSource(std::make_unique<NeedsLidar>(), policy);
        system.setOdometry<HookOdometry>();
        CHECK_THROWS_AS(system.init(), comm::WiringError);  // 没人提供这条外参
    }
    {
        modules::SlamSystem system;
        system.setCalibration(validCalibration());  // 外部标定补上了
        system.setSource(std::make_unique<NeedsLidar>(), policy);
        system.setOdometry<HookOdometry>();
        CHECK_NOTHROW(system.init());
    }
}

TEST_CASE("来源没有惯性流而策略要求 IMU：装配期失败", "[acceptance][calibration]") {
    struct NoImu final : fw::ISensorSource {
        std::optional<fw::SensorDatum> next() override { return std::nullopt; }
        fw::SourceDescriptor descriptor() const override {
            fw::SourceDescriptor d;
            d.declared = true;
            d.has_imu = false;
            return d;
        }
    };
    modules::SlamSystem system;
    CHECK_THROWS_AS(system.setSource(std::make_unique<NoImu>(), fw::PacketizerPolicy{}),
                    comm::WiringError);
}

TEST_CASE("求解回写只采纳位姿，固定节点不许动", "[acceptance][graph]") {
    fw::PoseGraphData graph;
    fw::PoseGraphNode a;
    a.id = fw::KeyframeId{1};
    a.fixed = true;
    a.time = fw::Timestamp{1ns};
    graph.addNode(a);
    auto problem = graph.toProblem();
    problem.nodes[0].fixed = false;
    problem.nodes[0].time = fw::Timestamp{2ns};
    graph.adoptSolution(problem);  // 只采纳位姿：其余字段是图的事实
    CHECK(graph.nodes.at(0).fixed);
    CHECK(graph.nodes.at(0).time == fw::Timestamp{1ns});

    auto moved = graph.toProblem();
    moved.nodes[0].T_world_body = SE3d(Vec3d(1.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
    CHECK_THROWS_AS(graph.adoptSolution(moved), std::logic_error);  // 固定节点不许动
}

TEST_CASE("信息矩阵要真的合法，不只是有限", "[acceptance][calibration]") {
    Mat6d information = Mat6d::Identity();
    CHECK(fw::isValidInformation(information));
    CHECK(fw::isValidInformation(Mat6d::Zero()));  // 退化方向允许零信息
    Mat6d asymmetric = Mat6d::Identity();
    asymmetric(0, 1) = 5.0;
    CHECK_FALSE(fw::isValidInformation(asymmetric));
    Mat6d indefinite = Mat6d::Identity();
    indefinite(2, 2) = -1.0;
    CHECK_FALSE(fw::isValidInformation(indefinite));
}

TEST_CASE("去畸变：已补偿的原样保留，轨迹不覆盖的不外推", "[acceptance][deskew]") {
    fw::PointCloud points;
    points.push(1.0F, 0.0F, 0.0F);
    points.time_offset_ns.push_back(-50'000'000);
    SE3d::Tangent twist;
    twist.coeffs() << 1.0, 0.0, 0.0, 0.0, 0.0, 0.5;

    fw::DeskewContext context;
    context.reference_time = fw::Timestamp{1s};
    context.calibration_revision = 7;
    context.sensor = "lidar";
    context.motion_compensation = fw::MotionCompensation::kCompensated;
    auto [kept, compensated] = fw::deskew(points, SE3d::Identity(), fw::ConstantTwist(twist),
                                          context);
    CHECK(compensated.status == fw::DeskewStatus::kAlreadyCompensated);
    CHECK(compensated.compensated());
    CHECK(kept.x[0] == 1.0F);
    CHECK(compensated.reference_time == fw::Timestamp{1s});
    CHECK(compensated.calibration_revision == 7);
    CHECK(compensated.output_frame == "lidar");

    context.motion_compensation = fw::MotionCompensation::kRaw;
    context.trajectory_coverage = fw::TimeRange{fw::Timestamp{-10ms}, fw::Timestamp{0}};
    auto [untouched, uncovered] =
        fw::deskew(points, SE3d::Identity(), fw::ConstantTwist(twist), context);
    CHECK(uncovered.status == fw::DeskewStatus::kTrajectoryNotCovered);
    CHECK(untouched.x[0] == 1.0F);
}

TEST_CASE("图里混入不同标定的节点就不再产出校正", "[acceptance][graph]") {
    // 一张图只属于一份标定：换代之后再往里加节点，解对应的就不是任何一份标定了
    modules::SlamResources resources;
    comm::Runtime runtime(comm::RuntimeOptions{true});
    auto keyframe_writer = resources.keyframes.acquireWriteToken();
    auto optimizer = std::make_unique<IdleOptimizer>();
    auto* idle = optimizer.get();
    defineModuleParams(runtime);
    auto& graph_module = runtime.addModule<modules::GraphOptimizerModule>(
        "/graph", std::move(optimizer), resources);
    runtime.init();

    comm::Publisher<fw::KeyframeEvent> notify(
        runtime.core().topic<fw::KeyframeEvent>(modules::topics::kKeyframe), comm::PublishOptions{});
    comm::Publisher<fw::LoopConstraint> loops(
        runtime.core().topic<fw::LoopConstraint>(modules::topics::kLoopConstraint),
        comm::PublishOptions{});
    const std::uint64_t revisions[] = {1, 1, 2};
    for (std::uint64_t i = 1; i <= 3; ++i) {
        keyframe_writer.modify([&](fw::KeyframeStoreData& store) {
            auto frame = std::make_shared<fw::Keyframe>();
            frame->id = fw::KeyframeId{i};
            frame->sequence = i;
            frame->previous = fw::KeyframeId{i - 1};  // 接在前一帧后：同一段里程计
            frame->time = fw::Timestamp{static_cast<std::int64_t>(i) * 100'000'000};
            frame->calibration_revision = revisions[i - 1];
            store.append(frame);
        });
        notify.publish(fw::KeyframeEvent{});
        settle(runtime);
    }
    REQUIRE(resources.graph.snapshot()->calibration_mixed);

    fw::LoopConstraint loop;
    loop.id = fw::ConstraintId{1};
    loop.query = fw::KeyframeId{3};
    loop.match = fw::KeyframeId{1};
    loops.publish(loop);
    settle(runtime);

    CHECK(graph_module.committedConstraints() == 1);          // 约束照样进真源
    CHECK(graph_module.skippedForMixedCalibration() == 1);    // 但不求解、不发校正
    CHECK(idle->calls == 0);
    CHECK(graph_module.optimizations() == 0);
}

TEST_CASE("插值关联把不同步的真值对上，最近邻对不上", "[acceptance][eval]") {
    // 估计在扫描终点（整 100 ms），真值在中段（+50 ms）：最近邻 20 ms 容差一对都配不上，
    // 放宽容差是拿相邻时刻冒充同一时刻；插值才是显式换算
    fw::Trajectory estimate;
    fw::Trajectory truth;
    for (int i = 0; i < 20; ++i) {
        const double x = 0.5 * i;
        estimate.push(fw::Timestamp{i * 100ms}, SE3d(Vec3d(x, 0.0, 0.0), Eigen::Quaterniond::Identity()));
        truth.push(fw::Timestamp{i * 100ms + 50ms},
                   SE3d(Vec3d(x + 0.25, 0.0, 0.0), Eigen::Quaterniond::Identity()));
    }
    eval::AteOptions nearest;
    nearest.alignment = eval::AlignmentKind::kNone;
    CHECK(eval::absoluteTrajectoryError(estimate, truth, nearest).pairs == 0);

    eval::AteOptions interpolate = nearest;
    interpolate.association = eval::AssociationMode::kInterpolate;
    interpolate.max_interpolation_gap = 150ms;
    const auto ate = eval::absoluteTrajectoryError(estimate, truth, interpolate);
    CHECK(ate.pairs >= 18);
    CHECK(ate.translation.rmse == Approx(0.0).margin(1e-9));  // 匀速直线：插值精确
    CHECK(ate.referenceCoverage() <= 1.0);
}

TEST_CASE("跟踪的传感器不同就拒绝比较，显式转换之后才能比", "[acceptance][eval]") {
    fw::Trajectory body;
    fw::Trajectory camera;
    body.setTracked("body");
    camera.setTracked("cam0");
    for (int i = 0; i < 5; ++i) {
        body.push(fw::Timestamp{i * 100ms}, SE3d::Identity());
        camera.push(fw::Timestamp{i * 100ms}, SE3d::Identity());
    }
    CHECK_THROWS_AS(eval::absoluteTrajectoryError(body, camera, eval::AteOptions{}),
                    fw::TrajectoryError);
    const auto converted = eval::rightMultiplied(body, SE3d::Identity(), "cam0");
    CHECK_NOTHROW(eval::absoluteTrajectoryError(converted, camera, eval::AteOptions{}));
    // Sim3 对齐的尺度部分同样带着跟踪的传感器：缩放之后照样拒绝
    CHECK(body.scaled(2.0).tracked() == "body");
    CHECK_THROWS_AS(eval::absoluteTrajectoryError(body.scaled(2.0), camera, eval::AteOptions{}),
                    fw::TrajectoryError);
}

TEST_CASE("协方差有值却没声明为已知：拒绝导出，不静默丢", "[acceptance][calibration]") {
    auto calibration = validCalibration();
    calibration.extrinsics["lidar"].covariance(0, 0) = 1e-14;  // 没有声明 covariance_known
    CHECK_THROWS_AS(cfg::dumpCalibration(calibration), cfg::CalibrationError);
}
