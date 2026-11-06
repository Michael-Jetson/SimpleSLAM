// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 相邻的状态转换：两件事紧挨着发生时，终态证据、寿命与结论是否仍然成立。
///
/// 一个局部 API 的错误输入只能说明那个函数；这里测的是**顺序**——算法出错之后又成功、
/// 服务受理之后服务端关闭、末条通知丢失之后再没有业务帧、读完之后终结再查结果、
/// 一条 IMU 同时释放好几帧、报告里的协议转存成基线。每个用例断言的都是结论
/// 该有的样子，而不是"多了一个字段"。

#include <SimpleSLAM/app/bindings.hpp>
#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/eval/baseline.hpp>
#include <SimpleSLAM/eval/evaluators.hpp>
#include <SimpleSLAM/eval/metrics.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>
#include <SimpleSLAM/framework/deskew.hpp>
#include <SimpleSLAM/modules/slam_system.hpp>
#include <SimpleSLAM/record/diagnostic_file_sink.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <typeindex>
#include <variant>
#include <vector>

using namespace simpleslam;
using Catch::Approx;
using namespace std::chrono_literals;

namespace comm = simpleslam::comm;
namespace fw = simpleslam::fw;
namespace modules = simpleslam::modules;
namespace plugin = simpleslam::plugin;
namespace eval = simpleslam::eval;

namespace {

/// 按给定顺序吐出数据的来源。
class ListSource final : public fw::ISensorSource {
public:
    explicit ListSource(std::vector<fw::SensorDatum> data) : data_(std::move(data)) {}
    [[nodiscard]] std::optional<fw::SensorDatum> next() override {
        if (index_ >= data_.size()) return std::nullopt;
        return data_[index_++];
    }

private:
    std::vector<fw::SensorDatum> data_;
    std::size_t index_ = 0;
};

fw::SensorDatum scanAt(fw::Timestamp t) {
    auto scan = std::make_shared<fw::LidarScan>();
    scan->reference_time = t;
    scan->coverage = fw::TimeRange{t - 100ms, t};
    scan->points.push(1.0F, 0.0F, 0.0F);
    return fw::Ptr<fw::LidarScan>(scan);
}

fw::SensorDatum imuAt(fw::Timestamp t) {
    auto sample = std::make_shared<fw::ImuSample>();
    sample->time = t;
    return fw::Ptr<fw::ImuSample>(sample);
}

std::vector<fw::SensorDatum> scans(int count) {
    std::vector<fw::SensorDatum> data;
    for (int i = 1; i <= count; ++i) data.push_back(scanAt(fw::Timestamp{i * 100ms}));
    return data;
}

fw::PacketizerPolicy lidarOnly() {
    fw::PacketizerPolicy policy;
    policy.imu = fw::ImuRequirement::kNotRequired;
    return policy;
}

/// 每帧前进 1 米；fail_at 非零时在那一帧抛异常。
class StepOdometry final : public plugin::OdometryBase {
public:
    explicit StepOdometry(int fail_at = 0) : fail_at_(fail_at) {}
    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }

protected:
    plugin::TrackResult track(const fw::SensorPacket&) override {
        ++frames_;
        if (frames_ == fail_at_) throw std::runtime_error("这一帧数值失败");
        plugin::TrackResult result;
        result.status = fw::TrackingStatus::kTracking;
        result.T_odom_body = SE3d(Vec3d(static_cast<double>(frames_), 0.0, 0.0),
                                  Eigen::Quaterniond::Identity());
        return result;
    }

private:
    int fail_at_;
    int frames_ = 0;
};

/// 缺支撑政策与连续性上限可配的算法：直接调框架骨架，看每个包落在哪个终态。
class PolicyOdometry final : public plugin::OdometryBase {
public:
    explicit PolicyOdometry(plugin::SupportGapPolicy policy,
                            std::optional<fw::Duration> limit = std::nullopt)
        : policy_(policy), limit_(limit) {}
    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override { return policy_; }
    [[nodiscard]] std::optional<fw::Duration> maxImuGap() const override { return limit_; }

    int tracked = 0;

protected:
    plugin::TrackResult track(const fw::SensorPacket&) override {
        ++tracked;
        plugin::TrackResult result;
        result.status = fw::TrackingStatus::kTracking;  // 算法自己说"正常"
        return result;
    }

private:
    plugin::SupportGapPolicy policy_;
    std::optional<fw::Duration> limit_;
};

fw::SensorPacket packetAt(fw::Timestamp t, bool gap) {
    fw::SensorPacket packet;
    packet.reference_time = t;
    packet.imu_coverage = fw::TimeRange{t, t};  // 没有样本：声明区间只能是空的
    packet.gap = gap ? fw::GapReason::kImuGap : fw::GapReason::kNone;
    return packet;
}

struct Ticker final : comm::Module {
    int ticks = 0;
    comm::TimerHandle timer;
    void onInit() override { timer = createTimer(100ms, [this] { ++ticks; }); }
};

/// 定时器回调里抛异常、谁也没处理：执行边界接住它，但这份工作确实丢了。
struct ThrowingTicker final : comm::Module {
    int ticks = 0;
    void onInit() override {
        (void)createTimer(100ms, [this] {
            ++ticks;
            throw std::runtime_error("定时器回调炸了");
        });
    }
};

struct Provider final : comm::Module {
    int handle(const int& n) { return n + 1; }
    void onInit() override { advertiseService<int, int>("/svc", &Provider::handle); }
};

struct Client final : comm::Module {
    comm::ServiceClient<int, int> cli;
    void onInit() override { cli = serviceClient<int, int>("/svc"); }
};

struct ParamWatcher final : comm::Module {
    std::atomic<std::uint64_t> revision{0};
    void onParamsChanged(const comm::ParamSnapshot& snapshot) override { revision = snapshot.revision; }
};

/// 第 fail_on 次求解报数值失败，其余照常收敛。
struct FlakyOptimizer final : plugin::GraphOptimizerBase {
    explicit FlakyOptimizer(int fail) : fail_on(fail) {}
    Report onOptimize(fw::GraphProblem&) override {
        ++calls;
        Report report;
        if (calls == fail_on) report.status = Status::kNumericFailure;
        return report;
    }
    int fail_on;
    int calls = 0;
};

void settle(comm::Runtime& runtime) {
    while (runtime.spinOnce() != 0) {
    }
}

void appendKeyframe(fw::WriteToken<fw::KeyframeStoreData>& writer, std::uint64_t id) {
    writer.modify([&](fw::KeyframeStoreData& store) {
        auto frame = std::make_shared<fw::Keyframe>();
        frame->id = fw::KeyframeId{id};
        frame->sequence = id;
        frame->previous = fw::KeyframeId{id - 1};  // 首帧为 0（没有前驱），其余接在前一帧后
        frame->time = fw::Timestamp{static_cast<std::int64_t>(id) * 100'000'000};
        frame->calibration_revision = 1;
        store.append(frame);
    });
}

/// 评测函数的输入：两条现成的轨迹与一个目标窗口。
class FixedInputs final : public eval::EvaluationInputs {
public:
    FixedInputs(fw::Trajectory estimate, fw::Trajectory reference,
                std::optional<fw::TimeRange> window)
        : estimate_(std::move(estimate)), reference_(std::move(reference)), window_(window) {}
    [[nodiscard]] eval::ComparablePair pair(std::string_view /*estimate*/) const override {
        return {.estimate = estimate_, .reference = reference_, .notes = {}};
    }
    [[nodiscard]] std::optional<fw::TimeRange> targetWindow() const override { return window_; }

private:
    fw::Trajectory estimate_;
    fw::Trajectory reference_;
    std::optional<fw::TimeRange> window_;
};

const eval::Scalar* scalarNamed(const eval::Evaluation& evaluation, const std::string& name) {
    for (const auto& scalar : evaluation.scalars) {
        if (scalar.name == name) return &scalar;
    }
    return nullptr;
}

}  // namespace

// ─────────────────────────── 运行结局与终态证据 ───────────────────────────

TEST_CASE("算法在中间一帧失败、之后又成功：那一帧有失败终态，运行不算完整",
          "[transitions][outcome]") {
    modules::SlamSystem system;
    system.setSource(std::make_unique<ListSource>(scans(3)), lidarOnly());
    auto& odometry = system.setOdometry<StepOdometry>(2);
    system.init();
    CHECK(system.runOffline() == 3);
    system.shutdown();

    const auto outcome = system.runOutcome();
    CHECK(odometry.receivedPackets() == 3);  // 三包都有了下落
    CHECK(odometry.processedFrames() == 2);
    CHECK(odometry.failedFrames() == 1);
    CHECK(outcome.failed_frames == 1);
    CHECK(outcome.packets_without_outcome == 0);
    CHECK_FALSE(outcome.processingComplete());
    CHECK_FALSE(outcome.complete());
    // 失败帧同样报告了进度：在途清零说明每包都有终态，不说明每包都成功
    CHECK(system.sensorIo()->packetizer().inFlight() == 0);
    CHECK(system.trajectory()->odometry().size() == 2);

    eval::GateRequest request;
    request.metrics = {{"ate.samples", 2.0}, {"ate.translation.rmse", 0.0}};
    request.completeness =
        outcome.complete() ? eval::RunCompleteness::kComplete : eval::RunCompleteness::kIncomplete;
    CHECK(eval::gate(request).outcome == eval::BaselineOutcome::kInvalid);
}

TEST_CASE("回调里没人处理的异常：执行边界接住它，运行照样不算完整", "[transitions][outcome]") {
    modules::SlamSystem system;
    auto& ticker = system.runtime().addModule<ThrowingTicker>("/ticker");
    system.setSource(std::make_unique<ListSource>(scans(5)), lidarOnly());
    system.setOdometry<StepOdometry>();
    system.init();
    system.runOffline();
    system.shutdown();

    const auto outcome = system.runOutcome();
    REQUIRE(ticker.ticks > 0);
    CHECK(outcome.isolated_failures == static_cast<std::uint64_t>(ticker.ticks));
    CHECK_FALSE(outcome.processingComplete());
}

TEST_CASE("读完之后终结再查结果：结局固定，模块可读，步进与写入都被拒绝",
          "[transitions][lifecycle]") {
    modules::SlamSystem system;
    auto& io = system.setSource(std::make_unique<ListSource>(scans(3)), lidarOnly());
    system.setOdometry<StepOdometry>();
    system.init();
    system.runOffline();
    CHECK_FALSE(system.runOutcome().finalized);  // 关闭之前读到的只是当前快照

    system.shutdown();
    const auto final_outcome = system.runOutcome();
    CHECK(final_outcome.finalized);
    CHECK(final_outcome.shutdown_drained);
    CHECK(final_outcome.complete());
    const auto poses = system.trajectory()->odometry().size();
    CHECK(poses == 3);

    // 终结之后：步进、泵数据、改标定、驱动运行时一律不生效
    CHECK(system.runOffline() == 0);
    CHECK(system.settle() == 0);
    CHECK(io.pump(8).samples == 0);
    CHECK_THROWS_AS(system.setCalibration(fw::CalibrationData{}), std::logic_error);
    CHECK(system.runtime().spinOnce() == 0);
    CHECK(system.trajectory()->odometry().size() == poses);
    CHECK(system.runOutcome().reasons() == final_outcome.reasons());

    system.shutdown();  // 再次终结是空操作
    CHECK(system.runOutcome().finalized);
}

// ─────────────────────────── 离线时间与单步输入 ───────────────────────────

TEST_CASE("只有惯性数据、没有触发帧：离线时钟照样跟着数据时间走", "[transitions][offline]") {
    std::vector<fw::SensorDatum> data;
    for (int i = 0; i <= 100; ++i) data.push_back(imuAt(fw::Timestamp{i * 10ms}));
    modules::SlamSystem system;
    auto& ticker = system.runtime().addModule<Ticker>("/ticker");
    system.setSource(std::make_unique<ListSource>(std::move(data)));
    system.setOdometry<StepOdometry>();
    system.init();
    system.runOffline();

    CHECK(system.runtime().clock().now() == comm::Time{1s});  // 最后一条样本的时刻
    CHECK(ticker.ticks >= 1);  // 等不到触发帧时，定时器照样拿得到数据时间
}

TEST_CASE("一条 IMU 同时让四帧凑齐括点：单步只发预算允许的那一帧", "[transitions][offline]") {
    std::vector<fw::SensorDatum> data{imuAt(fw::Timestamp{0})};
    for (auto& scan : scans(4)) data.push_back(scan);
    data.push_back(imuAt(fw::Timestamp{1s}));

    modules::SlamSystem system;
    auto& io = system.setSource(std::make_unique<ListSource>(data));
    system.setOdometry<StepOdometry>();
    system.init();

    auto step = io.pump(1);
    CHECK(step.published == 1);
    CHECK(io.packetizer().pendingTriggers() == 3);  // 其余三帧留在打包器里
    system.settle();

    step = io.pump(1);
    CHECK(step.published == 1);
    CHECK(step.samples == 0);  // 先发已就绪的，不读来源

    modules::SlamSystem limited;
    limited.setSource(std::make_unique<ListSource>(data));
    limited.setOdometry<StepOdometry>();
    limited.init();
    CHECK(limited.runOffline(1) == 1);
}

TEST_CASE("来源读完时还有好几帧在等括点：排空同样逐步进行", "[transitions][offline]") {
    std::vector<fw::SensorDatum> data{imuAt(fw::Timestamp{0})};
    for (auto& scan : scans(3)) data.push_back(scan);  // 之后再没有 IMU

    modules::SlamSystem system;
    auto& io = system.setSource(std::make_unique<ListSource>(data));
    system.setOdometry<StepOdometry>();
    system.init();

    auto step = io.pump(1);
    CHECK(io.sourceExhausted());
    CHECK(step.published == 1);
    CHECK_FALSE(step.finished);  // 还有两帧没发
    system.settle();
    step = io.pump(1);
    CHECK(step.published == 1);
    CHECK(step.samples == 0);
    CHECK_FALSE(step.finished);
    system.settle();
    step = io.pump(1);
    CHECK(step.published == 1);
    CHECK(step.finished);  // 排空完才算结束
}

// ─────────────────────────── 未决操作能否结案 ───────────────────────────

TEST_CASE("服务受理之后、执行之前服务端收件箱关闭：调用照样结案",
          "[transitions][service]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    auto& provider = runtime.addModule<Provider>("/p");
    auto& client = runtime.addModule<Client>("/c");
    runtime.init();

    std::vector<comm::ServiceStatus> results;
    const auto status =
        client.cli.asyncCall(1, 0ns, [&](const auto& result) { results.push_back(result.status); });
    REQUIRE(status == comm::ServiceStatus::kOk);
    provider.mailbox()->close();  // 排队中的请求随之被丢弃
    settle(runtime);

    REQUIRE(results.size() == 1);
    CHECK(results.front() == comm::ServiceStatus::kStopped);
}

TEST_CASE("阻塞调用等待期间服务端收件箱关闭：立即得到终态，不会等到超时",
          "[transitions][service]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    auto& provider = runtime.addModule<Provider>("/p");
    auto& client = runtime.addModule<Client>("/c");
    runtime.init();

    std::atomic<int> status{-1};
    const auto started = std::chrono::steady_clock::now();
    std::thread caller([&] {
        const auto result = client.cli.call(1, 10s);
        status.store(static_cast<int>(result.status));
    });
    // 等请求排进服务端收件箱，再把它关掉
    while (provider.mailbox()->stats().depth == 0) std::this_thread::yield();
    provider.mailbox()->close();
    caller.join();

    CHECK(status.load() == static_cast<int>(comm::ServiceStatus::kStopped));
    CHECK(std::chrono::steady_clock::now() - started < 5s);
}

// ─────────────────────────── 参数的停止边界与在线查询 ───────────────────────────

TEST_CASE("requestStop 生效起参数写入被拒；此前受理的提交在关闭排空时生效",
          "[transitions][params]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    runtime.params().preset("/x", std::int64_t{1});
    auto& watcher = runtime.addModule<ParamWatcher>("/m");
    runtime.init();

    const auto accepted = runtime.setParams({{"/x", comm::ParamValue{std::int64_t{2}}}});
    REQUIRE(accepted.ok());
    CHECK(watcher.revision.load() == 0);  // 只排进了收件箱

    runtime.requestStop();
    const auto late = runtime.setParams({{"/x", comm::ParamValue{std::int64_t{3}}}});
    CHECK(late.status == comm::ParamCommit::kRejectedStopped);
    CHECK(runtime.params().snapshot()->revision == accepted.revision);  // 存储不变

    runtime.shutdown();
    CHECK(watcher.revision.load() == accepted.revision);  // 按关闭协议排空时生效
}

TEST_CASE("在线运行中并发提交参数：快照按版本顺序换上，最后一代终会生效", "[transitions][params]") {
    comm::Runtime runtime(comm::RuntimeOptions{false});
    runtime.params().preset("/x", std::int64_t{0});
    auto& watcher = runtime.addModule<ParamWatcher>("/m");
    runtime.init();
    runtime.start();

    std::uint64_t last = 0;
    bool all_ok = true;
    for (int i = 1; i <= 200; ++i) {
        const auto result = runtime.setParams({{"/x", comm::ParamValue{std::int64_t{i}}}});
        all_ok = all_ok && result.ok();
        last = result.revision;
    }
    CHECK(runtime.waitIdle(5s));
    runtime.shutdown();

    CHECK(all_ok);
    CHECK(watcher.revision.load() == last);
}

// ─────────────────────────── 产物与评测协议 ───────────────────────────

TEST_CASE("评测函数的协议转存为基线，读回来每个字段都一样", "[transitions][eval]") {
    const auto path =
        (std::filesystem::temp_directory_path() / "simpleslam_transitions_baseline.yaml").string();
    const eval::Protocol protocol{{"type", "ate"},
                                  {"alignment", "se3"},
                                  {"association", "interpolate"},
                                  {"tolerance_s", "0.02"},
                                  {"max_interpolation_gap_s", "0.15"},
                                  {"estimate", "odometry"},
                                  // 长数字原样往返：协议是文本，不经过浮点
                                  {"window_begin_s", "1317346945.123456789"}};
    eval::DatasetBaseline baseline;
    baseline.dataset = "raw/0027";
    baseline.bounds.push_back({"ate.translation.rmse", 1.0});
    baseline.evaluators["ate"] = protocol;
    eval::writeBaselines(path, {baseline});
    const auto loaded = eval::loadBaselines(path);
    REQUIRE(loaded.size() == 1);
    CHECK(loaded.front().evaluators.at("ate") == protocol);
    std::filesystem::remove(path);
}

TEST_CASE("任务完成度按目标窗口算：括点支撑没被用到，不是任务没完成", "[transitions][eval]") {
    // 真值在导航流的相位上（+50 ms），两侧各多一个括点；估计在四个扫描参考时刻
    fw::Trajectory truth;
    for (int i = 0; i <= 5; ++i) {
        const double seconds = 0.05 + 0.1 * i;
        truth.push(fw::Timestamp{50ms + i * 100ms},
                   SE3d(Vec3d(seconds, 0.0, 0.0), Eigen::Quaterniond::Identity()));
    }
    fw::Trajectory estimate;
    for (int i = 1; i <= 4; ++i) {
        estimate.push(fw::Timestamp{i * 100ms},
                      SE3d(Vec3d(0.1 * i, 0.0, 0.0), Eigen::Quaterniond::Identity()));
    }

    eval::AteOptions options;
    options.alignment = eval::AlignmentKind::kNone;
    options.association = eval::AssociationMode::kInterpolate;
    options.max_interpolation_gap = 150ms;
    const fw::TimeRange window{fw::Timestamp{100ms}, fw::Timestamp{400ms}};
    options.target_window = window;
    const auto ate = eval::absoluteTrajectoryError(estimate, truth, options);
    CHECK(ate.pairs == 4);
    CHECK(ate.translation.rmse == Approx(0.0).margin(1e-9));
    CHECK(ate.targetCoverage() == Approx(1.0));  // 目标帧全部完成
    CHECK(ate.referenceCoverage() < 1.0);         // 支撑样本没用满
    CHECK(ate.spanRatio() < 1.0);                 // 参考跨度比任务长

    // 评测函数同样只在声明了目标时给任务完成度，不拿参考跨度冒充
    eval::AteEvaluator::Config config;
    config.alignment = "none";
    config.association = "interpolate";
    const eval::AteEvaluator evaluator(config);
    const auto declared = evaluator.evaluate(FixedInputs(estimate, truth, window));
    REQUIRE(scalarNamed(declared, "target_coverage") != nullptr);
    CHECK(scalarNamed(declared, "target_coverage")->value == Approx(1.0));
    const auto undeclared = evaluator.evaluate(FixedInputs(estimate, truth, std::nullopt));
    CHECK(scalarNamed(undeclared, "target_coverage") == nullptr);
}

TEST_CASE("门禁：完整性未知时没有基线照常测量并注明，有基线不可比较；人工声明要留痕",
          "[transitions][gate]") {
    eval::DatasetBaseline baseline;
    baseline.dataset = "kitti/00";
    baseline.bounds.push_back({"ate.translation.rmse", 1.0});
    const std::vector<eval::DatasetBaseline> baselines{baseline};

    eval::GateRequest request;
    request.metrics = {{"ate.samples", 30.0}, {"ate.translation.rmse", 0.05}};
    request.dataset = "kitti/00";
    CHECK(request.completeness == eval::RunCompleteness::kUnknown);  // 没有证据就不假定完整

    auto verdict = eval::gate(request);
    CHECK(verdict.outcome == eval::BaselineOutcome::kPass);
    CHECK(verdict.detail.find("未知") != std::string::npos);

    request.baselines = &baselines;
    CHECK(eval::gate(request).outcome == eval::BaselineOutcome::kIncomparable);

    request.completeness = eval::RunCompleteness::kDeclaredComplete;
    verdict = eval::gate(request);
    CHECK(verdict.outcome == eval::BaselineOutcome::kPass);
    CHECK(verdict.detail.find("人工声明") != std::string::npos);

    request.completeness = eval::RunCompleteness::kComplete;
    CHECK(eval::gate(request).outcome == eval::BaselineOutcome::kPass);
}

// ─────────────────────────── 支撑需求与缺支撑的包 ───────────────────────────

TEST_CASE("首帧的支撑需求从扫描起点算：前半段没有 IMU 就标缺口", "[transitions][packetizer]") {
    fw::Packetizer packetizer;
    (void)packetizer.feed(imuAt(fw::Timestamp{50ms}));
    (void)packetizer.feed(scanAt(fw::Timestamp{100ms}));  // 扫描覆盖 [0, 100 ms]，等右括点
    const auto packet = packetizer.feed(imuAt(fw::Timestamp{150ms}));
    REQUIRE(packet.has_value());
    CHECK(packet->imu_required.begin == fw::Timestamp{0});      // 需求由扫描决定
    CHECK(packet->imu_coverage.begin == fw::Timestamp{50ms});   // 实际支撑从第一条样本起
    CHECK(packet->gap == fw::GapReason::kImuGap);
    CHECK_FALSE(packet->supportComplete());
}

TEST_CASE("缺支撑的包按算法声明的政策处理，结果不会以完整包的面目出现",
          "[transitions][odometry]") {
    const fw::FrameContext context;

    SECTION("初始化之前等待") {
        PolicyOdometry odometry(plugin::SupportGapPolicy::kWaitForInitialization);
        auto outcome = odometry.process(packetAt(fw::Timestamp{100ms}, true), context);
        CHECK(outcome.disposition == plugin::FrameDisposition::kAwaitingSupport);
        CHECK(odometry.tracked == 0);

        outcome = odometry.process(packetAt(fw::Timestamp{200ms}, false), context);
        CHECK(outcome.disposition == plugin::FrameDisposition::kTracked);
        CHECK(outcome.result.status == fw::TrackingStatus::kTracking);

        // 初始化之后的缺支撑包照常跟踪，但结论只到"降级"
        outcome = odometry.process(packetAt(fw::Timestamp{300ms}, true), context);
        CHECK(outcome.disposition == plugin::FrameDisposition::kTracked);
        CHECK(outcome.result.status == fw::TrackingStatus::kDegraded);
    }
    SECTION("降级") {
        PolicyOdometry odometry(plugin::SupportGapPolicy::kDegrade);
        const auto outcome = odometry.process(packetAt(fw::Timestamp{100ms}, true), context);
        CHECK(outcome.disposition == plugin::FrameDisposition::kTracked);
        CHECK(outcome.support_gap);
        CHECK(outcome.result.status == fw::TrackingStatus::kDegraded);
    }
    SECTION("拒绝") {
        PolicyOdometry odometry(plugin::SupportGapPolicy::kReject);
        auto outcome = odometry.process(packetAt(fw::Timestamp{100ms}, true), context);
        CHECK(outcome.disposition == plugin::FrameDisposition::kRejectedForGap);
        CHECK(odometry.tracked == 0);
        outcome = odometry.process(packetAt(fw::Timestamp{200ms}, false), context);
        CHECK(outcome.disposition == plugin::FrameDisposition::kTracked);
        CHECK(outcome.result.status == fw::TrackingStatus::kTracking);
    }
}

TEST_CASE("算法自己的连续性上限独立生效：来源阈值放行的包也可能缺支撑",
          "[transitions][odometry]") {
    PolicyOdometry odometry(plugin::SupportGapPolicy::kDegrade, fw::Duration{50ms});
    auto packet = packetAt(fw::Timestamp{100ms}, false);
    packet.imu_max_gap = 100ms;  // 来源的阈值放行了（gap 为空），但超出了算法的上限
    const auto outcome = odometry.process(packet, fw::FrameContext{});
    CHECK(outcome.support_gap);
    CHECK(outcome.result.status == fw::TrackingStatus::kDegraded);
}

TEST_CASE("去畸变：没有惯性支撑时覆盖区间退化成一点，不能因此跳过核对",
          "[transitions][deskew]") {
    fw::PointCloud points;
    points.push(1.0F, 0.0F, 0.0F);
    points.time_offset_ns.push_back(-50'000'000);
    SE3d::Tangent twist;
    twist.coeffs() << 1.0, 0.0, 0.0, 0.0, 0.0, 0.5;

    fw::DeskewContext context;
    context.reference_time = fw::Timestamp{1s};
    context.motion_compensation = fw::MotionCompensation::kRaw;
    context.trajectory_coverage = fw::TimeRange{fw::Timestamp{0}, fw::Timestamp{0}};
    auto [kept, result] = fw::deskew(points, SE3d::Identity(), fw::ConstantTwist(twist), context);
    CHECK(result.status == fw::DeskewStatus::kTrajectoryNotCovered);
    CHECK(kept.x[0] == 1.0F);  // 一个点也没动

    context.trajectory_coverage.reset();  // 调用方显式担保覆盖
    [[maybe_unused]] auto [moved, trusted] =
        fw::deskew(points, SE3d::Identity(), fw::ConstantTwist(twist), context);
    CHECK(trusted.status == fw::DeskewStatus::kCorrected);
}

// ─────────────────────────── 后端追平 ───────────────────────────

TEST_CASE("后端追平按输入版本判：新回环没进最后一次成功求解就是没追平，只追加里程计节点不算",
          "[transitions][graph]") {
    modules::SlamResources resources;
    comm::Runtime runtime(comm::RuntimeOptions{true});
    auto keyframe_writer = resources.keyframes.acquireWriteToken();
    auto& graph = runtime.addModule<modules::GraphOptimizerModule>(
        "/graph", std::make_unique<FlakyOptimizer>(2), resources);
    runtime.init();

    comm::Publisher<fw::KeyframeEvent> notify(
        runtime.core().topic<fw::KeyframeEvent>(modules::topics::kKeyframe), comm::PublishOptions{});
    comm::Publisher<fw::LoopConstraint> loops(
        runtime.core().topic<fw::LoopConstraint>(modules::topics::kLoopConstraint),
        comm::PublishOptions{});
    const auto addKeyframe = [&](std::uint64_t id) {
        appendKeyframe(keyframe_writer, id);
        notify.publish(fw::KeyframeEvent{});
        settle(runtime);
    };
    const auto addLoop = [&](std::uint64_t id, std::uint64_t query) {
        fw::LoopConstraint loop;
        loop.id = fw::ConstraintId{id};
        loop.query = fw::KeyframeId{query};
        loop.match = fw::KeyframeId{1};
        loops.publish(loop);
        settle(runtime);
    };

    for (std::uint64_t id = 1; id <= 3; ++id) addKeyframe(id);
    addLoop(1, 3);  // 第一次求解成功
    CHECK(graph.optimizations() == 1);
    CHECK(graph.unsolvedRequiredConstraints() == 0);

    addKeyframe(4);  // 只追加了里程计节点：由当前校正延拓，不要求重新优化
    CHECK(graph.unsolvedRequiredConstraints() == 0);

    addLoop(2, 4);  // 第二次求解失败：新回环不在最后一次成功的解里
    CHECK(graph.failedSolutions() == 1);
    CHECK(graph.unsolvedRequiredConstraints() == 1);
}

// ─────────────────────────── 句柄寿命与锁外的用户代码 ───────────────────────────

TEST_CASE("Runtime 销毁之后用旧句柄：调用、取消、发布都安全地失效", "[transitions][lifetime]") {
    comm::ServiceClient<int, int> kept;
    comm::TimerHandle timer;
    comm::Publisher<int> publisher;
    {
        comm::Runtime runtime(comm::RuntimeOptions{true});
        runtime.addModule<Provider>("/p");
        auto& client = runtime.addModule<Client>("/c");
        auto& ticker = runtime.addModule<Ticker>("/t");
        runtime.init();
        kept = client.cli;
        timer = ticker.timer;
        publisher = comm::Publisher<int>(runtime.core().topic<int>("/x"), comm::PublishOptions{});
    }
    int callbacks = 0;
    CHECK(kept.asyncCall(1, 1s, [&](auto) { ++callbacks; }) == comm::ServiceStatus::kStopped);
    CHECK(kept.call(1, 1s).status == comm::ServiceStatus::kStopped);
    CHECK(callbacks == 0);
    timer.cancel();
    CHECK_NOTHROW(publisher.publish(1));
}

TEST_CASE("没有绑定话题的发布端，发布是空操作", "[transitions][lifetime]") {
    comm::Publisher<int> unbound;
    CHECK_NOTHROW(unbound.publish(1));
    CHECK_FALSE(unbound.valid());
}

namespace {

struct Reentrant {
    std::function<void()> on_destroy;
    ~Reentrant() {
        if (on_destroy) on_destroy();
    }
};

struct BrokenShutdown final : comm::Module {
    void onShutdown() override { throw std::runtime_error("收尾钩子炸了"); }
};

}  // namespace

TEST_CASE("latch 值被顶掉时旧值在锁外析构：析构里再调话题不自锁", "[transitions][lifetime]") {
    auto topic = std::make_shared<comm::Topic>("/latched", std::type_index(typeid(Reentrant)),
                                               std::make_shared<comm::Hub>(), comm::ErrorReporter{}, 0);
    int reentered = 0;
    auto old = std::make_shared<Reentrant>();
    old->on_destroy = [weak = std::weak_ptr(topic), &reentered] {
        if (auto alive = weak.lock()) {
            (void)alive->subscriberCount();  // 在锁内析构的话，这里会自锁
            ++reentered;
        }
    };
    topic->publish(old, true);
    old.reset();
    topic->publish(std::make_shared<Reentrant>(), true);
    CHECK(reentered == 1);
}

TEST_CASE("收尾钩子抛异常：关闭照常完成，异常被计数", "[transitions][lifetime]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    runtime.addModule<BrokenShutdown>("/broken");
    runtime.init();
    CHECK_NOTHROW(runtime.shutdown());
    CHECK(runtime.state() == comm::RuntimeState::kDone);
    CHECK(runtime.isolatedFailures() == 1);
}

// ─────────────────────────── 校正生效与历史条件 ───────────────────────────

namespace {

/// 地图重建总是失败的算法：校正准备失败。
class RebuildFails final : public plugin::OdometryBase {
public:
    int rebuilds = 0;
    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }

protected:
    plugin::TrackResult track(const fw::SensorPacket&) override {
        plugin::TrackResult result;
        result.status = fw::TrackingStatus::kTracking;
        result.suggest_keyframe = true;
        return result;
    }
    void onCorrection(const SE3d&, const fw::PoseGraphData&) override {
        ++rebuilds;
        throw std::runtime_error("地图重建失败");
    }
};

/// 每帧都选关键帧、从不失败的算法。
class EveryFrameKeyframe final : public plugin::OdometryBase {
public:
    explicit EveryFrameKeyframe(std::vector<bool> keyframes = {}) : keyframes_(std::move(keyframes)) {}
    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }

protected:
    plugin::TrackResult track(const fw::SensorPacket&) override {
        plugin::TrackResult result;
        result.status = fw::TrackingStatus::kTracking;
        result.suggest_keyframe = keyframes_.empty() || (frame_ < keyframes_.size() && keyframes_[frame_]);
        ++frame_;
        return result;
    }

private:
    std::vector<bool> keyframes_;
    std::size_t frame_ = 0;
};

struct KeyframeObserver final : comm::Module {
    std::vector<std::uint64_t> sequences;
    void onInit() override {
        subscribe<fw::KeyframeEvent>(modules::topics::kKeyframe, 0, [this](const fw::KeyframeEvent& event) {
            sequences.push_back(event.sequence);
        });
    }
};

void publishPacket(comm::Publisher<fw::SensorPacket>& publisher, std::uint64_t sequence,
                   fw::Timestamp time) {
    fw::SensorPacket packet;
    packet.sequence = sequence;
    packet.reference_time = time;
    packet.imu_coverage = fw::TimeRange{time, time};
    publisher.publish(packet);
}

}  // namespace

TEST_CASE("校正准备失败：校正不生效，算法重新初始化，之后的帧不顶着新校正的身份",
          "[transitions][correction]") {
    modules::SlamResources resources;
    comm::Runtime runtime(comm::RuntimeOptions{true});
    auto algorithm = std::make_unique<RebuildFails>();
    auto* raw = algorithm.get();
    auto& odometry =
        runtime.addModule<modules::OdometryModule>("/odometry", std::move(algorithm), resources);
    runtime.init();

    comm::Publisher<fw::SensorPacket> packets(
        runtime.core().topic<fw::SensorPacket>(modules::topics::kSensorPacket),
        comm::PublishOptions{false, true});
    publishPacket(packets, 1, fw::Timestamp{100ms});
    settle(runtime);

    auto graph = std::make_shared<fw::PoseGraphData>();
    graph->identity.session = resources.session;
    graph->identity.solution_revision = 1;
    graph->identity.covered_until = 1;
    fw::PoseGraphNode node;
    node.id = fw::KeyframeId{1};
    node.T_world_body = SE3d(Vec3d(5.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
    graph->addNode(node);
    comm::Publisher<fw::CorrectionEvent> corrections(
        runtime.core().topic<fw::CorrectionEvent>(modules::topics::kCorrection),
        comm::PublishOptions{});
    corrections.publish(fw::CorrectionEvent{fw::Timestamp{100ms}, graph});
    settle(runtime);

    publishPacket(packets, 2, fw::Timestamp{200ms});
    settle(runtime);

    CHECK(raw->rebuilds == 1);
    CHECK(odometry.failedCorrections() == 1);
    CHECK(odometry.reinitializations() == 1);
    CHECK(odometry.appliedSolution() == 0);  // 没有生效
    CHECK(odometry.handledSolution() == 1);  // 但不再被尝试：它属于失败之前的里程计系
    CHECK(resources.state.snapshot()->correction_identity.solution_revision == 0);  // 真源没写新校正
    CHECK(raw->lastResult().applied_correction.solution_revision == 0);  // 之后的帧不顶着新身份
    CHECK(raw->lastResult().T_world_body().translation().x() == Approx(0.0));
}

TEST_CASE("标定换代之后，早期关键帧仍能取到当时的标定内容", "[transitions][calibration]") {
    modules::SlamResources resources;
    fw::CalibrationData first;
    first.imu.gravity = 9.81;
    resources.setCalibration(first);
    comm::Runtime runtime(comm::RuntimeOptions{true});
    runtime.addModule<modules::OdometryModule>("/odometry",
                                               std::make_unique<EveryFrameKeyframe>(), resources);
    runtime.init();
    comm::Publisher<fw::SensorPacket> packets(
        runtime.core().topic<fw::SensorPacket>(modules::topics::kSensorPacket),
        comm::PublishOptions{});
    publishPacket(packets, 1, fw::Timestamp{100ms});
    settle(runtime);
    resources.modifyCalibration([](fw::CalibrationData& data) { data.imu.gravity = 9.70; });
    publishPacket(packets, 2, fw::Timestamp{200ms});
    settle(runtime);

    const auto store = resources.keyframes.snapshot();
    REQUIRE(store->size() == 2);
    const auto early = store->frames.at(0);
    REQUIRE(early->calibration != nullptr);
    CHECK(early->calibration_revision == 1);
    CHECK(early->calibration->imu.gravity == Approx(9.81));  // 版本号之外，内容也在
    CHECK(store->frames.at(1)->calibration->imu.gravity == Approx(9.70));
}

TEST_CASE("关键帧通知的序号是关键帧库的序号：断号才意味着缺口", "[transitions][keyframes]") {
    modules::SlamResources resources;
    comm::Runtime runtime(comm::RuntimeOptions{true});
    runtime.addModule<modules::OdometryModule>(
        "/odometry", std::make_unique<EveryFrameKeyframe>(std::vector<bool>{true, false, true}),
        resources);
    auto& observer = runtime.addModule<KeyframeObserver>("/observer");
    runtime.init();
    comm::Publisher<fw::SensorPacket> packets(
        runtime.core().topic<fw::SensorPacket>(modules::topics::kSensorPacket),
        comm::PublishOptions{});
    for (std::uint64_t i = 1; i <= 3; ++i) {
        publishPacket(packets, i, fw::Timestamp{static_cast<std::int64_t>(i) * 100'000'000});
        settle(runtime);
    }
    CHECK(observer.sequences == std::vector<std::uint64_t>{1, 2});  // 帧 1、3 成了关键帧 1、2
}

// ─────────────────────────── 时间、解析与数值 ───────────────────────────

TEST_CASE("时钟换算：长时间戳下恒等映射精确，换算结果带映射的身份", "[transitions][time]") {
    fw::ClockRegistry registry;
    fw::ClockMapping mapping;
    mapping.domain = fw::ClockDomainId{1};
    mapping.version = 3;
    mapping.uncertainty = fw::Duration{2ms};
    registry.setMapping(mapping);
    const fw::Timestamp device{1317042130000000001LL};
    CHECK(registry.toReference(mapping.domain, device) == device);  // 末位纳秒不丢

    const auto converted = registry.convert(mapping.domain, device);
    REQUIRE(converted.has_value());
    CHECK(converted->version == 3);
    CHECK(converted->uncertainty == fw::Duration{2ms});
    CHECK_FALSE(converted->synchronized);

    mapping.scale = 1.0 + 1e-6;  // 1 ppm 漂移，锚点取在数据附近
    mapping.device_anchor = fw::Timestamp{1317042130000000000LL};
    registry.setMapping(mapping);
    const auto drifted = registry.toReference(mapping.domain, device + fw::Duration{1s});
    REQUIRE(drifted.has_value());
    CHECK((*drifted - device).count() == 1'000'001'000);  // 1 s，外加 1 ppm 的漂移 1 µs
}

TEST_CASE("轨迹读取：不是旋转的矩阵、不是单位的四元数一律拒绝，舍入照常接受",
          "[transitions][trajectory]") {
    const auto dir = std::filesystem::temp_directory_path();
    const auto kitti = (dir / "simpleslam_transitions_bad.kitti").string();
    const auto tum = (dir / "simpleslam_transitions_bad.tum").string();
    const auto rounded = (dir / "simpleslam_transitions_rounded.tum").string();
    {
        std::ofstream(kitti) << "2 0 0 0 0 2 0 0 0 0 2 0\n";  // 缩放了两倍
        std::ofstream(tum) << "1.0 0 0 0 0 0 0 2\n";          // 模长为 2
        std::ofstream(rounded) << "1.0 0 0 0 0 0 0.7071 0.7071\n";  // 四位小数的舍入
    }
    CHECK_THROWS_AS(eval::readTrajectory(kitti, eval::TrajectoryFormat::kKitti),
                    eval::TrajectoryIoError);
    CHECK_THROWS_AS(eval::readTrajectory(tum, eval::TrajectoryFormat::kTum),
                    eval::TrajectoryIoError);
    CHECK(eval::readTrajectory(rounded, eval::TrajectoryFormat::kTum).size() == 1);
    std::filesystem::remove(kitti);
    std::filesystem::remove(tum);
    std::filesystem::remove(rounded);
}

TEST_CASE("RPE 按距离取段：走过的路沿真值自身的采样量，不是稀疏估计之间的弦长",
          "[transitions][eval]") {
    const auto pose = [](double x, double y) {
        return SE3d(Vec3d(x, y, 0.0), Eigen::Quaterniond::Identity());
    };
    fw::Trajectory truth;
    truth.push(fw::Timestamp{0}, pose(0, 0));
    truth.push(fw::Timestamp{1s}, pose(0, 10));
    truth.push(fw::Timestamp{2s}, pose(10, 10));
    fw::Trajectory estimate;
    estimate.push(fw::Timestamp{0}, pose(0, 0));
    estimate.push(fw::Timestamp{2s}, pose(11, 10));

    eval::RpeOptions options;
    options.delta_kind = eval::RpeDelta::kDistance;
    options.delta_distance_m = 10.0;
    options.normalize = true;
    const auto rpe = eval::relativePoseError(estimate, truth, options);
    REQUIRE(rpe.pairs == 1);
    CHECK(rpe.translation.rmse == Approx(1.0 / 20.0));  // 走过 20 米，不是弦长 14.1 米
}

TEST_CASE("诊断落盘：数值全精度，非有限值写成合法的 JSON", "[transitions][diagnostics]") {
    const auto path = (std::filesystem::temp_directory_path() / "simpleslam_transitions_diag.jsonl");
    {
        fw::MemoryDiagnosticSink memory;
        {
            std::string unit(100, 'u');  // 调用方的字符串先于落盘死掉：记录必须持有自己的内容
            fw::DiagnosticRecord record;
            record.payload = fw::MetricRecord{"x", 1.00000049, unit};
            memory.submit(record);
            record.payload =
                fw::MetricRecord{"y", std::numeric_limits<double>::quiet_NaN(), "m"};
            memory.submit(record);
        }
        record::FileDiagnosticSink file(path.string());
        for (const auto& record : memory.records()) file.submit(record);
        file.close();
        CHECK(file.written() == 2);
    }
    std::ifstream in(path);
    std::string first;
    std::string second;
    std::getline(in, first);
    std::getline(in, second);
    // 全精度意味着读回来就是原值（默认 6 位有效数字会写成 1）
    const auto at = first.find("\"value\":");
    REQUIRE(at != std::string::npos);
    CHECK(std::stod(first.substr(at + 8)) == 1.00000049);
    CHECK(second.find("\"value\":null") != std::string::npos);
    CHECK(second.find("\"value_text\":\"nan\"") != std::string::npos);
    std::filesystem::remove(path);
}

TEST_CASE("时长换算四舍五入，装不下的量拒绝，配置里的离谱时长在加载时就拒绝",
          "[transitions][config]") {
    CHECK(app::secondsToDuration(0.3) == fw::Duration{300'000'000});  // 截断会得到 299999999
    CHECK_THROWS_AS(app::secondsToDuration(1e12), std::out_of_range);
    CHECK_THROWS_AS(app::secondsToDuration(std::numeric_limits<double>::quiet_NaN()),
                    std::out_of_range);
    CHECK_THROWS_AS(config::loadTaskString("system:\n  packetizer:\n    post_support_s: 1e12\n",
                                           "<string>"),
                    config::ConfigError);
}

// ─────────────────────────── 终结：完成与取消 ───────────────────────────

TEST_CASE("末条关键帧通知丢了、之后再没有输入：终结时后端主动补读，追上全部关键帧",
          "[transitions][finish]") {
    modules::SlamResources resources;
    comm::Runtime runtime(comm::RuntimeOptions{true});
    auto keyframe_writer = resources.keyframes.acquireWriteToken();
    auto& graph = runtime.addModule<modules::GraphOptimizerModule>(
        "/graph", std::make_unique<FlakyOptimizer>(0), resources);
    runtime.init();
    comm::Publisher<fw::KeyframeEvent> notify(
        runtime.core().topic<fw::KeyframeEvent>(modules::topics::kKeyframe), comm::PublishOptions{});
    appendKeyframe(keyframe_writer, 1);
    notify.publish(fw::KeyframeEvent{});
    settle(runtime);
    appendKeyframe(keyframe_writer, 2);
    appendKeyframe(keyframe_writer, 3);  // 这两帧的通知丢了
    settle(runtime);
    REQUIRE(resources.graph.snapshot()->nodeCount() == 1);

    runtime.shutdown();  // 完成已接纳工作：模块主动核对游标
    CHECK(resources.graph.snapshot()->nodeCount() == 3);
    (void)graph;
}

TEST_CASE("取消运行：不再排空，排队中的调用明确结案，结果如实标为取消",
          "[transitions][finish]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    runtime.addModule<Provider>("/p");
    auto& client = runtime.addModule<Client>("/c");
    runtime.init();
    int callbacks = 0;
    REQUIRE(client.cli.asyncCall(1, 0ns, [&](auto) { ++callbacks; }) == comm::ServiceStatus::kOk);

    runtime.shutdown(comm::ShutdownMode::kCancel);
    CHECK(runtime.cancelled());
    CHECK_FALSE(runtime.shutdownDrained());
    CHECK(callbacks == 0);                         // 没有排空：服务端没处理，回调也送不到
    CHECK(client.mailbox()->stats().discarded >= 1);  // 送不到的终态可见

    modules::SlamSystem system;
    system.setSource(std::make_unique<ListSource>(scans(3)), lidarOnly());
    system.setOdometry<StepOdometry>();
    system.init();
    system.runOffline(1);
    system.cancel("操作员中止");
    const auto& result = system.result();
    CHECK(result.outcome.cancelled);
    CHECK_FALSE(result.outcome.complete());
    CHECK(result.outcome.reasons().front().find("输入不完整") != std::string::npos);
    bool mentions_cancel = false;
    for (const auto& reason : result.outcome.reasons()) {
        mentions_cancel = mentions_cancel || reason.find("操作员中止") != std::string::npos;
    }
    CHECK(mentions_cancel);
}

TEST_CASE("终结之前读结果视图是错误：那时只有半截产物", "[transitions][finish]") {
    modules::SlamSystem system;
    system.setSource(std::make_unique<ListSource>(scans(1)), lidarOnly());
    system.setOdometry<StepOdometry>();
    system.init();
    CHECK_THROWS_AS(system.result(), std::logic_error);
    system.runOffline();
    system.shutdown();
    CHECK(system.result().odometry.size() == 1);
}

TEST_CASE("离线时钟原点对到第一条数据：长时间戳下定时器不在第一步集体补触发",
          "[transitions][offline]") {
    constexpr std::int64_t kUnixOrigin = 1'317'042'130'000'000'000;  // 2011 年的 Unix 纳秒
    std::vector<fw::SensorDatum> data;
    for (int i = 1; i <= 5; ++i) data.push_back(scanAt(fw::Timestamp{kUnixOrigin + i * 100'000'000}));
    modules::SlamSystem system;
    auto& ticker = system.runtime().addModule<Ticker>("/ticker");
    system.setSource(std::make_unique<ListSource>(std::move(data)), lidarOnly());
    system.setOdometry<StepOdometry>();
    system.init();
    system.runOffline();
    // 第一条数据在原点 +100 ms；定时器从那里起算，在 +200、+300、+400、+500 ms 各触发一次
    CHECK(ticker.ticks == 4);
}

// ─────────────────────────── 估计推进与失败恢复 ───────────────────────────

namespace {

/// 第二帧报告状态不能再用。
class StateLoss final : public plugin::OdometryBase {
public:
    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }
    int resets = 0;

protected:
    plugin::TrackResult track(const fw::SensorPacket&) override {
        ++frames_;
        plugin::TrackResult result;
        result.status = fw::TrackingStatus::kTracking;
        result.state_usable = frames_ != 2;
        return result;
    }
    void onReset() override { ++resets; }

private:
    int frames_ = 0;
};

/// 保证强异常安全、在第二帧抛异常的算法：状态没被改动，只放弃那一帧。
class IntactOnThrow final : public plugin::OdometryBase {
public:
    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }
    [[nodiscard]] plugin::FailureRecovery failureRecovery() const override {
        return plugin::FailureRecovery::kStateIntact;
    }

protected:
    plugin::TrackResult track(const fw::SensorPacket&) override {
        ++frames_;
        if (frames_ == 2) throw std::runtime_error("这一帧失败，但状态没动");
        plugin::TrackResult result;
        result.status = fw::TrackingStatus::kTracking;
        return result;
    }

private:
    int frames_ = 0;
};

struct ResultObserver final : comm::Module {
    std::vector<fw::OdometryResult> results;
    void onInit() override {
        subscribe<fw::OdometryResult>(modules::topics::kOdometry, 0, [this](const fw::OdometryResult& result) {
            results.push_back(result);
        });
    }
};

}  // namespace

TEST_CASE("算法报告状态不能再用：结果不发布，按恢复协议重新初始化", "[transitions][recovery]") {
    modules::SlamSystem system;
    system.setSource(std::make_unique<ListSource>(scans(3)), lidarOnly());
    auto& odometry = system.setOdometry<StateLoss>();
    system.init();
    system.runOffline();
    system.shutdown();

    CHECK(odometry.stateLosses() == 1);
    CHECK(odometry.failedFrames() == 1);
    CHECK(odometry.reinitializations() == 1);
    CHECK(system.result().odometry.size() == 2);  // 第二帧的位姿不可信，没有发布
    CHECK(system.result().outcome.reinitializations == 1);
    CHECK_FALSE(system.result().outcome.processingComplete());
}

TEST_CASE("重新初始化前后：结果与失败记录绑定到实际的包，帧序号不重复，段号分开两个里程计系",
          "[transitions][recovery]") {
    modules::SlamSystem system;
    auto& observer = system.runtime().addModule<ResultObserver>("/observer");
    system.setSource(std::make_unique<ListSource>(scans(3)), lidarOnly());
    system.setOdometry<StateLoss>();
    system.init();
    system.runOffline();
    system.shutdown();

    REQUIRE(observer.results.size() == 2);
    const auto& before = observer.results.at(0);
    const auto& after = observer.results.at(1);
    CHECK(before.packet_sequence == 1);
    CHECK(after.packet_sequence == 3);  // 第二包没有结果，第三包的结果不冒充它
    CHECK(before.sequence == 1);
    CHECK(after.sequence == 2);         // 重新初始化不重数：会话里的帧序号唯一
    CHECK(before.odometry_segment == 0);
    CHECK(after.odometry_segment == 1);

    std::optional<fw::DiagnosticHeader> lost;
    for (const auto& record : system.diagnostics().records()) {
        const auto* event = std::get_if<fw::EventRecord>(&record.payload);
        if (event != nullptr && event->key == "odometry.state_unusable") lost = record.header;
    }
    REQUIRE(lost.has_value());
    CHECK(lost->frame_sequence == 2);  // 失败终态落在第二包上，不是"已经跟踪了几帧"
    CHECK(lost->data_time == fw::Timestamp{200ms});
}

TEST_CASE("强异常安全的算法失败一帧：不重新初始化，但那段输入没被积分要记下来",
          "[transitions][recovery]") {
    modules::SlamSystem system;
    system.setSource(std::make_unique<ListSource>(scans(3)), lidarOnly());
    auto& odometry = system.setOdometry<IntactOnThrow>();
    system.init();
    system.runOffline();
    system.shutdown();

    CHECK(odometry.failedFrames() == 1);
    CHECK(odometry.reinitializations() == 0);
    // 第三帧的传播从第二帧的参考时刻起算，算法的状态却停在第一帧：中间那段没进任何一包
    CHECK(odometry.propagationGaps() == 1);
    CHECK(system.result().outcome.propagation_gaps == 1);
}
