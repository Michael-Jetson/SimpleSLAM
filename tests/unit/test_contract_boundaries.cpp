// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/modules/slam_system.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace simpleslam;
using namespace std::chrono_literals;
using Catch::Approx;
namespace fw = simpleslam::fw;
namespace comm = simpleslam::comm;
namespace mod = simpleslam::modules;
namespace pl = simpleslam::plugin;

namespace {

fw::SensorPacket packet(std::uint64_t sequence) {
    fw::SensorPacket result;
    result.sequence = sequence;
    result.reference_time = fw::Timestamp{100ms * sequence};
    result.propagation = {result.reference_time - 100ms, result.reference_time};
    result.imu_coverage = {result.reference_time, result.reference_time};
    return result;
}

enum class Fault { kNone, kOutput, kStateLoss, kRollback, kFuture, kKeyframeHook };

class Estimator final : public pl::OdometryBase {
public:
    explicit Estimator(Fault fault = Fault::kNone) : fault(fault) {}
    pl::SupportGapPolicy supportGapPolicy() const override { return policy; }
    pl::FailureRecovery failureRecovery() const override { return recovery; }

    Fault fault;
    pl::SupportGapPolicy policy = pl::SupportGapPolicy::kDegrade;
    pl::FailureRecovery recovery = pl::FailureRecovery::kReinitialize;
    std::uint64_t fail_at = 2;
    int calls = 0;
    int resets = 0;
    bool reset_fails = false;
    std::optional<fw::TimeRange> seen_gap;

protected:
    pl::TrackResult track(const fw::SensorPacket& input) override {
        ++calls;
        ++position_;
        seen_gap = context().missing_propagation;
        pl::TrackResult result;
        result.T_odom_body = SE3d(Vec3d(position_, 0, 0), Eigen::Quaterniond::Identity());
        result.suggest_keyframe = true;
        if (input.sequence != fail_at) return result;
        switch (fault) {
            case Fault::kOutput: {
                position_ = 100;
                fw::RelativeConstraint constraint;
                constraint.to_time = input.reference_time + 1ms;
                result.constraint_to_last_keyframe = constraint;
                break;
            }
            case Fault::kStateLoss:
                result.state_usable = false;
                break;
            case Fault::kRollback:
                --position_;
                result.T_odom_body = SE3d(Vec3d(position_, 0, 0), Eigen::Quaterniond::Identity());
                result.state_time = input.reference_time - 100ms;
                result.status = fw::TrackingStatus::kDegraded;
                break;
            case Fault::kFuture:
                result.state_time = input.reference_time + 1ms;
                break;
            default:
                break;
        }
        return result;
    }
    void onReset() override {
        ++resets;
        if (reset_fails) throw std::runtime_error("恢复所需资源不可用");
        position_ = 0;
    }
    void onKeyframeSelected(const fw::OdometryResult& result) override {
        if (fault == Fault::kKeyframeHook && result.packet_sequence == fail_at) {
            throw std::runtime_error("关键帧地图构建失败");
        }
    }

private:
    double position_ = 0;
};

struct Rig {
    explicit Rig(Fault fault = Fault::kNone) {
        auto implementation = std::make_unique<Estimator>(fault);
        algorithm = implementation.get();
        module = &runtime.addModule<mod::OdometryModule>("/odometry", std::move(implementation),
                                                       resources);
        runtime.init();
        packets = comm::Publisher<fw::SensorPacket>(
            runtime.core().topic<fw::SensorPacket>(mod::topics::kSensorPacket), {false, true});
        corrections = comm::Publisher<fw::CorrectionEvent>(
            runtime.core().topic<fw::CorrectionEvent>(mod::topics::kCorrection), {});
    }
    void settle() {
        for (int i = 0; i < 32; ++i) {
            if (runtime.spinOnce() == 0) return;
        }
        FAIL("小夹具未能排空");
    }
    void send(fw::SensorPacket input) {
        REQUIRE(packets.publish(std::move(input)).deliveredTo(1));
        settle();
    }
    fw::PoseGraphData solution(std::uint64_t revision, std::uint64_t through, double offset) {
        fw::PoseGraphData graph;
        graph.identity.session = resources.session;
        graph.identity.solution_revision = revision;
        graph.identity.covered_until = through;
        for (const auto& frame : resources.keyframes.snapshot()->since(0)) {
            if (frame->sequence > through) break;
            fw::PoseGraphNode node;
            node.id = frame->id;
            node.time = frame->time;
            node.T_world_body = SE3d(Vec3d(offset, 0, 0), Eigen::Quaterniond::Identity())
                                    .compose(frame->T_odom_body);
            graph.addNode(node);
        }
        return graph;
    }
    void correct(fw::PoseGraphData graph) {
        corrections.publish(fw::CorrectionEvent{fw::Timestamp{0},
                              std::make_shared<const fw::PoseGraphData>(std::move(graph))});
        settle();
    }
    mod::SlamResources resources;
    comm::Runtime runtime{comm::RuntimeOptions{1, true}};
    Estimator* algorithm = nullptr;
    mod::OdometryModule* module = nullptr;
    comm::Publisher<fw::SensorPacket> packets;
    comm::Publisher<fw::CorrectionEvent> corrections;
};

class Source final : public fw::ISensorSource {
public:
    explicit Source(int count) : count_(count) {}
    std::optional<fw::SensorDatum> next() override {
        if (read == count_) return {};
        ++read;
        auto scan = std::make_shared<fw::LidarScan>();
        scan->reference_time = fw::Timestamp{100ms * read};
        scan->coverage = {scan->reference_time - 100ms, scan->reference_time};
        return fw::Ptr<fw::LidarScan>(scan);
    }
    int read = 0;

private:
    int count_;
};

fw::PacketizerPolicy lidarOnly() {
    fw::PacketizerPolicy policy;
    policy.imu = fw::ImuRequirement::kNotRequired;
    return policy;
}

struct Blocking final : comm::Module {
    void onInit() override {
        subscribe<int>("/input", [this](int n) {
            if (n == 1) {
                entered.set_value();
                resume.wait();
                ++completed;
            } else {
                ++tail;
            }
        }, comm::SubscribeOptions{8, comm::Delivery::kReliable});
    }
    std::promise<void> entered;
    std::promise<void> release;
    std::shared_future<void> resume = release.get_future().share();
    int completed = 0;
    int tail = 0;
};

struct Provider final : comm::Module {
    void onInit() override { advertiseService<int, int>("/service", &Provider::handle); }
    int handle(const int& n) { ++calls; return n; }
    int calls = 0;
};

struct Client final : comm::Module {
    void onInit() override { service = serviceClient<int, int>("/service"); }
    comm::ServiceClient<int, int> service;
};

struct ReentrantHooks final : comm::Module {
    explicit ReentrantHooks(comm::Runtime& owner) : owner(owner) {}
    void onFinish() override { ++finished; owner.shutdown(); }
    void onShutdown() override { ++stopped; owner.shutdown(); }
    comm::Runtime& owner;
    int finished = 0;
    int stopped = 0;
};

}  // namespace

TEST_CASE("复位之后仅由当前段的锚点推导校正，旧段迟到解不改变新段", "[boundaries][correction]") {
    Rig rig(Fault::kStateLoss);
    rig.algorithm->fail_at = 3;
    rig.send(packet(1));
    rig.send(packet(2));
    auto old_solution = rig.solution(1, 2, 5);
    rig.send(packet(3));
    rig.send(packet(4));
    REQUIRE(rig.resources.keyframes.snapshot()->find(fw::KeyframeId{3})->odometry_segment == 1);
    rig.correct(std::move(old_solution));
    CHECK(rig.module->appliedSolution() == 0);
    CHECK(rig.resources.state.snapshot()->T_world_odom.translation().x() == Approx(0));

    rig.correct(rig.solution(2, 3, 7));
    CHECK(rig.module->appliedSolution() == 2);
    rig.send(packet(5));
    const auto& result = rig.algorithm->lastResult();
    CHECK(result.odometry_segment == 1);
    CHECK(result.T_world_body().translation().x() == Approx(9));
}

TEST_CASE("产物校验失败必须恢复，即使 track 声明抛异常时状态不变", "[boundaries][recovery]") {
    Rig rig(Fault::kOutput);
    rig.algorithm->recovery = pl::FailureRecovery::kStateIntact;
    rig.send(packet(1));
    rig.send(packet(2));
    rig.send(packet(3));
    CHECK(rig.module->contractErrors() == 1);
    CHECK(rig.module->reinitializations() == 1);
    CHECK(rig.algorithm->lastResult().T_odom_body.translation().x() == Approx(1));
    CHECK(rig.algorithm->lastResult().sequence == 2);
    CHECK(rig.algorithm->lastResult().odometry_segment == 1);
    CHECK_FALSE(rig.resources.keyframes.snapshot()->find(fw::KeyframeId{2})->previous.valid());
}

TEST_CASE("算法执行前拒绝非法输入无需复位，输入错误不能伪造一次算法执行", "[boundaries][recovery]") {
    Rig rig;
    rig.send(packet(1));
    auto invalid = packet(2);
    invalid.imu_coverage = {fw::Timestamp{100ms}, fw::Timestamp{200ms}};
    rig.send(invalid);  // 没有 IMU 却声明覆盖非空区间
    CHECK(rig.algorithm->calls == 1);
    CHECK(rig.module->contractErrors() == 1);
    CHECK(rig.module->reinitializations() == 0);
    rig.send(packet(3));
    CHECK(rig.algorithm->lastResult().T_odom_body.translation().x() == Approx(2));
    CHECK(rig.algorithm->lastResult().support_gap);
}

TEST_CASE("关键帧后置钩子失败不提交结果和序号，并按产物失败恢复", "[boundaries][recovery]") {
    Estimator algorithm(Fault::kKeyframeHook);
    algorithm.recovery = pl::FailureRecovery::kStateIntact;
    (void)algorithm.process(packet(1), {});
    CHECK_THROWS_AS(algorithm.process(packet(2), {}), pl::OdometryContractError);
    CHECK(algorithm.lastResult().packet_sequence == 1);
    CHECK(algorithm.lastResult().sequence == 1);

    Rig rig(Fault::kKeyframeHook);
    rig.algorithm->recovery = pl::FailureRecovery::kStateIntact;
    rig.send(packet(1));
    rig.send(packet(2));
    rig.send(packet(3));
    CHECK(rig.module->reinitializations() == 1);
    CHECK(rig.algorithm->lastResult().sequence == 2);
}

TEST_CASE("传播缺口进入算法政策：拒绝不调用 track，降级可读到缺失区间", "[boundaries][support]") {
    Rig rig(Fault::kRollback);
    SECTION("拒绝") { rig.algorithm->policy = pl::SupportGapPolicy::kReject; }
    SECTION("降级") { rig.algorithm->policy = pl::SupportGapPolicy::kDegrade; }
    SECTION("初始化后等待政策按降级处理") {
        rig.algorithm->policy = pl::SupportGapPolicy::kWaitForInitialization;
    }
    rig.send(packet(1));
    rig.send(packet(2));
    rig.send(packet(3));
    CHECK(rig.module->propagationGaps() == 1);
    if (rig.algorithm->policy == pl::SupportGapPolicy::kReject) {
        CHECK(rig.algorithm->calls == 2);
        CHECK(rig.module->rejectedForGapFrames() == 1);
        CHECK(rig.algorithm->lastResult().packet_sequence == 2);
    } else {
        CHECK(rig.algorithm->calls == 3);
        CHECK(rig.algorithm->lastResult().support_gap);
        CHECK(rig.algorithm->lastResult().status == fw::TrackingStatus::kDegraded);
        REQUIRE(rig.algorithm->seen_gap.has_value());
        CHECK(rig.algorithm->seen_gap->begin == fw::Timestamp{100ms});
        CHECK(rig.algorithm->seen_gap->end == fw::Timestamp{200ms});
    }
}

TEST_CASE("状态时刻不能超前于输入，复位失败期间禁止继续 track，恢复后才能换段续跑",
          "[boundaries][recovery]") {
    Rig rig(Fault::kFuture);
    rig.send(packet(1));
    rig.algorithm->reset_fails = true;
    rig.send(packet(2));
    CHECK(rig.module->contractErrors() == 1);
    rig.send(packet(3));
    CHECK(rig.algorithm->calls == 2);  // 复位尚未成功，第三包不进入算法
    CHECK(rig.module->failedFrames() == 1);
    CHECK(rig.runtime.isolatedFailures() == 0);  // 恢复失败是领域终态，不欠消费确认
    rig.algorithm->reset_fails = false;
    rig.send(packet(4));  // 本包用于恢复，尚未跟踪
    rig.send(packet(5));
    CHECK(rig.algorithm->calls == 3);
    CHECK(rig.algorithm->lastResult().packet_sequence == 5);
    CHECK(rig.algorithm->lastResult().sequence == 2);
    CHECK(rig.algorithm->lastResult().T_odom_body.translation().x() == Approx(1));
    CHECK(rig.module->receivedPackets() == rig.module->processedFrames()
          + rig.module->contractErrors() + rig.module->failedFrames());
}

TEST_CASE("在线取消关闭队列后再等静止：本模块尾项和其他模块的服务均不开始",
          "[boundaries][cancel]") {
    comm::Runtime runtime({1, false});
    auto& running = runtime.addModule<Blocking>("/running");
    auto& provider = runtime.addModule<Provider>("/provider");
    auto& client = runtime.addModule<Client>("/client");
    runtime.init();
    comm::Publisher<int> input(runtime.core().topic<int>("/input"), {false, true});
    REQUIRE(input.publish(1).deliveredTo(1));
    std::thread driver([&] { runtime.spin(); });
    const auto entered = running.entered.get_future().wait_for(5s);
    CHECK(entered == std::future_status::ready);
    CHECK(input.publish(2).deliveredTo(1));
    int completions = 0;
    CHECK(client.service.asyncCall(1, 0ns, [&](auto) { ++completions; }) == comm::ServiceStatus::kOk);
    std::thread closer([&] { runtime.shutdown(comm::ShutdownMode::kCancel); });
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!runtime.executionClosed() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
    }
    const bool closed = runtime.executionClosed();
    running.release.set_value();
    closer.join();
    driver.join();
    CHECK(closed);
    CHECK(running.completed == 1);
    CHECK(running.tail == 0);
    CHECK(provider.calls == 0);
    CHECK(completions == 0);
    CHECK(client.service.pendingCalls() == 0);
    CHECK(client.service.undeliveredCompletions() == 1);
    CHECK(runtime.cancelled());
}

TEST_CASE("完成和收尾钩子重入 shutdown 不等待自身，收尾仅一次", "[boundaries][finish]") {
    comm::Runtime runtime({1, true});
    auto& hooks = runtime.addModule<ReentrantHooks>("/hooks", runtime);
    runtime.init();
    runtime.shutdown();
    CHECK(hooks.finished == 1);
    CHECK(hooks.stopped == 1);
    CHECK(runtime.state() == comm::RuntimeState::kTornDown);
}

TEST_CASE("回调内同步终结被明确拒绝，之后控制线程仍能生成真实最终结果", "[boundaries][finish]") {
    mod::SlamSystem system;
    system.setSource(std::make_unique<Source>(2), lidarOnly());
    auto& odometry = system.setOdometry<Estimator>();
    system.init();
    bool rejected = false;
    bool cancel = false;
    SECTION("完成") {}
    SECTION("取消") { cancel = true; }
    REQUIRE(odometry.mailbox()->postTask([&] {
        try {
            if (cancel) system.cancel("callback");
            else system.shutdown();
        } catch (const std::logic_error&) {
            rejected = true;
        }
    }));
    system.runtime().spinOnce();
    CHECK(rejected);
    CHECK_FALSE(system.finalized());
    CHECK_FALSE(odometry.stopped());
    CHECK_THROWS_AS(system.result(), std::logic_error);
    system.runOffline();
    system.shutdown();
    CHECK(system.result().outcome.complete());
    CHECK(system.result().odometry.size() == 2);
}

TEST_CASE("底层已经取消后高层收尾保留真实取消结局", "[boundaries][finish]") {
    mod::SlamSystem system;
    system.init();
    system.runtime().shutdown(comm::ShutdownMode::kCancel);
    system.shutdown();
    CHECK(system.result().outcome.cancelled);
}

TEST_CASE("停止输入后 runner 和直接 pump 都不再读取外部来源", "[boundaries][offline]") {
    mod::SlamSystem system;
    auto source = std::make_unique<Source>(4);
    auto* raw = source.get();
    auto& input = system.setSource(std::move(source), lidarOnly());
    system.setOdometry<Estimator>();
    system.init();
    system.requestStop();
    CHECK(system.runOffline(1) == 0);
    CHECK(input.pump(1).stopped);
    CHECK(raw->read == 0);
    CHECK(input.rejectedPackets() == 0);
}

TEST_CASE("有限步进的拒收也消耗预算，后续调用能从下一包恢复", "[boundaries][offline]") {
    mod::SlamSystem system;
    auto source = std::make_unique<Source>(4);
    auto* raw = source.get();
    system.setSource(std::move(source), lidarOnly());
    auto& odometry = system.setOdometry<Estimator>();
    system.init();
    comm::Publisher<fw::SensorPacket> input(
        system.runtime().core().topic<fw::SensorPacket>(mod::topics::kSensorPacket), {});
    // 先制造真实背压。夹具包的时刻都早于来源首帧，避免影响后续轨迹的时间检查。
    for (std::uint64_t i = 1; i <= 8; ++i) {
        auto queued = packet(i);
        queued.reference_time = fw::Timestamp{1ms * i};
        queued.imu_coverage = {queued.reference_time, queued.reference_time};
        queued.propagation = {queued.reference_time, queued.reference_time};
        REQUIRE(input.publish(queued).deliveredTo(1));
    }
    CHECK(system.runOffline(1) == 0);  // 唯一一次尝试被拒；settle 随后释放容量
    CHECK(raw->read == 1);
    CHECK(system.sensorIo()->rejectedPackets() == 1);
    CHECK(odometry.processedFrames() == 8);
    CHECK(system.runOffline(1) == 1);
    CHECK(raw->read == 2);
}
