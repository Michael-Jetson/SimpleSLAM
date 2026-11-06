// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 端到端：用三个最小算法把整条链路跑起来。
///
/// 这个文件同时是写算法的模板——除了三个 `override`，其余全是框架的事。

#include <SimpleSLAM/modules/slam_system.hpp>

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace simpleslam;
namespace fw = simpleslam::fw;
namespace plugin = simpleslam::plugin;

namespace {

// ── 数据来源：IMU 100 Hz，激光 100 Hz 的合成数据 ──────────────────────────────
class SyntheticSource final : public fw::ISensorSource {
public:
    explicit SyntheticSource(int scans) : scans_(scans) {}

    std::optional<fw::SensorDatum> next() override {
        if (emitted_scans_ >= scans_) return std::nullopt;
        if (next_imu_ns_ <= next_scan_ns_) {
            auto sample = std::make_shared<fw::ImuSample>();
            sample->time = fw::Timestamp{next_imu_ns_};
            sample->gyro = {0.0, 0.0, 0.01};
            sample->accel = {0.0, 0.0, 9.81};
            next_imu_ns_ += kImuPeriodNs;
            return fw::SensorDatum{fw::Ptr<fw::ImuSample>(sample)};
        }
        auto scan = std::make_shared<fw::LidarScan>();
        scan->reference_time = fw::Timestamp{next_scan_ns_};
        scan->coverage = fw::TimeRange{fw::Timestamp{next_scan_ns_ - kScanPeriodNs},
                                       fw::Timestamp{next_scan_ns_}};
        for (int i = 0; i < 64; ++i) {
            scan->points.push(static_cast<float>(i), 0.0f, 0.0f);
            scan->points.time_offset_ns.push_back(static_cast<std::int32_t>(i * 100));
        }
        next_scan_ns_ += kScanPeriodNs;
        ++emitted_scans_;
        return fw::SensorDatum{fw::Ptr<fw::LidarScan>(scan)};
    }

    [[nodiscard]] std::string name() const override { return "SyntheticSource"; }

private:
    static constexpr std::int64_t kImuPeriodNs = 5'000'000;   // 200 Hz
    static constexpr std::int64_t kScanPeriodNs = 10'000'000; // 100 Hz
    int scans_;
    int emitted_scans_ = 0;
    std::int64_t next_imu_ns_ = 0;
    std::int64_t next_scan_ns_ = kScanPeriodNs;
};

// ── 算法一：里程计。真实实现只需要换掉 track() 的内容 ─────────────────────────
class ConstantVelocityOdometry final : public plugin::OdometryBase {
public:
    [[nodiscard]] std::string name() const override { return "ConstantVelocityOdometry"; }
    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }

protected:
    plugin::TrackResult track(const fw::SensorPacket& packet) override {
        ++frames_;
        const double x = static_cast<double>(frames_);
        pose_ = SE3d(Eigen::Vector3d(x, 0.0, 0.0), Eigen::Quaterniond::Identity());

        plugin::TrackResult result;
        result.status = fw::TrackingStatus::kTracking;
        result.T_odom_body = pose_;
        result.registration_information = Mat6d::Identity() * 100.0;
        result.registration_information_source = fw::InformationSource::kPropagated;
        // 跨关键帧的相对约束由算法给出：框架不会拿上面那个单帧量冒充
        if (hasKeyframe()) {
            fw::RelativeConstraint constraint;
            constraint.from_time = lastKeyframeTime();
            constraint.to_time = packet.reference_time;
            constraint.measurement = lastKeyframePose().inverse().compose(pose_);
            constraint.information = Mat6d::Identity() * 25.0;
            constraint.source = fw::InformationSource::kPropagated;
            result.constraint_to_last_keyframe = constraint;
        }
        result.suggest_keyframe = (frames_ % 2 == 0);
        result.inliers = static_cast<std::uint32_t>(packet.lidar() != nullptr
                                                        ? packet.lidar()->points.size()
                                                        : 0);
        result.iterations = 3;
        return result;
    }

    void onCorrection(const SE3d& T_world_odom, const fw::PoseGraphData& graph) override {
        (void)graph;
        corrections_seen_ = true;
        last_correction_ = T_world_odom;
    }

    void onReset() override { frames_ = 0; }

public:
    [[nodiscard]] bool sawCorrection() const noexcept { return corrections_seen_; }

private:
    SE3d pose_ = SE3d::Identity();
    SE3d last_correction_ = SE3d::Identity();
    std::uint64_t frames_ = 0;
    bool corrections_seen_ = false;
};

// ── 算法二：位置识别。第 5 个关键帧时报一次回环 ──────────────────────────────
class ScriptedPlaceRecognizer final : public plugin::PlaceRecognizerBase {
public:
    [[nodiscard]] std::string name() const override { return "ScriptedPlaceRecognizer"; }

protected:
    void onInsert(const fw::Keyframe& keyframe) override { ids_.push_back(keyframe.id); }

    std::vector<fw::LoopConstraint> onDetectLoops(const fw::Keyframe& keyframe) override {
        if (ids_.size() != 5) return {};
        fw::LoopConstraint constraint;
        constraint.query = keyframe.id;
        constraint.match = ids_.front();
        constraint.T_match_query = SE3d::Identity();
        constraint.information = Mat6d::Identity() * 10.0;
        constraint.score = 0.9;
        return {constraint};
    }

    fw::RelocationResult onRelocalize(const fw::SensorPacket& packet,
                                      const fw::RelocationRequest& request) override {
        (void)packet;
        (void)request;
        fw::RelocationResult result;
        result.success = !ids_.empty();
        return result;
    }

private:
    std::vector<fw::KeyframeId> ids_;
};

// ── 算法三：图优化。把非固定节点整体拉回一点，模拟一次收敛 ────────────────────
class ShrinkingOptimizer final : public plugin::GraphOptimizerBase {
public:
    [[nodiscard]] std::string name() const override { return "ShrinkingOptimizer"; }

protected:
    Report onOptimize(fw::GraphProblem& problem) override {
        for (auto& node : problem.nodes) {
            if (node.fixed) continue;
            const Eigen::Vector3d t = node.T_world_body.translation() * 0.9;
            node.T_world_body = SE3d(t, node.T_world_body.quat());
        }
        Report report;
        report.status = Status::kConverged;
        report.iterations = 5;
        report.initial_cost = 10.0;
        report.final_cost = 0.5;
        return report;
    }
};

modules::SlamSystem::Options offlineOptions() {
    modules::SlamSystem::Options options;
    options.offline = true;
    return options;
}

}  // namespace

TEST_CASE("端到端：数据源到位姿图，校正回流到里程计", "[modules][system]") {
    modules::SlamSystem system(offlineOptions());
    system.setSource(std::make_unique<SyntheticSource>(20));
    auto& odometry = system.setOdometry<ConstantVelocityOdometry>();
    auto& recognizer = system.setPlaceRecognizer<ScriptedPlaceRecognizer>();
    auto& optimizer = system.setGraphOptimizer<ShrinkingOptimizer>();
    system.init();

    const std::size_t packets = system.runOffline();

    CHECK(packets == 20);
    CHECK(odometry.processedFrames() == 20);
    CHECK(odometry.keyframeCount() == 10);          // 每两帧一个关键帧
    CHECK(recognizer.indexedFrames() == 10);        // 位置识别把每个关键帧都补读到了
    CHECK(recognizer.loopsFound() == 1);
    CHECK(optimizer.optimizations() == 1);

    const auto keyframes = system.resources().keyframes.snapshot();
    const auto graph = system.resources().graph.snapshot();
    CHECK(keyframes->size() == 10);
    CHECK(graph->nodeCount() == 10);
    CHECK(graph->edgeCount() == 10);  // 9 条里程计边 + 1 条回环边

    // 真源完整：每个关键帧都带着整包原始数据
    keyframes->frames.forEach([](const fw::KeyframePtr& frame) {
        REQUIRE(frame->packet != nullptr);
        CHECK(frame->packet->lidar() != nullptr);
        CHECK(frame->packet->imuCoversDeclaredRange());
    });

    // 算法声明的来源原样入库，不由"矩阵是否为零"反推
    const auto second_frame = keyframes->find(fw::KeyframeId{2});
    REQUIRE(second_frame != nullptr);
    CHECK(second_frame->registration_information_source == fw::InformationSource::kPropagated);
    REQUIRE(second_frame->constraint_from_previous.has_value());
    CHECK(second_frame->constraint_from_previous->source == fw::InformationSource::kPropagated);
    CHECK(second_frame->previous == fw::KeyframeId{1});

    // 图边用的是跨关键帧的约束，不是单帧配准量
    bool checked_edge = false;
    graph->edges.forEach([&](const fw::PoseGraphEdge& edge) {
        if (edge.kind != fw::EdgeKind::kOdometry || checked_edge) return;
        CHECK(edge.information_source == fw::InformationSource::kPropagated);
        CHECK_FALSE(edge.heuristicWeight());
        CHECK(edge.information(0, 0) == 25.0);
        checked_edge = true;
    });
    CHECK(checked_edge);

    // 校正回流：当前状态的世界偏移不再是单位阵，算法也收到了通知
    const auto state = system.resources().state.snapshot();
    CHECK(state->status == fw::TrackingStatus::kTracking);
    CHECK(state->correction_identity.solution_revision == 1);
    CHECK(state->T_world_odom.translation().norm() > 1e-9);
    auto& algorithm = dynamic_cast<ConstantVelocityOdometry&>(odometry.algorithm());
    CHECK(algorithm.sawCorrection());
}

TEST_CASE("离线双跑逐位一致", "[modules][system]") {
    auto run = [] {
        modules::SlamSystem system(offlineOptions());
        system.setSource(std::make_unique<SyntheticSource>(12));
        system.setOdometry<ConstantVelocityOdometry>();
        system.setPlaceRecognizer<ScriptedPlaceRecognizer>();
        system.setGraphOptimizer<ShrinkingOptimizer>();
        system.init();
        system.runOffline();

        std::vector<double> trajectory;
        system.resources().graph.snapshot()->nodes.forEach([&](const fw::PoseGraphNode& node) {
            trajectory.push_back(node.T_world_body.translation().x());
            trajectory.push_back(node.T_world_body.translation().y());
            trajectory.push_back(node.T_world_body.translation().z());
        });
        return trajectory;
    };

    const auto first = run();
    const auto second = run();
    REQUIRE(first.size() == second.size());
    CHECK(first == second);  // 逐位相同，不是近似相等
}

TEST_CASE("身份失效的校正被拒绝", "[modules][system]") {
    modules::SlamSystem system(offlineOptions());
    system.setSource(std::make_unique<SyntheticSource>(6));
    auto& odometry = system.setOdometry<ConstantVelocityOdometry>();
    system.setPlaceRecognizer<ScriptedPlaceRecognizer>();
    system.setGraphOptimizer<ShrinkingOptimizer>();
    system.init();
    system.runOffline();

    auto publisher = comm::Publisher<fw::CorrectionEvent>(
        system.runtime().core().topic<fw::CorrectionEvent>(modules::topics::kCorrection),
        comm::PublishOptions{});

    // 校正事件携带解本身：身份写在快照里，与数据同源
    auto foreign = std::make_shared<fw::PoseGraphData>();
    foreign->identity.session = fw::SessionId{999};  // 另一个会话
    foreign->identity.solution_revision = 99;
    foreign->identity.calibration_revision = system.resources().calibrationRevision();
    fw::CorrectionEvent stale;
    stale.solution = foreign;
    publisher.publish(stale);
    system.settle();

    // 会话不符：拒绝，不污染当前状态（这段数据太短，本来也没有合法校正）
    const auto state = system.resources().state.snapshot();
    CHECK_FALSE(state->correction_identity.session == fw::SessionId{999});
    CHECK(state->correction_identity.solution_revision == 0);
    CHECK(state->T_world_odom.translation().norm() < 1e-12);
    CHECK(odometry.processedFrames() == 6);
}

TEST_CASE("写令牌保证资源单写者", "[modules][system]") {
    modules::SlamSystem system(offlineOptions());
    system.setOdometry<ConstantVelocityOdometry>();  // 已取走关键帧库与当前状态的写令牌
    CHECK_THROWS_AS(system.resources().keyframes.acquireWriteToken(), std::logic_error);
    CHECK_THROWS_AS(system.resources().state.acquireWriteToken(), std::logic_error);

    system.setGraphOptimizer<ShrinkingOptimizer>();  // 取走位姿图写令牌
    CHECK_THROWS_AS(system.resources().graph.acquireWriteToken(), std::logic_error);
}

TEST_CASE("处理约束之前先追平关键帧，通知排在后面也读得到端点", "[modules][system][constraint]") {
    modules::SlamSystem system(offlineOptions());
    auto& io = system.setSource(std::make_unique<SyntheticSource>(8));
    system.setOdometry<ConstantVelocityOdometry>();
    auto& optimizer = system.setGraphOptimizer<ShrinkingOptimizer>();
    system.init();

    auto publisher = comm::Publisher<fw::LoopConstraint>(
        system.runtime().core().topic<fw::LoopConstraint>(modules::topics::kLoopConstraint),
        comm::PublishOptions{});

    // 约束排在关键帧通知前面。关键帧先入库后通知，所以只要入过库，图优化就读得到端点
    io.pump(4);
    fw::LoopConstraint loop;
    loop.id = fw::ConstraintId{7};
    loop.query = fw::KeyframeId{2};
    loop.match = fw::KeyframeId{1};
    loop.T_match_query = SE3d::Identity();
    loop.information = Mat6d::Identity() * 10.0;
    publisher.publish(loop);
    system.odometry()->mailbox()->drainSome();  // 关键帧入库，通知排到约束后面
    system.settle();

    CHECK(optimizer.committedConstraints() == 1);
    CHECK(optimizer.rejectedConstraints() == 0);

    std::size_t loop_edges = 0;
    system.resources().graph.snapshot()->edges.forEach([&](const fw::PoseGraphEdge& edge) {
        if (edge.kind == fw::EdgeKind::kLoop) ++loop_edges;
    });
    CHECK(loop_edges == 1);
}

TEST_CASE("端点不存在的约束被拒绝，并记进运行结局", "[modules][system][constraint]") {
    modules::SlamSystem system(offlineOptions());
    system.setSource(std::make_unique<SyntheticSource>(4));
    system.setOdometry<ConstantVelocityOdometry>();
    auto& optimizer = system.setGraphOptimizer<ShrinkingOptimizer>();
    system.init();

    auto publisher = comm::Publisher<fw::LoopConstraint>(
        system.runtime().core().topic<fw::LoopConstraint>(modules::topics::kLoopConstraint),
        comm::PublishOptions{});
    fw::LoopConstraint orphan;
    orphan.id = fw::ConstraintId{7};
    orphan.query = fw::KeyframeId{300};
    orphan.match = fw::KeyframeId{100};
    orphan.T_match_query = SE3d::Identity();
    orphan.information = Mat6d::Identity() * 10.0;
    publisher.publish(orphan);
    system.runOffline();

    CHECK(optimizer.rejectedConstraints() == 1);
    CHECK(optimizer.committedConstraints() == 0);
    CHECK(system.diagnostics().countEvent("graph.constraint_rejected") == 1);
    CHECK(system.runOutcome().rejected_constraints == 1);
    CHECK_FALSE(system.runOutcome().outputComplete());
}

TEST_CASE("校正事件自带解快照，身份与数据同源", "[modules][system][solution]") {
    modules::SlamSystem system(offlineOptions());
    system.setSource(std::make_unique<SyntheticSource>(20));
    system.setOdometry<ConstantVelocityOdometry>();
    system.setPlaceRecognizer<ScriptedPlaceRecognizer>();
    system.setGraphOptimizer<ShrinkingOptimizer>();
    std::vector<fw::CorrectionEvent> seen;
    system.runtime().node().subscribe<fw::CorrectionEvent>(
        modules::topics::kCorrection, 0, [&](const fw::CorrectionEvent& e) { seen.push_back(e); });
    system.init();

    system.runOffline();
    REQUIRE(seen.size() == 1);
    REQUIRE(seen.front().valid());
    // 身份来自快照自身，不是另外贴上去的
    CHECK(seen.front().identity().solution_revision == 1);
    CHECK(seen.front().solution->identity.session == system.resources().session);

    // 这正是"读最新图"会错的地方：解产生于第 5 个关键帧，事件里的快照就是那一刻的图；
    // 跑完之后真源已经有 10 个节点，但这份解的身份对应的仍然是 5 个节点的那张图。
    CHECK(seen.front().solution->nodeCount() == 5);
    CHECK(seen.front().identity().covered_until == 5);
    CHECK(system.resources().graph.snapshot()->nodeCount() == 10);
}

TEST_CASE("处理进度回到打包器，原始 IMU 生成之后就回收", "[modules][system][watermark]") {
    modules::SlamSystem system(offlineOptions());
    auto& io = system.setSource(std::make_unique<SyntheticSource>(6));
    system.setOdometry<ConstantVelocityOdometry>();
    system.init();
    system.runOffline();

    CHECK(io.publishedPackets() == 6);
    CHECK(io.consumedPackets() == 6);
    CHECK(io.packetizer().inFlight() == 0);
    CHECK(io.packetizer().imuBacklog() <= 2);  // 只留跨越生成水位的边界样本
}

TEST_CASE("离线虚拟时钟随数据时间推进", "[modules][system][clock]") {
    // 只喂包不推时钟的话，虚拟时钟一直停在 0，
    // 依赖定时器的业务就完全没有被端到端路径覆盖。
    modules::SlamSystem system(offlineOptions());
    system.setSource(std::make_unique<SyntheticSource>(10));
    system.setOdometry<ConstantVelocityOdometry>();
    system.init();

    CHECK(system.runtime().clock().now().count() == 0);
    system.runOffline();

    // 合成源每 10ms 一帧，第 10 帧在 100ms
    CHECK(system.runtime().clock().now().count() == 100'000'000);
}

TEST_CASE("回环约束一次送达进图，不需要回执", "[modules][system][constraint]") {
    modules::SlamSystem system(offlineOptions());
    system.setSource(std::make_unique<SyntheticSource>(12));
    system.setOdometry<ConstantVelocityOdometry>();
    auto& recognizer = system.setPlaceRecognizer<ScriptedPlaceRecognizer>();
    auto& optimizer = system.setGraphOptimizer<ShrinkingOptimizer>();
    system.init();
    system.runOffline();

    CHECK(recognizer.loopsFound() == 1);
    CHECK(optimizer.committedConstraints() == 1);
    CHECK(optimizer.rejectedConstraints() == 0);
}

TEST_CASE("解不可用就不进真源，也不发校正", "[modules][graph]") {
    // "没收敛"与"数值失败"是两回事：前者的解仍可用，后者会把发散的位姿带进图里
    fw::PoseGraphData graph;
    fw::PoseGraphNode node;
    node.id = fw::KeyframeId{1};
    node.T_world_body = SE3d::Identity();
    graph.addNode(node);

    auto problem = graph.toProblem();
    problem.nodes[0].id = fw::KeyframeId{7};  // 换了身份
    CHECK_THROWS_AS(graph.adoptSolution(problem), std::logic_error);

    auto diverged = graph.toProblem();
    diverged.nodes[0].T_world_body =
        SE3d(Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
             Eigen::Quaterniond::Identity());
    CHECK_THROWS_AS(graph.adoptSolution(diverged), std::logic_error);

    auto more = graph.toProblem();
    more.nodes.push_back(node);
    CHECK_THROWS_AS(graph.adoptSolution(more), std::logic_error);
}

namespace {

/// 永不结束的来源：每次都给一帧扫描。
class EndlessScanSource final : public fw::ISensorSource {
public:
    std::optional<fw::SensorDatum> next() override {
        auto scan = std::make_shared<fw::LidarScan>();
        scan->sensor = "lidar";
        scan->reference_time = fw::Timestamp{++count_ * 100'000'000};
        scan->coverage = fw::TimeRange{scan->reference_time - fw::Duration{100'000'000},
                                       scan->reference_time};
        scan->points.push(1.0F, 0.0F, 0.0F);
        return fw::SensorDatum{fw::Ptr<fw::LidarScan>(scan)};
    }
    [[nodiscard]] std::string name() const override { return "EndlessScanSource"; }
    [[nodiscard]] std::size_t reads() const noexcept { return count_; }

private:
    std::int64_t count_ = 0;
};

}  // namespace

TEST_CASE("pump 按预算返回，不空转读来源", "[modules][sensor_io]") {
    comm::Runtime runtime(comm::RuntimeOptions{true});
    modules::SlamResources resources;
    auto source = std::make_unique<EndlessScanSource>();
    auto* raw = source.get();

    fw::PacketizerPolicy policy;
    policy.imu = fw::ImuRequirement::kNotRequired;
    auto& io = runtime.addModule<modules::SensorIoModule>("/slam/sensor_io", std::move(source),
                                                          policy, resources);
    struct Idle final : comm::Module {
        void onInit() override {
            subscribe<fw::SensorPacket>(modules::topics::kSensorPacket, 0, [](const fw::SensorPacket&) {});
        }
    };
    runtime.addModule<Idle>("/idle");
    runtime.init();

    const auto first = io.pump(3);
    CHECK(first.published == 3);
    CHECK(raw->reads() == 3);  // 够了就停，不多读
}

// ── 在线：按数据时间的节奏放行 ───────────────────────────────────────────────

/// 记下每一帧被跟踪时的墙钟时刻。只在里程计线程上写，运行终结之后才读。
class TimingOdometry final : public plugin::OdometryBase {
public:
    [[nodiscard]] std::string name() const override { return "TimingOdometry"; }
    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }

    std::vector<std::pair<fw::Timestamp, std::chrono::steady_clock::time_point>> seen;

protected:
    plugin::TrackResult track(const fw::SensorPacket& packet) override {
        seen.emplace_back(packet.reference_time, std::chrono::steady_clock::now());
        return {};
    }
};

TEST_CASE("在线按数据时间的节奏放行：不提前，读完就停", "[modules][system][online]") {
    modules::SlamSystemOptions options;
    options.offline = false;
    modules::SlamSystem system(options);
    system.setSource(std::make_unique<SyntheticSource>(20));  // 数据从 0 到 200 ms
    auto& odometry = system.setOdometry<TimingOdometry>();
    system.init();

    const auto start = std::chrono::steady_clock::now();
    const std::size_t packets = system.runOnline();
    const auto elapsed = std::chrono::steady_clock::now() - start;
    system.shutdown();

    CHECK(packets == 20);
    CHECK(system.result().outcome.complete());
    const auto* algorithm = dynamic_cast<const TimingOdometry*>(&odometry.algorithm());
    REQUIRE(algorithm != nullptr);
    REQUIRE(algorithm->seen.size() == 20);
    // 回放原点不早于 start，数据时刻为 t 的帧不早于 原点 + t 放行：这是下界，不需要容差
    for (const auto& [time, wall] : algorithm->seen) CHECK(wall - start >= time);
    CHECK(elapsed >= std::chrono::milliseconds(200));
    CHECK(elapsed < std::chrono::seconds(10));  // 读完就停，不等超时
}

TEST_CASE("runOnline 要在线模式", "[modules][system][online]") {
    modules::SlamSystem system;  // 默认离线
    system.setSource(std::make_unique<SyntheticSource>(2));
    system.setOdometry<TimingOdometry>();
    system.init();
    CHECK_THROWS_AS(system.runOnline(), std::logic_error);
}
