// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 把评测与配置接到系统上跑一遍。
///
/// 这个文件是"写算法的人只需要写算法"的验收：从数据源到精度数字、
/// 从一次运行到可复现的第二次运行，中间没有一步需要算法作者自己搭。

#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/eval/metrics.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>
#include <SimpleSLAM/app/bindings.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <memory>

using namespace simpleslam;
using Catch::Approx;

namespace {

constexpr std::int64_t kImuPeriodNs = 5'000'000;    // 200 Hz
constexpr std::int64_t kScanPeriodNs = 100'000'000; // 10 Hz
constexpr double kStepM = 0.5;                      // 每扫描前进 0.5 米

/// 合成来源：匀速直线 + IMU。真值由同一组参数解析给出，因此精度应当接近零。
class SyntheticSource final : public fw::ISensorSource {
public:
    explicit SyntheticSource(int scans) : scans_(scans) {}

    std::optional<fw::SensorDatum> next() override {
        if (emitted_scans_ >= scans_) return std::nullopt;
        if (next_imu_ns_ <= next_scan_ns_) {
            auto sample = std::make_shared<fw::ImuSample>();
            sample->time = fw::Timestamp{next_imu_ns_};
            sample->accel = {0.0, 0.0, 9.81};
            next_imu_ns_ += kImuPeriodNs;
            return fw::SensorDatum{fw::Ptr<fw::ImuSample>(sample)};
        }
        auto scan = std::make_shared<fw::LidarScan>();
        scan->reference_time = fw::Timestamp{next_scan_ns_};
        scan->coverage = fw::TimeRange{fw::Timestamp{next_scan_ns_ - kScanPeriodNs},
                                       fw::Timestamp{next_scan_ns_}};
        scan->sensor = "synthetic";
        for (int i = 0; i < 16; ++i) {
            scan->points.push(static_cast<float>(i), 0.0F, 0.0F);
            scan->points.time_offset_ns.push_back(static_cast<std::int32_t>(i) * 1000);
        }
        next_scan_ns_ += kScanPeriodNs;
        ++emitted_scans_;
        return fw::SensorDatum{fw::Ptr<fw::LidarScan>(scan)};
    }

    [[nodiscard]] std::string name() const override { return "SyntheticSource"; }

private:
    int scans_;
    int emitted_scans_ = 0;
    std::int64_t next_imu_ns_ = 0;
    std::int64_t next_scan_ns_ = kScanPeriodNs;
};

/// 读到一半就断了的来源：数据给到一半，终态报不完整（文件被截断时就是这样）。
class CutShortSource final : public fw::ISensorSource {
public:
    explicit CutShortSource(int scans) : inner_(scans) {}
    std::optional<fw::SensorDatum> next() override { return inner_.next(); }
    [[nodiscard]] std::string name() const override { return "CutShortSource"; }
    [[nodiscard]] fw::SourceOutcome outcome() const override {
        return fw::SourceOutcome::kIncomplete;
    }

private:
    SyntheticSource inner_;
};

/// 与合成运动一致的里程计。写真实算法时换掉 track() 即可。
class StraightLineOdometry final : public plugin::OdometryBase {
public:
    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }

protected:
    plugin::TrackResult track(const fw::SensorPacket& packet) override {
        ++frames_;
        const SE3d pose(Vec3d(static_cast<double>(frames_) * kStepM, 0.0, 0.0),
                        Eigen::Quaterniond::Identity());
        plugin::TrackResult result;
        result.status = fw::TrackingStatus::kTracking;
        result.T_odom_body = pose;
        result.registration_information = Mat6d::Identity() * 100.0;
        result.registration_information_source = fw::InformationSource::kPropagated;
        if (hasKeyframe()) {
            fw::RelativeConstraint constraint;
            constraint.from_time = lastKeyframeTime();
            constraint.to_time = packet.reference_time;
            constraint.measurement = lastKeyframePose().inverse().compose(pose);
            constraint.information = Mat6d::Identity() * 25.0;
            constraint.source = fw::InformationSource::kPropagated;
            result.constraint_to_last_keyframe = constraint;
        }
        result.suggest_keyframe = (frames_ % 2 == 0);
        return result;
    }
    void onReset() override { frames_ = 0; }

private:
    std::uint64_t frames_ = 0;
};

/// 真值：与合成运动解析一致。时刻取每一包的参考时刻。
fw::Trajectory groundTruth(int scans) {
    fw::Trajectory trajectory;
    trajectory.setFrame(fw::TrajectoryFrame::kWorld);
    for (int i = 1; i <= scans; ++i) {
        trajectory.push(fw::Timestamp{static_cast<std::int64_t>(i) * kScanPeriodNs},
                        SE3d(Vec3d(static_cast<double>(i) * kStepM, 0.0, 0.0),
                             Eigen::Quaterniond::Identity()));
    }
    return trajectory;
}

modules::SlamSystemOptions offlineOptions() {
    modules::SlamSystemOptions options;
    options.offline = true;
    return options;
}

std::filesystem::path tempFile(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}

}  // namespace

TEST_CASE("跑完就能拿到三条轨迹", "[system][eval]") {
    modules::SlamSystem system(offlineOptions());
    system.setSource(std::make_unique<SyntheticSource>(20));
    system.setOdometry<StraightLineOdometry>();
    system.init();
    system.runOffline();

    auto* trajectory = system.trajectory();
    REQUIRE(trajectory != nullptr);
    CHECK(trajectory->odometry().size() == 20);
    CHECK(trajectory->online().size() == 20);
    CHECK(trajectory->skipped() == 0);
    CHECK(trajectory->odometry().frame() == fw::TrajectoryFrame::kOdom);
    CHECK(trajectory->online().frame() == fw::TrajectoryFrame::kWorld);
    CHECK(trajectory->odometry().pathLength() == Approx(19.0 * kStepM));

    // 没有后端时关键帧轨迹为空：位姿图没有写者，这是事实而不是缺陷
    CHECK(trajectory->keyframes().empty());
}

TEST_CASE("对着真值算出的 ATE 接近零", "[system][eval]") {
    modules::SlamSystem system(offlineOptions());
    system.setSource(std::make_unique<SyntheticSource>(30));
    system.setOdometry<StraightLineOdometry>();
    system.init();
    system.runOffline();

    const auto estimate = system.trajectory()->odometry();
    const auto truth = groundTruth(30);

    eval::AteOptions options;
    options.alignment = eval::AlignmentKind::kSe3;
    options.tolerance = std::chrono::milliseconds(5);
    const auto ate = eval::absoluteTrajectoryError(estimate, truth, options);

    CHECK(ate.pairs == 30);
    CHECK(ate.coverage() == Approx(1.0));
    CHECK(ate.translation.rmse == Approx(0.0).margin(1e-9));
    CHECK(ate.rotation_deg.rmse == Approx(0.0).margin(1e-7));

    eval::RpeOptions rpe_options;
    rpe_options.delta_time = std::chrono::milliseconds(500);
    const auto rpe = eval::relativePoseError(estimate, truth, rpe_options);
    CHECK(rpe.pairs > 0);
    CHECK(rpe.translation.rmse == Approx(0.0).margin(1e-9));
}

TEST_CASE("轨迹导出之后能被再读回来", "[system][eval][io]") {
    modules::SlamSystem system(offlineOptions());
    system.setSource(std::make_unique<SyntheticSource>(12));
    system.setOdometry<StraightLineOdometry>();
    system.init();
    system.runOffline();

    const auto path = tempFile("simpleslam_system_estimate.tum");
    eval::writeTrajectory(path.string(), system.trajectory()->odometry(),
                          eval::TrajectoryFormat::kTum);
    const auto loaded = eval::readTrajectory(path.string(), eval::TrajectoryFormat::kTum);

    REQUIRE(loaded.size() == 12);
    const auto ate = eval::absoluteTrajectoryError(loaded, groundTruth(12));
    CHECK(ate.translation.rmse == Approx(0.0).margin(1e-6));
    std::filesystem::remove(path);
}

TEST_CASE("同一份输入离线跑两次，结果逐位相同", "[system][determinism]") {
    auto runAndCollect = [](std::unique_ptr<fw::ISensorSource> source) {
        modules::SlamSystem system(offlineOptions());
        system.setSource(std::move(source));
        system.setOdometry<StraightLineOdometry>();
        system.init();
        system.runOffline();
        std::vector<double> values;
        for (const auto& sample : system.trajectory()->odometry()) {
            values.push_back(static_cast<double>(sample.time.count()));
            values.push_back(sample.pose.translation().x());
            values.push_back(sample.pose.translation().y());
            values.push_back(sample.pose.translation().z());
        }
        return values;
    };

    const auto first = runAndCollect(std::make_unique<SyntheticSource>(16));
    const auto second = runAndCollect(std::make_unique<SyntheticSource>(16));
    REQUIRE(!first.empty());
    REQUIRE(second.size() == first.size());
    CHECK(second == first);  // 逐位相同，不是近似相等
}

TEST_CASE("配置驱动打包器：纯 LiDAR 声明不需要 IMU", "[system][config]") {
    const auto config = config::loadTaskString(R"(
task:
  mode: offline
system:
  packetizer:
    imu_required: false
)", "<string>").config;
    CHECK_FALSE(config.system.packetizer.imu_required);

    modules::SlamSystem system(app::toSystemOptions(config));
    auto& io = system.setSource(std::make_unique<SyntheticSource>(8),
                                app::toPacketizerPolicy(config.system.packetizer));
    system.setOdometry<StraightLineOdometry>();
    system.init();
    system.runOffline();

    CHECK(io.publishedPackets() == 8);
    CHECK(system.trajectory()->odometry().size() == 8);
}

TEST_CASE("配置的运行期可调部分进参数存储", "[system][config]") {
    const auto config = config::loadTaskString(
        "system:\n  modules:\n    graph_optimizer:\n      odometry_translation_weight: 1.7\n",
        "<string>").config;
    modules::SlamSystem system(offlineOptions());
    app::applyToParameters(config, system.runtime().params());
    system.setSource(std::make_unique<SyntheticSource>(4));
    system.setOdometry<StraightLineOdometry>();
    system.init();

    const auto snapshot = system.runtime().params().snapshot();
    // 模块级参数是那个模块的私有名：模块用 ~odometry_translation_weight 读，不认识配置布局
    CHECK(snapshot->get<double>("/slam/graph_optimizer/odometry_translation_weight") == 1.7);
}

TEST_CASE("运行期改参数走的是同一套校验", "[config][params]") {
    // 只保证类型一致是不够的：启动时会被 validateConfig 拒绝的值，
    // 运行期不该能直接改进去
    config::TaskConfig config;
    comm::ParameterStore store;
    app::applyToParameters(config, store);

    const auto name =
        app::configParamName("system.modules.graph_optimizer.odometry_translation_weight");
    const auto bad = store.commit({comm::ParamChange{name, comm::ParamValue{-1.0}}});
    CHECK_FALSE(bad.ok());
    CHECK(bad.status == comm::ParamCommit::kRejectedValue);
    CHECK(store.snapshot()->revision == 0);  // 整批拒绝，版本不变

    const auto good = store.commit({comm::ParamChange{name, comm::ParamValue{2.0}}});
    REQUIRE(good.ok());
    REQUIRE(good.snapshot != nullptr);
    CHECK(good.snapshot->revision == good.revision);  // 带回的是本次提交那一份
}

TEST_CASE("来源读到一半就断了：运行结局要说出来", "[system][outcome]") {
    // 同样的数字，一个是跑完全程测出来的，一个是只跑了前半段
    modules::SlamSystem system(offlineOptions());
    system.setSource(std::make_unique<CutShortSource>(6));
    system.setOdometry<StraightLineOdometry>();
    system.init();
    system.runOffline();

    const auto outcome = system.runOutcome();
    CHECK(outcome.input == fw::SourceOutcome::kIncomplete);
    CHECK_FALSE(outcome.inputComplete());
    CHECK_FALSE(outcome.complete());
    CHECK_FALSE(outcome.reasons().empty());
}
