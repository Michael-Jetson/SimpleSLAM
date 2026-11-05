// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 把评测、回放、配置三层接到系统上跑一遍。
///
/// 这个文件是"写算法的人只需要写算法"的验收：从数据源到精度数字、
/// 从一次运行到可复现的第二次运行，中间没有一步需要算法作者自己搭。

#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/eval/baseline.hpp>
#include <SimpleSLAM/eval/metrics.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>
#include <SimpleSLAM/modules/config_bindings.hpp>
#include <SimpleSLAM/record/bundle.hpp>
#include <SimpleSLAM/record/reader.hpp>
#include <SimpleSLAM/record/writer.hpp>

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
eval::Trajectory groundTruth(int scans) {
    eval::Trajectory trajectory;
    trajectory.setFrame(eval::TrajectoryFrame::kWorld);
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
    options.worker_threads = 1;
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
    CHECK(trajectory->odometry().frame() == eval::TrajectoryFrame::kOdom);
    CHECK(trajectory->online().frame() == eval::TrajectoryFrame::kWorld);
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

TEST_CASE("录制之后回放得到逐位相同的结果", "[system][record]") {
    const auto path = tempFile("simpleslam_system_run.ssrec");

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

    record::RecordMetadata metadata;
    metadata.name = "replay-determinism";
    metadata.config_hash = config::configHash(config::SlamConfig{});
    auto writer = std::make_shared<record::RecordWriter>(path.string(), metadata);

    const auto live = runAndCollect(std::make_unique<record::RecordingSource>(
        std::make_unique<SyntheticSource>(16), writer));
    writer->close();

    const auto replayed = runAndCollect(std::make_unique<record::ReplaySource>(path.string()));

    REQUIRE(replayed.size() == live.size());
    CHECK(replayed == live);  // 逐位相同，不是近似相等

    record::ReplaySource inspect(path.string());
    CHECK(inspect.metadata().name == "replay-determinism");
    CHECK(inspect.metadata().config_hash == config::configHash(config::SlamConfig{}));
    std::filesystem::remove(path);
}

TEST_CASE("配置驱动打包器：纯 LiDAR 声明不需要 IMU", "[system][config]") {
    const auto config = config::loadConfigString(R"(
runtime:
  offline: true
  worker_threads: 1
packetizer:
  imu_required: false
)");
    CHECK_FALSE(config.packetizer.imu_required);

    modules::SlamSystem system(modules::toSystemOptions(config));
    auto& io = system.setSource(std::make_unique<SyntheticSource>(8),
                                config::toPacketizerPolicy(config.packetizer));
    system.setOdometry<StraightLineOdometry>();
    system.init();
    system.runOffline();

    CHECK(io.publishedPackets() == 8);
    CHECK(system.trajectory()->odometry().size() == 8);
}

TEST_CASE("配置的运行期可调部分进参数存储", "[system][config]") {
    const auto config = config::loadConfigString("odometry:\n  keyframe_distance_m: 1.7\n");
    modules::SlamSystem system(offlineOptions());
    config::applyToParameters(config, system.runtime().params());
    system.setSource(std::make_unique<SyntheticSource>(4));
    system.setOdometry<StraightLineOdometry>();
    system.init();

    const auto snapshot = system.runtime().params().snapshot();
    CHECK(snapshot->get<double>("/config/odometry/keyframe_distance_m") == 1.7);
}

TEST_CASE("实验回放包把配置、输入、种子钉在一起", "[system][record][bundle]") {
    const auto record_path = tempFile("simpleslam_bundle_run.ssrec");
    auto config = config::SlamConfig{};
    config.name = "bundle-demo";
    config.seed = 7;

    {
        auto writer = std::make_shared<record::RecordWriter>(record_path.string());
        modules::SlamSystem system(modules::toSystemOptions(config));
        system.setSource(std::make_unique<record::RecordingSource>(
            std::make_unique<SyntheticSource>(10), writer));
        system.setOdometry<StraightLineOdometry>();
        system.init();
        system.runOffline();
        writer->close();

        const auto ate = eval::absoluteTrajectoryError(system.trajectory()->odometry(),
                                                       groundTruth(10));
        record::ExperimentBundle bundle;
        bundle.name = config.name;
        bundle.config_hash = config::configHash(config);
        bundle.seed = config.seed;
        std::uint64_t bytes = 0;
        bundle.inputs.push_back(record::InputDigest{
            "recording", record_path.string(), record::fileDigest(record_path.string(), &bytes),
            bytes});
        bundle.metrics = eval::toMetricMap(ate, eval::RpeResult{});
        record::writeBundle(tempFile("simpleslam_bundle_run.yaml").string(), bundle);
    }

    const auto bundle = record::readBundle(tempFile("simpleslam_bundle_run.yaml").string());
    CHECK(bundle.config_hash == config::configHash(config));
    CHECK(bundle.seed == 7);
    REQUIRE(bundle.input("recording") != nullptr);
    CHECK(bundle.input("recording")->bytes > 0);
    CHECK(bundle.metrics.at("ate.translation.rmse") == Approx(0.0).margin(1e-9));

    // 换一个配置，哈希就变——"这个结果是哪份配置跑的"因此可查
    auto other = config;
    other.odometry.max_iterations += 1;
    CHECK(config::configHash(other) != bundle.config_hash);

    std::filesystem::remove(record_path);
    std::filesystem::remove(tempFile("simpleslam_bundle_run.yaml"));
}

TEST_CASE("运行期改参数走的是同一套校验", "[config][params]") {
    // 只保证类型一致是不够的：启动时会被 validateConfig 拒绝的值，
    // 运行期不该能直接改进去
    config::SlamConfig config;
    comm::ParameterStore store;
    config::applyToParameters(config, store);

    const auto name = config::configParamName("odometry.keyframe_distance_m");
    const auto bad = store.commit({comm::ParamChange{name, comm::ParamValue{-1.0}}});
    CHECK_FALSE(bad.ok());
    CHECK(bad.status == comm::ParamCommit::kRejectedValue);
    CHECK(store.revision() == 0);  // 整批拒绝，版本不变

    const auto good = store.commit({comm::ParamChange{name, comm::ParamValue{2.0}}});
    REQUIRE(good.ok());
    REQUIRE(good.snapshot != nullptr);
    CHECK(good.snapshot->revision == good.revision);  // 带回的是本次提交那一份
}

TEST_CASE("回放不完整时实验产物要说出来", "[record][eval]") {
    // 同样的数字，一个是跑完全程测出来的，一个是只跑了前半段
    const auto path = std::filesystem::temp_directory_path() / "simpleslam_incomplete.ssrec";
    {
        record::RecordWriter writer(path.string());
        for (int i = 0; i < 3; ++i) {
            fw::LidarScan scan;
            scan.sensor = "lidar";
            scan.reference_time = fw::Timestamp{(i + 1) * 100'000'000};
            scan.coverage = fw::TimeRange{fw::Timestamp{i * 100'000'000}, scan.reference_time};
            scan.points.push(1.0F, 0.0F, 0.0F);
            writer.write(scan);
        }
        writer.close();
    }
    const auto full = static_cast<std::streamoff>(std::filesystem::file_size(path));
    std::filesystem::resize_file(path, static_cast<std::uintmax_t>(full - 28));  // 砍掉收尾记录

    modules::SlamSystem::Options options;
    options.offline = true;
    options.worker_threads = 1;
    modules::SlamSystem system(options);
    fw::PacketizerPolicy policy;
    policy.imu = fw::ImuRequirement::kNotRequired;
    system.setSource(std::make_unique<record::ReplaySource>(path.string()), policy);
    system.setOdometry<StraightLineOdometry>();
    system.init();
    system.runOffline();

    const auto outcome = system.runOutcome();
    CHECK(outcome.input == fw::SourceOutcome::kIncomplete);
    CHECK_FALSE(outcome.inputComplete());
    CHECK_FALSE(outcome.complete());
    CHECK_FALSE(outcome.reasons().empty());
    std::filesystem::remove(path);
}
