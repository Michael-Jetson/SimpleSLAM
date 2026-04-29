#include <SimpleSLAM/odometry/odometry_base.hpp>
#include <SimpleSLAM/core/infra/topic_hub.hpp>
#include <SimpleSLAM/core/infra/topic_names.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace simpleslam;

namespace {

class MockOdometry final : public OdometryBase {
public:
    int process_count = 0;

    OdometryResult processLidar(const LidarScan& scan) override {
        ++process_count;
        OdometryResult result;
        result.timestamp = scan.timestamp;
        result.status = TrackingStatus::Tracking;
        result.is_keyframe = (process_count % 3 == 0);

        publishResult(result);

        if (result.is_keyframe) {
            KeyframeData kf;
            kf.id = static_cast<uint64_t>(process_count);
            kf.timestamp = scan.timestamp;
            kf.scan = std::make_shared<const LidarScan>(scan);
            publishKeyframe(kf);
        }

        return result;
    }

    void reset() override { process_count = 0; }
    [[nodiscard]] std::string_view name() const override { return "MockOdometry"; }
};

LidarScan makeScan(Timestamp ts) {
    LidarScan scan;
    scan.timestamp = ts;
    scan.points.push_back(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    return scan;
}

}  // namespace

TEST_CASE("OdometryBase initialize 创建 Publisher", "[odometry_base]") {
    TopicHub hub(true);
    MockOdometry odom;
    odom.initialize(hub);

    REQUIRE(hub.hasTopic(std::string(topic_names::kSlamOdometry)));
    REQUIRE(hub.hasTopic(std::string(topic_names::kSlamKeyframe)));
}

TEST_CASE("OdometryBase processLidarImu 默认委托 processLidar", "[odometry_base]") {
    TopicHub hub(true);
    MockOdometry odom;
    odom.initialize(hub);

    auto scan = makeScan(1.0);
    std::vector<ImuSample> imu;

    auto result = odom.processLidarImu(scan, imu);
    REQUIRE(result.timestamp == 1.0);
    REQUIRE(odom.process_count == 1);
}

TEST_CASE("OdometryBase publishResult 触发 on_after_update", "[odometry_base]") {
    TopicHub hub(true);
    MockOdometry odom;
    odom.initialize(hub);

    int callback_count = 0;
    odom.on_after_update.connect(
        [&](const OdometryResult&) { ++callback_count; });

    odom.processLidar(makeScan(1.0));
    REQUIRE(callback_count == 1);

    odom.processLidar(makeScan(2.0));
    REQUIRE(callback_count == 2);
}

TEST_CASE("OdometryBase publishKeyframe 触发 on_keyframe", "[odometry_base]") {
    TopicHub hub(true);
    MockOdometry odom;
    odom.initialize(hub);

    int kf_callback_count = 0;
    odom.on_keyframe.connect(
        [&](const KeyframeData&) { ++kf_callback_count; });

    odom.processLidar(makeScan(1.0));
    odom.processLidar(makeScan(2.0));
    REQUIRE(kf_callback_count == 0);

    odom.processLidar(makeScan(3.0));
    REQUIRE(kf_callback_count == 1);
}

TEST_CASE("OdometryBase publishResult 发布到 Topic", "[odometry_base]") {
    TopicHub hub(true);
    MockOdometry odom;
    odom.initialize(hub);

    int topic_count = 0;
    auto sub = hub.subscribeImpl<OdometryResult>(
        std::string(topic_names::kSlamOdometry),
        [&](MsgPtr<OdometryResult>) { ++topic_count; });

    odom.processLidar(makeScan(1.0));
    hub.drainAll();

    REQUIRE(topic_count == 1);
}

TEST_CASE("OdometryBase name 返回正确名称", "[odometry_base]") {
    MockOdometry odom;
    REQUIRE(odom.name() == "MockOdometry");
}

TEST_CASE("OdometryBase on_after_prediction slot", "[odometry_base]") {
    MockOdometry odom;

    int prediction_count = 0;
    odom.on_after_prediction.connect(
        [&](const SE3d&) { ++prediction_count; });

    SE3d pose;
    odom.on_after_prediction.emit(pose);
    REQUIRE(prediction_count == 1);
}
