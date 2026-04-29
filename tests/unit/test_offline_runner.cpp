#include <SimpleSLAM/runner/offline_runner.hpp>
#include <SimpleSLAM/backend/service_base.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace simpleslam;

namespace {

class MockSource final : public ISensorSource {
public:
    explicit MockSource(int frame_count) : total_(frame_count) {}

    [[nodiscard]] bool hasNext() const override { return current_ < total_; }

    std::optional<LidarScan> nextScan() override {
        if (current_ >= total_) return std::nullopt;
        LidarScan scan;
        scan.timestamp = static_cast<double>(current_);
        scan.points.push_back(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
        ++current_;
        return scan;
    }

    [[nodiscard]] Timestamp currentTimestamp() const override {
        return static_cast<double>(current_);
    }

private:
    int total_;
    int current_ = 0;
};

class MockOdometry final : public OdometryBase {
public:
    int process_count = 0;

    OdometryResult processLidar(const LidarScan& scan) override {
        ++process_count;
        OdometryResult result;
        result.timestamp = scan.timestamp;
        result.status = TrackingStatus::Tracking;
        result.is_keyframe = (process_count % 5 == 0);
        publishResult(result);
        return result;
    }

    void reset() override { process_count = 0; }
    [[nodiscard]] std::string_view name() const override { return "MockOdometry"; }
};

class MockService final : public ServiceBase {
public:
    bool initialized = false;
    bool shut_down = false;

    MockService() : ServiceBase("MockService") {}

    void initialize(TopicHub& hub) override {
        (void)hub;
        initialized = true;
    }

    void shutdown() override {
        shut_down = true;
    }
};

}  // namespace

TEST_CASE("OfflineRunner 处理所有帧", "[offline_runner]") {
    auto source = std::make_unique<MockSource>(10);
    auto odom = std::make_unique<MockOdometry>();
    auto* odom_ptr = odom.get();

    OfflineRunner runner(std::move(source), std::move(odom));
    auto result = runner.run();

    REQUIRE(result.frames_processed == 10);
    REQUIRE(odom_ptr->process_count == 10);
    REQUIRE(result.elapsed_seconds > 0.0);
}

TEST_CASE("OfflineRunner max_frames 限制", "[offline_runner]") {
    auto source = std::make_unique<MockSource>(10);
    auto odom = std::make_unique<MockOdometry>();

    OfflineRunner runner(std::move(source), std::move(odom));
    auto result = runner.run(5);

    REQUIRE(result.frames_processed == 5);
}

TEST_CASE("OfflineRunner trajectory 记录位姿", "[offline_runner]") {
    auto source = std::make_unique<MockSource>(8);
    auto odom = std::make_unique<MockOdometry>();

    OfflineRunner runner(std::move(source), std::move(odom));
    runner.run();

    REQUIRE(runner.trajectory().size() == 8);
}

TEST_CASE("OfflineRunner addService 生命周期", "[offline_runner]") {
    auto source = std::make_unique<MockSource>(3);
    auto odom = std::make_unique<MockOdometry>();
    auto svc = std::make_unique<MockService>();
    auto* svc_ptr = svc.get();

    OfflineRunner runner(std::move(source), std::move(odom));
    runner.addService(std::move(svc));

    REQUIRE_FALSE(svc_ptr->initialized);
    runner.run();
    REQUIRE(svc_ptr->initialized);
    REQUIRE(svc_ptr->shut_down);
}

TEST_CASE("OfflineRunner 空数据源", "[offline_runner]") {
    auto source = std::make_unique<MockSource>(0);
    auto odom = std::make_unique<MockOdometry>();

    OfflineRunner runner(std::move(source), std::move(odom));
    auto result = runner.run();

    REQUIRE(result.frames_processed == 0);
    REQUIRE(result.keyframes == 0);
    REQUIRE(runner.trajectory().empty());
}
