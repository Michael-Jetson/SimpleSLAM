/// @file offline_runner.cpp
/// OfflineRunner 实现——离线主循环 + TopicHub 生命周期管理

#include <SimpleSLAM/runner/offline_runner.hpp>

#include <SimpleSLAM/core/infra/topic_hub.hpp>

#include <cassert>
#include <chrono>

namespace simpleslam {

OfflineRunner::OfflineRunner(std::unique_ptr<ISensorSource> source,
                             std::unique_ptr<OdometryBase> odometry)
    : source_(std::move(source)), odometry_(std::move(odometry)) {
    assert(source_ && "ISensorSource must not be null");
    assert(odometry_ && "OdometryBase must not be null");

    TopicHub::init(true);
    odometry_->initialize(TopicHub::instance());
}

OfflineRunner::~OfflineRunner() {
    odometry_->shutdown();
    TopicHub::shutdown();
}

void OfflineRunner::addService(std::unique_ptr<ServiceBase> service) {
    assert(service && "ServiceBase must not be null");
    services_.push_back(std::move(service));
}

RunResult OfflineRunner::run(size_t max_frames) {
    for (auto& svc : services_) {
        svc->initialize(TopicHub::instance());
    }

    RunResult result;
    auto start_time = std::chrono::steady_clock::now();

    while (source_->hasNext()) {
        if (max_frames > 0 && result.frames_processed >= max_frames) break;

        auto scan = source_->nextScan();
        if (!scan) continue;

        auto odom_result = odometry_->processLidar(*scan);

        if (odom_result.isTracking()) {
            trajectory_.append(odom_result.timestamp, odom_result.pose);
        }

        if (odom_result.is_keyframe) {
            ++result.keyframes;
        }

        TopicHub::instance().drainAll();
        ++result.frames_processed;
    }

    for (auto& svc : services_) {
        svc->shutdown();
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    result.elapsed_seconds = std::chrono::duration<double>(elapsed).count();

    return result;
}

}  // namespace simpleslam
