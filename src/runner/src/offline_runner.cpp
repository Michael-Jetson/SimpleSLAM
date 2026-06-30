/// @file offline_runner.cpp
/// OfflineRunner 实现——离线主循环 + 持有注入式 TopicHub 实例（ADR-001）

#include <SimpleSLAM/runner/offline_runner.hpp>

#include <SimpleSLAM/core/infra/comm/comm_config.hpp>
#include <SimpleSLAM/core/infra/comm/topic.hpp>

#include <cassert>
#include <chrono>
#include <cstdio>
#include <exception>

namespace simpleslam {

OfflineRunner::OfflineRunner(std::unique_ptr<ISensorSource> source,
                             std::unique_ptr<OdometryBase> odometry,
                             const Config& cfg)
    : hub_(loadOfflineMode(cfg)),
      source_(std::move(source)),
      odometry_(std::move(odometry)) {
    assert(source_ && "ISensorSource must not be null");
    assert(odometry_ && "OdometryBase must not be null");

    odometry_->initialize(hub_);
}

OfflineRunner::~OfflineRunner() {
    odometry_->shutdown();
}

void OfflineRunner::addService(std::unique_ptr<ServiceBase> service) {
    assert(service && "ServiceBase must not be null");
    services_.push_back(std::move(service));
}

RunResult OfflineRunner::run(size_t max_frames) {
    for (auto& svc : services_) {
        svc->initialize(hub_);
    }

    RunResult result;
    auto start_time = std::chrono::steady_clock::now();

    while (source_->hasNext()) {
        if (max_frames > 0 && result.frames_processed >= max_frames) break;

        auto scan = source_->nextScan();
        if (!scan) continue;

        // 单帧异常隔离：一帧 processLidar 抛出不应掀翻整轮（否则跳过 service shutdown、
        // 丢失 RunResult、且无从知道是哪帧）。记录并计数，继续下一帧。
        OdometryResult odom_result;
        try {
            odom_result = odometry_->processLidar(*scan);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[SimpleSLAM] 第 %zu 帧 processLidar 抛出: %s\n",
                         result.frames_processed, e.what());
            ++result.frames_failed;
            ++result.frames_processed;
            continue;
        }

        if (odom_result.isTracking()) {
            trajectory_.append(odom_result.timestamp, odom_result.pose);
        }

        if (odom_result.is_keyframe) {
            ++result.keyframes;
        }

        hub_.drainAll();
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
