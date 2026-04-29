#pragma once

/// @file offline_runner.hpp
/// 离线数据集回放模式的主循环编排器
///
/// 单线程顺序执行：拉数据 → processLidar → drainAll → 循环。
/// 确定性、可断点调试、不丢帧。
/// 管理 TopicHub 全局单例的生命周期（init/shutdown）。

#include <SimpleSLAM/backend/service_base.hpp>
#include <SimpleSLAM/odometry/odometry_base.hpp>
#include <SimpleSLAM/resources/trajectory.hpp>
#include <SimpleSLAM/sensor_io/sensor_source.hpp>

#include <cstddef>
#include <memory>
#include <vector>

namespace simpleslam {

/// 一次离线运行的统计结果
struct RunResult {
    size_t frames_processed{0};
    size_t keyframes{0};
    double elapsed_seconds{0.0};
};

/// 离线 Runner——从数据源拉取数据，驱动 Odometry，协调后端服务
class OfflineRunner final {
public:
    /// 构造时初始化 TopicHub 全局单例（离线模式）并初始化 Odometry
    OfflineRunner(std::unique_ptr<ISensorSource> source,
                  std::unique_ptr<OdometryBase> odometry);

    /// 析构时关闭 Odometry 并销毁 TopicHub 全局单例
    ~OfflineRunner();

    /// 注册后端服务（在 run() 之前调用）
    void addService(std::unique_ptr<ServiceBase> service);

    /// 执行主循环，max_frames=0 表示处理全部帧
    [[nodiscard]] RunResult run(size_t max_frames = 0);

    [[nodiscard]] Trajectory& trajectory() { return trajectory_; }
    [[nodiscard]] const Trajectory& trajectory() const { return trajectory_; }

    OfflineRunner(const OfflineRunner&) = delete;
    OfflineRunner& operator=(const OfflineRunner&) = delete;
    OfflineRunner(OfflineRunner&&) = delete;
    OfflineRunner& operator=(OfflineRunner&&) = delete;

private:
    std::unique_ptr<ISensorSource> source_;
    std::unique_ptr<OdometryBase> odometry_;
    std::vector<std::unique_ptr<ServiceBase>> services_;
    Trajectory trajectory_;
};

}  // namespace simpleslam
