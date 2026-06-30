#pragma once

/// @file offline_runner.hpp
/// 离线数据集回放模式的主循环编排器
///
/// 单线程顺序执行：拉数据 → processLidar → drainAll → 循环。
/// 确定性、可断点调试、不丢帧。
/// 持有 TopicHub 实例（注入式，无全局单例——ADR-001）。

#include <SimpleSLAM/backend/service_base.hpp>
#include <SimpleSLAM/core/infra/comm/topic.hpp>
#include <SimpleSLAM/core/infra/config.hpp>
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
    size_t frames_failed{0};   ///< processLidar 抛异常的帧数（>0 表示有帧被隔离丢弃）
    double elapsed_seconds{0.0};
};

/// 离线 Runner——从数据源拉取数据，驱动 Odometry，协调后端服务
class OfflineRunner final {
public:
    /// 构造时创建 TopicHub 实例并注入初始化 Odometry。
    /// cfg 提供 runtime.offline_mode（缺省 true=离线确定性）。
    OfflineRunner(std::unique_ptr<ISensorSource> source,
                  std::unique_ptr<OdometryBase> odometry,
                  const Config& cfg = {});

    /// 析构时关闭 Odometry
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
    TopicHub hub_;   ///< 本 Runner 私有的 comm 中枢（注入给 odometry/services），声明在先，先构造
    std::unique_ptr<ISensorSource> source_;
    std::unique_ptr<OdometryBase> odometry_;
    std::vector<std::unique_ptr<ServiceBase>> services_;
    Trajectory trajectory_;
};

}  // namespace simpleslam
