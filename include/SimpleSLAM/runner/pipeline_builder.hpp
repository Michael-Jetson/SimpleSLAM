#pragma once

/// @file pipeline_builder.hpp
/// 配置驱动的管道组装器——从 YAML 一键创建 OfflineRunner
///
/// 用户只需一个 YAML 配置文件和几行 C++ 代码即可跑通完整 SLAM 管道：
///
///   auto cfg = Config::load("pipeline.yaml");
///   auto runner = PipelineBuilder::fromConfig(cfg);
///   auto result = runner->run();
///   runner->trajectory().exportTUM("result.tum");
///
/// 前提：所有用到的组件已通过 SIMPLESLAM_REGISTER_MODULE / SIMPLESLAM_REGISTER_PLUGIN
/// 宏注册到全局 Registry 中（链接对应 .o 即自动注册）。
///
/// YAML 配置格式：
///   pipeline:
///     source:
///       type: kitti
///       path: /data/kitti/00
///     odometry:
///       type: lo_icp
///       max_iterations: 30
///     services:                     # 可选，无后端服务也可运行
///       - type: loop_closure
///         detector: { type: scan_context, num_sectors: 60 }
///       - type: pgo
///         optimizer: { type: gtsam_isam2 }

#include <SimpleSLAM/backend/service_base.hpp>
#include <SimpleSLAM/core/infra/config.hpp>
#include <SimpleSLAM/core/infra/logger.hpp>
#include <SimpleSLAM/core/infra/registry.hpp>
#include <SimpleSLAM/odometry/odometry_base.hpp>
#include <SimpleSLAM/runner/offline_runner.hpp>
#include <SimpleSLAM/sensor_io/sensor_source.hpp>

#include <memory>
#include <string>

namespace simpleslam {

/// 配置驱动的管道组装器
///
/// 从 YAML Config 读取 pipeline 配置，通过 Registry 创建各组件，
/// 组装成 OfflineRunner 并返回。返回 unique_ptr 因为 OfflineRunner 不可 move。
class PipelineBuilder final {
public:
    /// 从 Config 对象创建完整的 OfflineRunner
    ///
    /// @param cfg 包含 pipeline 节点的 Config 对象
    /// @return 组装完成的 OfflineRunner（已就绪，可直接调用 run()）
    /// @throws std::runtime_error 必要的 type 未注册时抛出
    /// @throws ConfigError 必要的配置项缺失时抛出
    static std::unique_ptr<OfflineRunner> fromConfig(const Config& cfg) {
        auto log = Logger::get("PipelineBuilder");

        // ── 创建数据源 ──
        auto source_type = cfg.get<std::string>("pipeline.source.type");
        log->info("Creating source: {}", source_type);
        auto source = Registry<std::unique_ptr<ISensorSource>>::create(
            source_type, cfg.node("pipeline.source"));

        // ── 创建里程计 ──
        auto odom_type = cfg.get<std::string>("pipeline.odometry.type");
        log->info("Creating odometry: {}", odom_type);
        auto odom = Registry<std::unique_ptr<OdometryBase>>::create(
            odom_type, cfg.node("pipeline.odometry"));

        // ── 组装 Runner ──
        auto runner = std::make_unique<OfflineRunner>(
            std::move(source), std::move(odom));

        // ── 创建后端服务（可选）──
        if (cfg.has("pipeline.services")) {
            auto services_node = cfg.node("pipeline.services");
            for (const auto& svc_node : services_node) {
                auto svc_type = svc_node["type"].as<std::string>();
                log->info("Creating service: {}", svc_type);
                auto svc = Registry<std::unique_ptr<ServiceBase>>::create(
                    svc_type, svc_node);
                runner->addService(std::move(svc));
            }
        }

        log->info("Pipeline assembled");
        return runner;
    }
};

}  // namespace simpleslam
