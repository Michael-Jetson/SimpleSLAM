#pragma once

/// @file offline_app.hpp
/// L1 配置驱动离线应用（蓝图三层使用深度 L1：改 YAML 就跑）。
///
/// 把"YAML → 组装 KITTI + LoIcp + OfflineRunner → 跑 → 导出轨迹/落盘配置"
/// 封装为可测函数，simpleslam_run 仅作薄壳。当前只接一条真垂直（KITTI + 纯 LO）；
/// 多模块通用 Factory(type) 留待 ≥2 实现时再抽象（法则一，R3+）。

#include <SimpleSLAM/odometry/lo_icp_odometry.hpp>
#include <SimpleSLAM/runner/offline_runner.hpp>

#include <cstddef>
#include <string>

namespace simpleslam {

/// 离线 L1 应用配置（从 YAML 解析）。
struct OfflineAppConfig {
    std::string dataset_path;       ///< KITTI 序列目录（含 velodyne/ + times.txt）
    std::string output_dir = ".";   ///< 轨迹 + resolved config 输出目录
    std::size_t max_frames = 0;     ///< 0 = 全部
    LoIcpConfig lo;                 ///< LO-ICP 参数（缺省走结构体默认）
};

/// 从 Config 解析 OfflineAppConfig（缺省走默认；存在但类型不符抛 ConfigError）。
[[nodiscard]] OfflineAppConfig loadOfflineAppConfig(const Config& cfg);

/// 组装 KITTI→LoIcp→OfflineRunner：落盘 resolved config、跑、导出 TUM 轨迹。
/// 返回运行统计。供 simpleslam_run 与集成测试共用。
RunResult runOfflineApp(const Config& cfg);

}  // namespace simpleslam
