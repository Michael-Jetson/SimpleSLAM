/// @file offline_app.cpp
/// L1 配置驱动离线应用实现（ADR / R1c）。

#include <SimpleSLAM/runner/offline_app.hpp>

#include <SimpleSLAM/sensor_io/kitti_source.hpp>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <memory>

namespace simpleslam {

OfflineAppConfig loadOfflineAppConfig(const Config& cfg) {
    OfflineAppConfig app;
    app.dataset_path = cfg.get<std::string>("dataset.path", std::string{});
    app.output_dir = cfg.get<std::string>("output.dir", std::string{"."});
    const int mf = cfg.get<int>("runtime.max_frames", 0);
    app.max_frames = mf > 0 ? static_cast<std::size_t>(mf) : 0;

    // LO-ICP 参数：缺省走 LoIcpConfig 结构体默认（存在但类型不符则 get 抛 ConfigError）
    app.lo.downsample_voxel_size =
        cfg.get<float>("odometry.downsample_voxel_size", app.lo.downsample_voxel_size);
    app.lo.target.voxel_size =
        cfg.get<float>("odometry.target.voxel_size", app.lo.target.voxel_size);
    app.lo.target.max_correspondence_dist = cfg.get<double>(
        "odometry.target.max_correspondence_dist", app.lo.target.max_correspondence_dist);
    app.lo.solver.max_iterations =
        cfg.get<int>("odometry.solver.max_iterations", app.lo.solver.max_iterations);
    return app;
}

RunResult runOfflineApp(const Config& cfg) {
    namespace fs = std::filesystem;
    const auto app = loadOfflineAppConfig(cfg);

    if (!app.output_dir.empty()) {
        fs::create_directories(app.output_dir);
        // resolved config 落盘（KISS-ICP 式：配置与输出绑定，便于精确复现本次运行）
        std::ofstream out(fs::path(app.output_dir) / "resolved_config.yaml");
        out << YAML::Dump(cfg.root()) << '\n';
    }

    auto source = std::make_unique<KittiSource>(app.dataset_path);
    auto odom = std::make_unique<LoIcpOdometry>(app.lo, cfg.sub("odometry"));
    OfflineRunner runner(std::move(source), std::move(odom), cfg);

    auto result = runner.run(app.max_frames);

    if (!app.output_dir.empty()) {
        runner.trajectory().exportTUM(
            (fs::path(app.output_dir) / "trajectory.tum").string());
    }
    return result;
}

}  // namespace simpleslam
