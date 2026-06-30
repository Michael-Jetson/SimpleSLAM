/// @file test_offline_app.cpp
/// L1 walking skeleton 契约：YAML 解析 + 端到端组装跑通（KITTI→LoIcp→Runner）。

#include <SimpleSLAM/runner/offline_app.hpp>
#include <SimpleSLAM/core/infra/config.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>

using namespace simpleslam;
namespace fs = std::filesystem;

namespace {

/// 在 seq_dir 下写一个最小 KITTI 序列：velodyne/*.bin（每点 4 float=16B）+ times.txt
void writeKittiMini(const fs::path& seq_dir, int frames) {
    fs::create_directories(seq_dir / "velodyne");
    std::ofstream ts(seq_dir / "times.txt");
    for (int f = 0; f < frames; ++f) {
        char name[32];
        std::snprintf(name, sizeof(name), "%06d.bin", f);
        std::ofstream bin(seq_dir / "velodyne" / name, std::ios::binary);
        for (int i = -3; i <= 3; ++i)
            for (int j = -3; j <= 3; ++j) {
                // 非平面（鞍面 z=0.1·i·j）：给 z/roll/pitch 可观性，ICP 满秩可跟踪
                const float z = 0.1f * static_cast<float>(i * j);
                const float pt[4] = {static_cast<float>(i), static_cast<float>(j),
                                     z, 0.5f};
                bin.write(reinterpret_cast<const char*>(pt), sizeof(pt));
            }
        ts << (0.1 * f) << "\n";
    }
}

}  // namespace

TEST_CASE("loadOfflineAppConfig 从 YAML 解析字段", "[offline_app]") {
    YAML::Node node;
    node["dataset"]["path"] = "/foo/seq";
    node["output"]["dir"] = "/tmp/out";
    node["runtime"]["max_frames"] = 7;
    node["odometry"]["downsample_voxel_size"] = 2.0;
    node["odometry"]["target"]["voxel_size"] = 0.5;

    const auto cfg = Config::fromNode(node);
    const auto app = loadOfflineAppConfig(cfg);

    REQUIRE(app.dataset_path == "/foo/seq");
    REQUIRE(app.output_dir == "/tmp/out");
    REQUIRE(app.max_frames == 7);
    REQUIRE(app.lo.downsample_voxel_size == Catch::Approx(2.0f));
    REQUIRE(app.lo.target.voxel_size == Catch::Approx(0.5f));
}

TEST_CASE("runOfflineApp 端到端跑通并导出轨迹 + resolved config", "[offline_app]") {
    const auto base = fs::temp_directory_path() / "ss_offline_app_test";
    fs::remove_all(base);
    const auto seq = base / "seq";
    const auto out = base / "out";
    writeKittiMini(seq, 3);

    YAML::Node node;
    node["dataset"]["path"] = seq.string();
    node["output"]["dir"] = out.string();
    node["runtime"]["max_frames"] = 0;
    const auto cfg = Config::fromNode(node);

    const auto result = runOfflineApp(cfg);

    REQUIRE(result.frames_processed == 3);                 // 全部帧被处理
    REQUIRE(fs::exists(out / "trajectory.tum"));           // 轨迹导出
    REQUIRE(fs::file_size(out / "trajectory.tum") > 0);
    REQUIRE(fs::exists(out / "resolved_config.yaml"));     // 配置落盘（可复现）

    fs::remove_all(base);
}
