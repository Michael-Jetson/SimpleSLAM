/// @file simpleslam_run.cpp
/// L1 入口（蓝图三层使用深度 L1）：`simpleslam_run <config.yaml>`
///
/// 配置驱动组装 KITTI→LoIcp→OfflineRunner，跑离线回放，导出 TUM 轨迹 +
/// resolved config。薄壳：所有装配逻辑在 runner 库的 runOfflineApp（可测）。

#include <SimpleSLAM/core/infra/config.hpp>
#include <SimpleSLAM/runner/offline_app.hpp>

#include <cstdio>
#include <exception>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "用法: simpleslam_run <config.yaml>\n");
        return 2;
    }
    try {
        const auto cfg = simpleslam::Config::load(argv[1]);
        const auto result = simpleslam::runOfflineApp(cfg);
        std::printf("处理 %zu 帧（失败 %zu），关键帧 %zu，用时 %.3fs\n",
                    result.frames_processed, result.frames_failed,
                    result.keyframes, result.elapsed_seconds);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[simpleslam_run] 致命错误: %s\n", e.what());
        return 1;
    }
}
