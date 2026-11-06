// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// @file eval_tool.cpp
/// simpleslam-eval：单独评测一条轨迹（别的系统跑出来的也行）。
///
/// 两条轨迹放进一份最小的任务记录，用内置的评测函数 ate、rpe 评测，结论写成同一种
/// `result.json`，操作台能把它和自己的结果放进一张表；不给 `--out` 时只打印结论、不留
/// 记录。只评测，不判通过与否：退出码 0 是评测完成，1 是出错，2 是用法错误。
/// 一条轨迹文件说不出产生它的运行有没有跑完，所以结论里的完整性写 null。

#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/run.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unistd.h>

namespace {

int usage() {
    std::cerr << R"(用法: simpleslam-eval <估计轨迹> <真值轨迹> [--out <目录>] [选项]

选项:
  --out <目录>                写出的任务记录：两条轨迹、结论 result.json、逐样本表格；
                              不给时只打印结论、不留记录
  --overwrite                 目录已有内容时照样写
  --format <tum|kitti>        两条轨迹的格式（默认按内容判定）
  --align <none|first_pose|se3|sim3>   对齐方式（默认 se3）
  --tolerance <秒>            最近邻关联容差（默认 0.02）
  --interpolate <秒>          改为把真值插值到估计时刻，括点间隔不超过给定秒数
  --rpe-delta <秒>            RPE 的时间间隔（默认 1.0）
  --rpe-distance <米>         改用距离间隔，并按每米归一
  --window <起秒> <止秒>      目标窗口：任务完成度按它算（不给就不算）
  --dataset <名字>            结论里的名字，进对比表时认它（默认取估计轨迹的文件名）
)";
    return 2;
}

/// 解析一个数值选项：整串都得是数，且有限。"nan"、"1e999"、"0.1s" 一律拒绝：
/// 它们会原样写进结论里评测函数的参数。
double finiteNumber(const std::string& flag, const std::string& text) {
    std::size_t used = 0;
    double value = 0.0;
    try {
        value = std::stod(text, &used);
    } catch (const std::exception&) {
        throw std::runtime_error("选项 " + flag + " 要一个数，得到 " + text);
    }
    if (used != text.size() || !std::isfinite(value)) {
        throw std::runtime_error("选项 " + flag + " 要一个有限的数，得到 " + text);
    }
    return value;
}

void requireRange(bool ok, const std::string& what) {
    if (!ok) throw std::runtime_error(what);
}

simpleslam::fw::Timestamp secondsToTimestamp(double seconds) {
    return simpleslam::fw::Timestamp{static_cast<std::int64_t>(std::llround(seconds * 1e9))};
}

}  // namespace

int main(int argc, char** argv) {
    namespace app = simpleslam::app;
    namespace config = simpleslam::config;
    if (argc < 3) return usage();
    app::TrajectoryEvaluation request;
    request.estimate = argv[1];
    request.groundtruth = argv[2];
    std::string alignment = "se3";
    double tolerance_s = 0.02;
    double interpolate_gap_s = 0.0;
    double rpe_delta_s = 1.0;
    double rpe_distance_m = 0.0;
    for (int i = 3; i < argc; ++i) {
        const std::string flag = argv[i];
        const auto value = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("选项 " + flag + " 缺少取值");
            return argv[++i];
        };
        try {
            if (flag == "--out") request.out = value();
            else if (flag == "--overwrite") request.overwrite = true;
            else if (flag == "--format") request.format = value();
            else if (flag == "--align") alignment = value();
            else if (flag == "--tolerance") tolerance_s = finiteNumber(flag, value());
            else if (flag == "--interpolate") interpolate_gap_s = finiteNumber(flag, value());
            else if (flag == "--rpe-delta") rpe_delta_s = finiteNumber(flag, value());
            else if (flag == "--rpe-distance") rpe_distance_m = finiteNumber(flag, value());
            else if (flag == "--window") {
                const double begin = finiteNumber(flag, value());
                const double end = finiteNumber(flag, value());
                requireRange(begin <= end, "--window 的起点晚于终点");
                request.window = simpleslam::fw::TimeRange{secondsToTimestamp(begin),
                                                           secondsToTimestamp(end)};
            }
            else if (flag == "--dataset") request.name = value();
            else if (flag == "--config-hash") {
                (void)value();  // 旧选项：身份不再按哈希核对，照样接受，提示一句
                std::cerr << "注意: --config-hash 不再核对（任务靠名字区分），忽略\n";
            }
            else if (flag == "--baselines" || flag == "--run-report"
                     || flag == "--assume-complete" || flag == "--allow-incomparable") {
                throw std::runtime_error(
                    "选项 " + flag + " 去掉了：simpleslam-eval 只评测、不过门禁；"
                    "自己跑出来的轨迹用 <入口> evaluate <任务记录> 补评");
            }
            else if (flag == "--report") {
                throw std::runtime_error("选项 --report 已经不用了：结论写进 --out 目录里的 "
                                         "result.json；不给 --out 时只打印、不留记录");
            }
            else return usage();
        } catch (const std::exception& e) {
            std::cerr << "错误: " << e.what() << '\n';
            return 2;
        }
    }
    try {
        requireRange(tolerance_s >= 0.0, "--tolerance 不能为负");
        requireRange(interpolate_gap_s >= 0.0, "--interpolate 不能为负");
        requireRange(rpe_delta_s > 0.0, "--rpe-delta 必须为正");
        requireRange(rpe_distance_m >= 0.0, "--rpe-distance 不能为负");
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << '\n';
        return 2;
    }

    // 选项换成 ate、rpe 两个评测函数的参数，随结论保存
    const bool interpolate = interpolate_gap_s > 0.0;
    config::PluginSpec ate{.type = "ate", .params = {}, .children = {}};
    ate.params["alignment"] = alignment;
    ate.params["association"] = std::string(interpolate ? "interpolate" : "nearest");
    ate.params["tolerance_s"] = tolerance_s;
    if (interpolate) ate.params["max_interpolation_gap_s"] = interpolate_gap_s;
    config::PluginSpec rpe{.type = "rpe", .params = ate.params, .children = {}};
    rpe.params.erase("alignment");
    if (rpe_distance_m > 0.0) {
        rpe.params["delta_m"] = rpe_distance_m;
        rpe.params["delta_s"] = 0.0;
    } else {
        rpe.params["delta_s"] = rpe_delta_s;
    }
    request.evaluators = {{"ate", ate}, {"rpe", rpe}};
    if (!request.out.empty()) {
        return app::evaluateTrajectories(request, app::builtinCatalogs(), std::cout, std::cerr);
    }
    // 速查：任务记录写在临时目录里，结论打印出来就删掉
    const auto scratch = std::filesystem::temp_directory_path()
                         / ("simpleslam-eval-" + std::to_string(::getpid()));
    request.out = scratch.string();
    request.overwrite = true;
    request.keep_record = false;
    std::error_code ignored;
    std::filesystem::remove_all(scratch, ignored);
    const int code =
        app::evaluateTrajectories(request, app::builtinCatalogs(), std::cout, std::cerr);
    std::filesystem::remove_all(scratch, ignored);
    return code;
}
