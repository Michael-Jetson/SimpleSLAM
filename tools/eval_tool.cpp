// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// @file eval_tool.cpp
/// simpleslam-eval：单独评测一条轨迹（别的系统跑出来的也行）。
///
/// 两条轨迹放进一份最小的任务记录，用内置的评测函数 ate、rpe 评测，结论写成同一种
/// `result.json`，操作台能把它和自己的结果放进一张表。门禁与任务的执行器是同一个
/// `gate()`，退出码分四档：0 通过 / 3 低于基线 / 4 无效 / 5 不可比较。
///
/// **独立评测不知道运行是否完整**。一条轨迹文件本身说不出产生它的运行有没有跑完，
/// 所以完整性默认"未知"：没有基线时照常给出误差测量并写明未知，有基线时不可比较。
/// 要拿它与基线比，用 --run-report 读产生它的那次运行的结论（它列着轨迹的内容摘要，
/// 不是这条轨迹就拒绝），或用 --assume-complete 人工声明——声明的原话留在结论里。

#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/run.hpp>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int usage() {
    std::cerr << R"(用法: simpleslam-eval <估计轨迹> <真值轨迹> --out <目录> [选项]

选项:
  --out <目录>                写出的任务记录：两条轨迹、结论 result.json、逐样本表格
  --overwrite                 目录已有内容时照样写
  --format <tum|kitti>        两条轨迹的格式（默认按内容判定）
  --align <none|first_pose|se3|sim3>   对齐方式（默认 se3）
  --tolerance <秒>            最近邻关联容差（默认 0.02）
  --interpolate <秒>          改为把真值插值到估计时刻，括点间隔不超过给定秒数
  --rpe-delta <秒>            RPE 的时间间隔（默认 1.0）
  --rpe-distance <米>         改用距离间隔，并按每米归一
  --window <起秒> <止秒>      目标窗口：任务完成度按它算（默认取运行结论里的）
  --run-report <result.json>  产生这条估计的那次运行的结论：从中取运行完整性与目标窗口；
                              它没列着这条估计的内容摘要就拒绝
  --assume-complete <说明>    没有运行结论时人工声明运行完整；说明原样写进结论
  --baselines <文件>          精度基线；0 通过 / 3 低于基线 / 4 无效 / 5 不可比较
  --dataset <名字>            基线里对应的数据集名（有基线时必填）
  --config-hash <哈希>        写进结论，并与基线的 config_hash 核对
  --allow-incomparable        不可比较时仍返回 0（默认返回 5）
)";
    return 2;
}

/// 解析一个数值选项：整串都得是数，且有限。"nan"、"1e999"、"0.1s" 一律拒绝——
/// 它们会原样进入协议，让结论与基线在"同一协议"的判断上失去意义。
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
            else if (flag == "--run-report") request.run_result = value();
            else if (flag == "--assume-complete") request.declaration = value();
            else if (flag == "--baselines") request.baselines = value();
            else if (flag == "--dataset") request.name = value();
            else if (flag == "--config-hash") request.config_hash = value();
            else if (flag == "--allow-incomparable") request.allow_incomparable = true;
            else return usage();
        } catch (const std::exception& e) {
            std::cerr << "错误: " << e.what() << '\n';
            return 2;
        }
    }
    try {
        requireRange(!request.out.empty(), "要用 --out 给出写结论的目录");
        requireRange(tolerance_s >= 0.0, "--tolerance 不能为负");
        requireRange(interpolate_gap_s >= 0.0, "--interpolate 不能为负");
        requireRange(rpe_delta_s > 0.0, "--rpe-delta 必须为正");
        requireRange(rpe_distance_m >= 0.0, "--rpe-distance 不能为负");
        if (!request.run_result.empty() && request.declaration) {
            throw std::runtime_error(
                "--run-report 与 --assume-complete 只能二选一：有证据就不需要声明");
        }
        if (request.declaration && request.declaration->empty()) {
            throw std::runtime_error("--assume-complete 要写明依据，声明会原样留在结论里");
        }
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << '\n';
        return 2;
    }

    // 选项换成 ate、rpe 两个评测函数的参数：协议随结论保存
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
    return app::evaluateTrajectories(request, app::builtinCatalogs(), std::cout, std::cerr);
}
