// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// @file eval_tool.cpp
/// simpleslam-eval：轨迹精度的命令行入口。
///
/// 输出两种：给人看的表格，和给 CI 看的 YAML 报告 + 退出码。
///
/// 退出码分四档，因为"没通过"有四种不同的原因，混成一个 bool 会让 CI 误判：
///   0 通过 / 3 低于基线 / 4 无效（无关联、非有限值、缺指标）/ 5 不可比较
/// 无法评价不等于通过。结论与退出码都取自 eval::gate()，与示例入口是同一套判据；
/// 报告里写的结论与退出码因此永远一致。
///
/// **独立评测不知道运行是否完整**。一条轨迹文件本身说不出产生它的运行有没有跑完，
/// 所以完整性默认"未知"：没有基线时照常给出误差测量并写明未知，有基线时不可比较。
/// 要拿它与基线比，用 --run-report 读产生它的运行报告（报告绑定了轨迹内容摘要，
/// 不是这条轨迹就拒绝），或用 --assume-complete 人工声明——声明的原话留在报告里。

#include <SimpleSLAM/eval/baseline.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace eval = simpleslam::eval;
namespace fw = simpleslam::fw;

namespace {

int usage() {
    std::cerr << R"(用法: simpleslam-eval <估计轨迹> <真值轨迹> [选项]

选项:
  --format <tum|kitti>        两条轨迹的格式（默认按内容判定）
  --align <none|first_pose|se3|sim3>   对齐方式（默认 se3）
  --tolerance <秒>            最近邻关联容差（默认 0.02）
  --interpolate <秒>          改为把真值插值到估计时刻，括点间隔不超过给定秒数
  --rpe-delta <秒>            RPE 的时间间隔（默认 1.0）
  --rpe-distance <米>         改用距离间隔，并按每米归一
  --window <起秒> <止秒>      评测的目标窗口：任务完成度按它算（默认取运行报告里的）
  --run-report <文件>         产生这条估计轨迹的运行报告：从中取运行完整性与目标窗口。
                              报告绑定了轨迹内容摘要，与这条估计不符就拒绝
  --assume-complete <说明>    没有运行报告时人工声明运行完整；说明原样写进报告
  --baselines <文件>          精度基线；0 通过 / 3 低于基线 / 4 无效 / 5 不可比较
  --dataset <名字>            基线里对应的数据集名（有基线时必填）
  --config-hash <哈希>        写进报告，并与基线的 config_hash 核对
  --report <文件>             把报告写成 YAML
  --allow-incomparable        不可比较时仍返回 0（默认返回 5）
)";
    return 2;
}

void printStats(const std::string& title, const eval::Statistics& stats, const char* unit) {
    std::cout << std::left << std::setw(22) << title << std::right << std::fixed
              << std::setprecision(6) << " rmse=" << std::setw(12) << stats.rmse
              << " mean=" << std::setw(12) << stats.mean << " median=" << std::setw(12)
              << stats.median << " max=" << std::setw(12) << stats.max << "  [" << unit << "] n="
              << stats.count << '\n';
}

/// 一族指标一对都没测出来时不打印数字：零误差与"无结果"不能长得一样。
void printFamily(const std::string& title, std::size_t pairs, const eval::Statistics& translation,
                 const eval::Statistics& rotation, const char* translation_unit,
                 const char* rotation_unit) {
    if (pairs == 0) {
        std::cout << std::left << std::setw(22) << title << " 无结果（没有可比的样本）\n";
        return;
    }
    printStats(title + " 平移", translation, translation_unit);
    printStats(title + " 旋转", rotation, rotation_unit);
}

/// 解析一个数值选项：整串都得是数，且有限。"nan"、"1e999"、"0.1s" 一律拒绝——
/// 它们会原样进入协议，让报告与基线在"同一协议"的判断上失去意义。
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

fw::Timestamp secondsToTimestamp(double seconds) {
    return fw::Timestamp{static_cast<std::int64_t>(std::llround(seconds * 1e9))};
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) return usage();
    const std::string estimate_path = argv[1];
    const std::string reference_path = argv[2];

    std::string format_text;
    std::string alignment_text = "se3";
    double tolerance_s = 0.02;
    double interpolate_gap_s = 0.0;
    double rpe_delta_s = 1.0;
    double rpe_distance_m = 0.0;
    std::optional<std::pair<double, double>> window_s;
    std::string run_report_path;
    std::optional<std::string> declaration;
    std::string baselines_path;
    std::string dataset;
    std::string config_hash;
    std::string report_path;
    bool allow_incomparable = false;

    for (int i = 3; i < argc; ++i) {
        const std::string flag = argv[i];
        const auto value = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("选项 " + flag + " 缺少取值");
            return argv[++i];
        };
        try {
            if (flag == "--format") format_text = value();
            else if (flag == "--align") alignment_text = value();
            else if (flag == "--tolerance") tolerance_s = finiteNumber(flag, value());
            else if (flag == "--interpolate") interpolate_gap_s = finiteNumber(flag, value());
            else if (flag == "--rpe-delta") rpe_delta_s = finiteNumber(flag, value());
            else if (flag == "--rpe-distance") rpe_distance_m = finiteNumber(flag, value());
            else if (flag == "--window") {
                const double begin = finiteNumber(flag, value());
                const double end = finiteNumber(flag, value());
                window_s = std::make_pair(begin, end);
            }
            else if (flag == "--run-report") run_report_path = value();
            else if (flag == "--assume-complete") declaration = value();
            else if (flag == "--baselines") baselines_path = value();
            else if (flag == "--dataset") dataset = value();
            else if (flag == "--config-hash") config_hash = value();
            else if (flag == "--report") report_path = value();
            else if (flag == "--allow-incomparable") allow_incomparable = true;
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
        requireRange(!window_s || window_s->first <= window_s->second, "--window 的起点晚于终点");
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << '\n';
        return 2;
    }
    if (!run_report_path.empty() && declaration) {
        std::cerr << "错误: --run-report 与 --assume-complete 只能二选一：有证据就不需要声明\n";
        return 2;
    }
    if (declaration && declaration->empty()) {
        std::cerr << "错误: --assume-complete 要写明依据，声明会原样留在报告里\n";
        return 2;
    }

    try {
        const auto format = format_text.empty()
                                ? eval::detectTrajectoryFormat(estimate_path)
                                : [&] {
                                      const auto parsed = eval::parseTrajectoryFormat(format_text);
                                      if (!parsed) throw std::runtime_error("未知格式 " + format_text);
                                      return *parsed;
                                  }();
        const auto estimate = eval::readTrajectory(estimate_path, format);
        const auto reference = eval::readTrajectory(
            reference_path,
            format_text.empty() ? eval::detectTrajectoryFormat(reference_path) : format);
        const std::string estimate_digest = eval::trajectoryFileDigest(estimate_path);

        // 运行完整性：没有证据就是"未知"
        eval::RunCompleteness completeness = eval::RunCompleteness::kUnknown;
        std::string completeness_source;
        std::vector<std::string> run_problems;
        if (!run_report_path.empty()) {
            const auto manifest = eval::readRunManifest(run_report_path);
            if (manifest.trajectory_digest.empty()) {
                throw std::runtime_error("运行报告没有绑定轨迹内容，不能当运行凭据: "
                                         + run_report_path);
            }
            if (manifest.trajectory_digest != estimate_digest) {
                throw std::runtime_error("运行报告描述的不是这条估计轨迹（内容摘要不符）: "
                                         + run_report_path);
            }
            completeness = manifest.completeness;
            completeness_source = run_report_path;
            run_problems = manifest.run_problems;
            if (!window_s && (manifest.protocol.window_begin_s != 0.0
                              || manifest.protocol.window_end_s != 0.0)) {
                window_s = std::make_pair(manifest.protocol.window_begin_s,
                                          manifest.protocol.window_end_s);
            }
        } else if (declaration) {
            completeness = eval::RunCompleteness::kDeclaredComplete;
            completeness_source = "人工声明";
        }

        const auto parsed_alignment = eval::parseAlignmentKind(alignment_text);
        if (!parsed_alignment) throw std::runtime_error("未知对齐方式 " + alignment_text);
        const eval::AlignmentKind alignment = *parsed_alignment;

        const auto tolerance = std::chrono::nanoseconds(
            static_cast<std::int64_t>(tolerance_s * 1e9));

        const bool interpolate = interpolate_gap_s > 0.0;
        const auto max_gap =
            std::chrono::nanoseconds(static_cast<std::int64_t>(interpolate_gap_s * 1e9));

        eval::AteOptions ate_options;
        ate_options.tolerance = tolerance;
        ate_options.alignment = alignment;
        ate_options.association =
            interpolate ? eval::AssociationMode::kInterpolate : eval::AssociationMode::kNearest;
        ate_options.max_interpolation_gap = max_gap;
        if (window_s) {
            ate_options.target_window =
                fw::TimeRange{secondsToTimestamp(window_s->first), secondsToTimestamp(window_s->second)};
        }
        const auto ate = eval::absoluteTrajectoryError(estimate, reference, ate_options);

        eval::RpeOptions rpe_options;
        rpe_options.tolerance = tolerance;
        rpe_options.association = ate_options.association;
        rpe_options.max_interpolation_gap = max_gap;
        if (rpe_distance_m > 0.0) {
            rpe_options.delta_kind = eval::RpeDelta::kDistance;
            rpe_options.delta_distance_m = rpe_distance_m;
            rpe_options.normalize = true;
        } else {
            rpe_options.delta_time =
                std::chrono::nanoseconds(static_cast<std::int64_t>(rpe_delta_s * 1e9));
        }
        const auto rpe = eval::relativePoseError(estimate, reference, rpe_options);

        std::cout << "估计: " << estimate_path << " (" << estimate.size() << " 帧)\n"
                  << "真值: " << reference_path << " (" << reference.size() << " 帧)\n"
                  << "对齐: " << eval::toString(ate.alignment.kind)
                  << "  尺度: " << ate.alignment.scale << "  关联: " << ate.pairs << " 对 ("
                  << std::fixed << std::setprecision(1) << ate.coverage() * 100.0 << "%)\n";
        if (ate.target_declared) {
            std::cout << "目标窗口完成度: " << ate.targetCoverage() * 100.0 << "%";
        } else {
            std::cout << "目标窗口: 未声明（任务完成度无从计算）";
        }
        std::cout << "  参考样本利用: " << ate.referenceCoverage() * 100.0
                  << "%  参考跨度利用: " << ate.spanRatio() * 100.0 << "%\n";
        std::cout << "运行完整性: " << eval::toString(completeness)
                  << (completeness_source.empty() ? "" : "（" + completeness_source + "）") << '\n';
        printFamily("ATE", ate.pairs, ate.translation, ate.rotation_deg, "m", "deg");
        printFamily(rpe.normalized ? "RPE(每米)" : "RPE", rpe.pairs, rpe.translation,
                    rpe.rotation_deg, rpe.normalized ? "m/m" : "m",
                    rpe.normalized ? "deg/m" : "deg");

        eval::EvaluationReport report;
        report.dataset = dataset.empty() ? reference_path : dataset;
        report.config_hash = config_hash;
        report.alignment = eval::toString(ate.alignment.kind);
        report.metrics = eval::toMetricMap(ate, rpe);
        report.completeness = completeness;
        report.completeness_source = completeness_source;
        if (declaration) report.completeness_declaration = *declaration;
        report.run_problems = run_problems;
        report.input_complete = completeness == eval::RunCompleteness::kComplete
                                || completeness == eval::RunCompleteness::kDeclaredComplete;
        // 这份报告描述的是刚评测的这条估计：绑定它的内容
        report.trajectory_file = estimate_path;
        report.trajectory_digest = estimate_digest;
        // 协议取自实际生效的选项：实际采用的对齐方式（样本不足时可能退化）与请求的分开记
        report.protocol.alignment = eval::toString(ate.alignment.kind);
        report.protocol.alignment_requested = eval::toString(alignment);
        report.protocol.association = eval::toString(ate_options.association);
        report.protocol.association_tolerance_s = tolerance_s;
        report.protocol.max_interpolation_gap_s = interpolate_gap_s;
        report.protocol.rpe_delta_kind =
            rpe.delta_kind == eval::RpeDelta::kDistance ? "distance" : "time";
        report.protocol.rpe_delta = rpe.delta_kind == eval::RpeDelta::kDistance ? rpe_distance_m
                                                                                 : rpe_delta_s;
        report.protocol.rpe_normalized = rpe.normalized;
        if (window_s) {
            report.protocol.window_begin_s = window_s->first;
            report.protocol.window_end_s = window_s->second;
        }

        std::vector<eval::DatasetBaseline> baselines;
        if (!baselines_path.empty()) baselines = eval::loadBaselines(baselines_path);

        eval::GateRequest request;
        request.metrics = report.metrics;
        request.dataset = dataset;
        request.config_hash = config_hash;
        request.protocol = report.protocol;
        request.completeness = completeness;
        request.baselines = baselines_path.empty() ? nullptr : &baselines;
        report.verdict = eval::gate(request);

        switch (report.verdict.outcome) {
            case eval::BaselineOutcome::kPass:
                if (!report.verdict.detail.empty()) std::cerr << "注意: " << report.verdict.detail << '\n';
                break;
            case eval::BaselineOutcome::kFail:
                std::cerr << "精度低于基线:\n";
                for (const auto& failure : report.verdict.failures) {
                    std::cerr << "  " << failure.metric << " = " << failure.measured
                              << (failure.upper ? " > 上限 " : " < 下限 ") << failure.bound << '\n';
                }
                break;
            case eval::BaselineOutcome::kInvalid:
                std::cerr << "本次评测没有产生有效结论: " << report.verdict.detail << '\n';
                for (const auto& missing : report.verdict.missing_metrics) {
                    std::cerr << "  缺少指标 " << missing << '\n';
                }
                for (const auto& invalid : report.verdict.invalid_metrics) {
                    std::cerr << "  非有限指标 " << invalid << '\n';
                }
                break;
            case eval::BaselineOutcome::kIncomparable:
                std::cerr << "不可比较: " << report.verdict.detail << '\n';
                if (completeness == eval::RunCompleteness::kUnknown) {
                    std::cerr << "  用 --run-report 绑定产生这条轨迹的运行报告，"
                                 "或用 --assume-complete <说明> 人工声明\n";
                }
                break;
        }
        const int exit_code = eval::exitCodeFor(report.verdict, allow_incomparable);

        if (!report_path.empty()) eval::writeReport(report_path, report);
        return exit_code;
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << '\n';
        return 1;
    }
}
