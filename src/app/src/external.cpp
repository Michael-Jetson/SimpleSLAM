// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// @file external.cpp
/// 单独评测别的系统跑出来的轨迹：把两条轨迹放进一份最小的任务记录，再走同一条评测路径。

#include "evaluation.hpp"
#include "run_dir.hpp"

#include <SimpleSLAM/app/run.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>

#include <yaml-cpp/yaml.h>

#include <format>
#include <iostream>
#include <stdexcept>

#ifndef SIMPLESLAM_VERSION
#define SIMPLESLAM_VERSION "unknown"
#endif

namespace simpleslam::app {

namespace {

constexpr const char* kExternalFiles[] = {"trajectory", "evaluation", "identity.yaml",
                                          "result.json"};

eval::TrajectoryFormat formatOf(const std::string& text, const std::string& path) {
    if (text.empty()) return eval::detectTrajectoryFormat(path);
    const auto parsed = eval::parseTrajectoryFormat(text);
    if (!parsed) throw std::runtime_error("未知的轨迹格式 " + text);
    return *parsed;
}

/// v2 的运行报告（`report.yaml`）：内容摘要与完整性是平铺的键，摘要的算法与现在相同。
Completeness fromV2Report(const YAML::Node& root, const std::string& path,
                          const std::string& estimate_digest) {
    const YAML::Node digest = root["trajectory_digest"];
    if (!digest || digest.as<std::string>() != estimate_digest) {
        throw std::runtime_error("运行报告描述的不是这条估计轨迹（内容摘要不符）: " + path);
    }
    const std::string state = root["run_completeness"].as<std::string>();
    if (state != eval::toString(eval::RunCompleteness::kComplete)
        && state != eval::toString(eval::RunCompleteness::kIncomplete)) {
        throw std::runtime_error("运行报告里没有运行完整性的证据（" + state + "）: " + path);
    }
    return Completeness{.state = state == eval::toString(eval::RunCompleteness::kComplete)
                                     ? eval::RunCompleteness::kComplete
                                     : eval::RunCompleteness::kIncomplete,
                        .source = path,
                        .declaration = {}};
}

/// 产生这条估计的运行的结论：它必须列着这条估计的内容摘要，完整性取自它。
Completeness fromRunResult(const std::string& path, const std::string& estimate_digest,
                           std::optional<fw::TimeRange>& window) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("读不懂运行结论 " + path + ": " + e.what());
    }
    if (!root["schema"] && root["run_completeness"]) {
        return fromV2Report(root, path, estimate_digest);
    }
    if (!root["schema"] || root["schema"].as<int>() != run_dir::kResultSchema) {
        throw std::runtime_error("运行结论 " + path + " 的格式版本不对");
    }
    bool described = false;
    for (const auto& entry : root["trajectories"]) {
        if (entry.second["digest"].as<std::string>() == estimate_digest) described = true;
    }
    if (!described) {
        throw std::runtime_error("运行结论描述的不是这条估计轨迹（内容摘要不符）: " + path);
    }
    const YAML::Node complete = root["complete"];
    if (!complete || complete.IsNull()) {
        throw std::runtime_error("运行结论里没有运行完整性的证据: " + path);
    }
    if (!window) {
        // 目标窗口在那次运行的身份里
        const auto identity = std::filesystem::path(path).parent_path() / "identity.yaml";
        if (std::filesystem::exists(identity)) {
            const YAML::Node effective = YAML::LoadFile(identity.string())["effective"];
            const YAML::Node begin = effective["source.evaluation_window_begin_ns"];
            const YAML::Node end = effective["source.evaluation_window_end_ns"];
            if (begin && end) {
                window = fw::TimeRange{fw::Timestamp{begin.as<std::int64_t>()},
                                       fw::Timestamp{end.as<std::int64_t>()}};
            }
        }
    }
    return Completeness{.state = complete.as<bool>() ? eval::RunCompleteness::kComplete
                                                     : eval::RunCompleteness::kIncomplete,
                        .source = path,
                        .declaration = {}};
}

void writeExternalIdentity(const fs::path& dir, const TrajectoryEvaluation& request,
                           const std::optional<fw::TimeRange>& window) {
    using run_dir::quoted;
    std::string effective;
    if (window) {
        effective += std::format("  \"source.evaluation_window_begin_ns\": \"{}\"\n",
                                 window->begin.count());
        effective += std::format("  \"source.evaluation_window_end_ns\": \"{}\"\n",
                                 window->end.count());
    }
    run_dir::writeText(dir / "identity.yaml",
                       std::format("software_version: {}\neffective_hash: {}\nestimate: {}\n"
                                   "groundtruth: {}\neffective:{}\n{}",
                                   quoted(SIMPLESLAM_VERSION), quoted(request.config_hash),
                                   quoted(request.estimate), quoted(request.groundtruth),
                                   effective.empty() ? " {}" : "", effective));
}

}  // namespace

int evaluateTrajectories(const TrajectoryEvaluation& request, const config::Catalogs& catalogs,
                         std::ostream& out, std::ostream& err) {
    try {
        if (!request.run_result.empty() && request.declaration) {
            throw std::invalid_argument("运行结论与人工声明只能二选一：有证据就不需要声明");
        }
        if (request.declaration && request.declaration->empty()) {
            throw std::invalid_argument("人工声明要写明依据，声明会原样留在结论里");
        }
        // 输入、证据、评测函数都核对过才动任务记录目录：出错时不留下半份任务记录
        const auto estimate =
            eval::readTrajectory(request.estimate, formatOf(request.format, request.estimate));
        const auto truth = eval::readTrajectory(request.groundtruth,
                                                formatOf(request.format, request.groundtruth));

        // 完整性：没有证据就是"未知"
        std::optional<fw::TimeRange> window = request.window;
        Completeness completeness;
        if (!request.run_result.empty()) {
            completeness = fromRunResult(request.run_result,
                                         eval::trajectoryFileDigest(request.estimate), window);
        } else if (request.declaration) {
            completeness = Completeness{.state = eval::RunCompleteness::kDeclaredComplete,
                                        .source = "人工声明",
                                        .declaration = *request.declaration};
        }

        ResolvedTask task;
        task.config.task.name = request.name;
        task.config.evaluation.baselines = request.baselines;
        task.config.evaluation.evaluators = request.evaluators;
        if (task.config.evaluation.evaluators.empty()) {
            task.config.evaluation.evaluators = {
                {"ate", {.type = "ate", .params = {}, .children = {}}},
                {"rpe", {.type = "rpe", .params = {}, .children = {}}}};
        }
        for (auto& [name, spec] : task.config.evaluation.evaluators) {
            auto slot =
                catalogs.resolve<eval::EvaluatorBase>(spec, "evaluation.evaluators." + name);
            spec = slot.resolved();
            task.evaluators.emplace(name, std::move(slot));
        }

        // 两条轨迹进任务记录：评测只读任务记录
        const fs::path dir(request.out);
        run_dir::refuseOccupied(dir, request.overwrite);
        run_dir::prepare(dir, kExternalFiles);
        fs::create_directories(dir / "trajectory");
        eval::writeTrajectory((dir / "trajectory" / "odometry.tum").string(), estimate,
                              eval::TrajectoryFormat::kTum);
        eval::writeTrajectory((dir / "trajectory" / "groundtruth.tum").string(), truth,
                              eval::TrajectoryFormat::kTum);
        writeExternalIdentity(dir, request, window);

        const auto evaluated = evaluateRunDir(dir, task, completeness.state, out);
        // 没有任务：名字取调用方给的数据集名，没给就取估计轨迹的文件名；哈希只有调用方给的
        const std::string name =
            request.name.empty() ? fs::path(request.estimate).stem().string() : request.name;
        const ResultIdentity identity{
            .task = name,
            .mode = {},
            .task_hash = request.config_hash,
            .system_hash = {}};
        writeResult(dir, identity, completeness, &evaluated);
        const auto& verdict = evaluated.verdict;
        switch (verdict.outcome) {
            case eval::BaselineOutcome::kPass:
                if (!verdict.detail.empty()) err << "注意: " << verdict.detail << '\n';
                break;
            case eval::BaselineOutcome::kFail:
                err << "精度低于基线:\n";
                for (const auto& check : verdict.failures()) {
                    err << "  " << check.metric << " = " << check.measured
                        << (check.upper ? " > 上限 " : " < 下限 ") << check.limit << '\n';
                }
                break;
            case eval::BaselineOutcome::kInvalid:
                err << "本次评测没有产生有效结论: " << verdict.detail << '\n';
                for (const auto& missing : verdict.missing_metrics) {
                    err << "  缺少指标 " << missing << '\n';
                }
                for (const auto& bad : verdict.invalid_metrics) {
                    err << "  非有限指标 " << bad << '\n';
                }
                break;
            case eval::BaselineOutcome::kIncomparable:
                err << "不可比较: " << verdict.detail << '\n';
                if (completeness.state == eval::RunCompleteness::kUnknown) {
                    err << "  用 --run-report 绑定产生这条轨迹的运行结论，"
                           "或用 --assume-complete <说明> 人工声明\n";
                }
                break;
        }
        if (request.keep_record) out << "结论: " << (dir / "result.json").string() << '\n';
        return eval::exitCodeFor(verdict, request.allow_incomparable);
    } catch (const std::exception& e) {
        err << "错误: " << e.what() << '\n';
        return 1;
    }
}

}  // namespace simpleslam::app
