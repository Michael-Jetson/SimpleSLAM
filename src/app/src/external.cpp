// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// @file external.cpp
/// 单独评测别的系统跑出来的轨迹：把两条轨迹放进一份最小的任务记录，再走同一条评测路径。

#include "evaluation.hpp"
#include "run_dir.hpp"

#include <SimpleSLAM/app/run.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>

#include <stdexcept>

namespace simpleslam::app {

namespace {

/// 外部轨迹的任务记录里由这里写的文件。
constexpr const char* kExternalFiles[] = {"trajectory", "evaluation", "result.json"};

eval::TrajectoryFormat formatOf(const std::string& text, const std::string& path) {
    if (text.empty()) return eval::detectTrajectoryFormat(path);
    const auto parsed = eval::parseTrajectoryFormat(text);
    if (!parsed) throw std::runtime_error("未知的轨迹格式 " + text);
    return *parsed;
}

}  // namespace

int evaluateTrajectories(const TrajectoryEvaluation& request, const config::Catalogs& catalogs,
                         std::ostream& out, std::ostream& err) {
    try {
        // 输入与评测函数都核对过才动任务记录目录：出错时不留下半份任务记录
        eval::TrajectoryIoOptions io;
        io.kitti_times_path = request.times;
        const auto estimate = eval::readTrajectory(
            request.estimate, formatOf(request.format, request.estimate), io);
        const auto truth = eval::readTrajectory(
            request.groundtruth, formatOf(request.format, request.groundtruth), io);

        config::TaskConfig wanted;
        wanted.task.name = request.name;
        wanted.evaluation.evaluators = request.evaluators;
        if (wanted.evaluation.evaluators.empty()) {
            wanted.evaluation.evaluators = {
                {"ate", {.type = "ate", .params = {}, .children = {}}},
                {"rpe", {.type = "rpe", .params = {}, .children = {}}}};
        }
        const ResolvedTask task = resolveEvaluators(std::move(wanted), catalogs);

        // 两条轨迹进任务记录：评测只读任务记录
        const fs::path dir(request.out);
        run_dir::refuseOccupied(dir, request.overwrite);
        run_dir::prepare(dir, kExternalFiles);
        fs::create_directories(dir / "trajectory");
        eval::writeTrajectory((dir / "trajectory" / "odometry.tum").string(), estimate,
                              eval::TrajectoryFormat::kTum);
        eval::writeTrajectory((dir / "trajectory" / "groundtruth.tum").string(), truth,
                              eval::TrajectoryFormat::kTum);

        const auto evaluated = evaluateRunDir(dir, task, request.window, out);
        // 没有任务：名字取调用方给的，没给就取估计轨迹的文件名；运行方式留空，完整性不知道
        const std::string name =
            request.name.empty() ? fs::path(request.estimate).stem().string() : request.name;
        writeResult(dir, ResultHeader{.task = name, .mode = {}}, std::nullopt, &evaluated);
        if (request.keep_record) out << "结论: " << (dir / "result.json").string() << '\n';
        return 0;
    } catch (const std::exception& e) {
        err << "错误: " << e.what() << '\n';
        return 1;
    }
}

}  // namespace simpleslam::app
