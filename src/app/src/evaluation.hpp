// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file evaluation.hpp
/// 对一份任务记录评测，写下结论。入口层内部的头文件，不是公共接口。
///
/// 执行器跑完之后、单独评测、多序列的每个子任务记录都走这里，评测只读任务记录，
/// 所以跑完立刻评与事后补评数字一样。结论写成 `result.json`（格式版本 2），
/// 逐样本的表格写进 `evaluation/`。

#include <SimpleSLAM/app/task.hpp>
#include <SimpleSLAM/eval/evaluator.hpp>

#include <filesystem>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>

namespace simpleslam::app {

namespace fs = std::filesystem;

/// 任务记录作为评测函数的输入：轨迹、真值与生效的标定从目录里读；目标窗口由调用方给，
/// 任务记录的取自它的 `outcome.yaml`（`run_dir::readTargetWindow`）。
class RunDirInputs final : public eval::EvaluationInputs {
public:
    RunDirInputs(fs::path dir, std::optional<fw::TimeRange> window);

    [[nodiscard]] eval::ComparablePair pair(std::string_view estimate) const override;
    [[nodiscard]] std::optional<fw::TimeRange> targetWindow() const override;

    /// 自上次取走以来用到的轨迹文件（相对任务记录的路径），取走后清空。
    [[nodiscard]] std::vector<std::string> takeUsed() const;

private:
    fs::path dir_;
    std::optional<fw::TimeRange> window_;
    mutable std::vector<std::string> used_;
};

/// 一个评测函数的结果，连同它的名字、节点与用到的输入。
struct NamedEvaluation {
    std::string name;
    config::PluginSpec spec;
    eval::Evaluation result;
    std::vector<std::string> inputs;
};

/// 一次评测的全部结果。
struct EvaluationOutcome {
    std::vector<NamedEvaluation> evaluations;
};

/// 任务记录对应的退出码：运行完整是 0，不完整是 4（任务记录照常写完）。外部轨迹不知道
/// 产生它的运行是否完整，也是 0。
[[nodiscard]] int exitCodeFor(std::optional<bool> complete);

/// 所选评测函数里有没有要真值的。执行器在运行之前据此核对数据有没有真值。
[[nodiscard]] bool needsGroundTruth(const ResolvedTask& task);

/// 对任务记录评测：依次跑评测函数、核对它们给的名字，都成了才写表格；给人看的每项一行
/// 写到 out。出错时抛异常，任务记录里上一次的表格不动。
[[nodiscard]] EvaluationOutcome evaluateRunDir(const fs::path& dir, const ResolvedTask& task,
                                               std::optional<fw::TimeRange> window,
                                               std::ostream& out);

/// 结论的抬头：任务名与运行方式。任务记录的取自任务；外部轨迹没有任务，名字由调用方给，
/// 不编一个出来。
struct ResultHeader {
    std::string task;
    std::string mode;
};

[[nodiscard]] ResultHeader headerOf(const ResolvedTask& task);

/// 写 `result.json`：抬头、运行是否完整（外部轨迹为 null）、退出码、评测（没评测时为空）。
void writeResult(const fs::path& dir, const ResultHeader& header, std::optional<bool> complete,
                 const EvaluationOutcome* evaluated);

}  // namespace simpleslam::app
