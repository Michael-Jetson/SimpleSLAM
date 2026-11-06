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
#include <SimpleSLAM/eval/baseline.hpp>
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

/// 任务记录作为评测函数的输入：轨迹、真值、生效的标定、目标窗口都从目录里读。
class RunDirInputs final : public eval::EvaluationInputs {
public:
    explicit RunDirInputs(fs::path dir);

    [[nodiscard]] eval::ComparablePair pair(std::string_view estimate) const override;
    [[nodiscard]] std::optional<fw::TimeRange> targetWindow() const override;

    /// 有效身份的哈希，门禁拿它与基线核对。
    [[nodiscard]] const std::string& effectiveHash() const noexcept { return effective_hash_; }
    /// 自上次取走以来用到的轨迹文件（相对任务记录的路径），取走后清空。
    [[nodiscard]] std::vector<std::string> takeUsed() const;

private:
    fs::path dir_;
    std::string effective_hash_;
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

/// 一次评测的全部结论。
struct EvaluationOutcome {
    std::vector<NamedEvaluation> evaluations;
    eval::BaselineVerdict verdict;
    std::string baselines;  ///< 基线文件；空表示没有
    int exit_code = 0;
};

/// 产生这份任务记录的运行是否完整，以及这个判断从哪里来。
struct Completeness {
    eval::RunCompleteness state = eval::RunCompleteness::kUnknown;
    std::string source;       ///< 本次运行、产生轨迹的那次运行的结论文件，或人工声明
    std::string declaration;  ///< 人工声明的原话：声明不是证据，所以原话留在结论里
};

/// 本次运行的完整性：读任务记录里的运行结局。
[[nodiscard]] Completeness completenessOf(const fs::path& dir);

/// 所选评测函数里有没有要真值的。执行器在运行之前据此核对数据有没有真值。
[[nodiscard]] bool needsGroundTruth(const ResolvedTask& task);

/// 对任务记录评测：依次跑评测函数、核对它们给的名字、过门禁，都成了才写表格；
/// 给人看的每项一行写到 out。出错时抛异常，任务记录里上一次的表格不动。
[[nodiscard]] EvaluationOutcome evaluateRunDir(const fs::path& dir, const ResolvedTask& task,
                                               eval::RunCompleteness completeness,
                                               std::ostream& out);

/// 结论里的身份。任务记录的身份取自任务；外部轨迹没有任务，只有调用方给的名字与哈希，
/// 不编一个出来。
struct ResultIdentity {
    std::string task;
    std::string mode;
    std::string task_hash;
    std::string system_hash;
};

[[nodiscard]] ResultIdentity identityOf(const ResolvedTask& task);

/// 写 `result.json`：身份、运行的完整性、轨迹的内容摘要、评测与门禁（没评测时为空）。
void writeResult(const fs::path& dir, const ResultIdentity& identity,
                 const Completeness& completeness, const EvaluationOutcome* evaluated);

}  // namespace simpleslam::app
