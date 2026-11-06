// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file sequences.hpp
/// 多序列：一个任务列出一个数据集的多个序列（`data.sequences`），每个序列在一个子进程里
/// 跑成一份独立的子任务记录，最后汇总成一张表。入口层内部的头文件，不是公共接口。
///
/// 子进程是同一个可执行文件，带上四个覆盖项：`task.name=<任务名>-<序列>`、
/// `data.sequences=[]`、`data.source.sequence=<序列>`、`outputs.dir=<任务记录目录>/<序列>`。
/// 用户登记的实现因此在子进程里同样有；一个序列崩了，不影响别的序列。

#include <SimpleSLAM/app/run.hpp>
#include <SimpleSLAM/app/task.hpp>

#include <filesystem>
#include <iosfwd>
#include <string>
#include <vector>

namespace simpleslam::app {

/// 跑一个 `data.sequences` 非空的任务。返回各序列里最严重的退出码：
/// 出错 > 无效 > 不可比较 > 失败 > 通过。
int runSequences(const ResolvedTask& task, const RunOptions& options, std::ostream& out);

/// 单独评测多序列的任务记录：每个子任务记录各评一遍（覆盖项照样施加），再汇总。
int evaluateSequences(const ResolvedTask& task, const std::filesystem::path& dir,
                      const std::vector<std::string>& overrides, const config::Catalogs& catalogs,
                      std::ostream& out, std::ostream& err);

}  // namespace simpleslam::app
