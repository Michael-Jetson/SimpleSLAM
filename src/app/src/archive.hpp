// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file archive.hpp
/// 任务档案目录的公用件：单次运行与多序列都用。入口层内部的头文件，不是公共接口。
///
/// `outcome.yaml` 的写与读放在这里：单次运行写它，多序列的汇总读它，格式只有一份。

#include <SimpleSLAM/config/config.hpp>
#include <SimpleSLAM/eval/baseline.hpp>
#include <SimpleSLAM/modules/slam_system.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace simpleslam::app::archive {

namespace fs = std::filesystem;

/// 档案目录：`outputs.dir`，没写就是 `runs/<任务名>`。
[[nodiscard]] fs::path directory(const config::TaskConfig& config);

/// 已有内容时拒绝写入，免得两次任务的结果混在一起。只查，不动目录。
void refuseOccupied(const fs::path& dir, bool overwrite);

/// 建档案目录。执行器自己写的文件先删掉（覆盖时才有），目录里别的文件不动。
void prepare(const fs::path& dir, std::span<const char* const> own_files);

/// 写一个档案文件，写不进去就抛。
void writeText(const fs::path& path, const std::string& text);

/// YAML 双引号字符串。档案里的字符串一律加引号，读回来不会变成别的类型。
[[nodiscard]] std::string quoted(const std::string& text);
[[nodiscard]] std::string quotedList(const std::vector<std::string>& items);

/// 汇总表的列：表头与指标名。单序列的 `result.json`、多序列的汇总与终端表格用的都是它。
inline constexpr std::array<std::pair<const char*, const char*>, 4> kSummaryColumns = {{
    {"ATE_t", "ate.translation.rmse"},
    {"ATE_r", "ate.rotation_deg.rmse"},
    {"RPE_t", "rpe.translation.rmse"},
    {"RPE_r", "rpe.rotation_deg.rmse"},
}};

/// 汇总表里的指标。结论无效、或这一族一对都没关联上时没有值：零误差与没有结果不能长得一样。
[[nodiscard]] std::map<std::string, double> summaryMetrics(
    eval::BaselineOutcome outcome, const std::map<std::string, double>& metrics);

/// `result.json` 的格式版本。字段改了含义或删了字段就加一，读的一方先核对它。
inline constexpr int kResultSchema = 1;

/// JSON 字符串。`result.json` 是档案的机器读摘要，操作台（Python）读它，不解析 YAML。
[[nodiscard]] std::string jsonString(const std::string& text);
/// 指标表的 JSON 对象，键按字典序；非有限值写 null（JSON 没有 NaN）。
[[nodiscard]] std::string jsonMetrics(const std::map<std::string, double>& metrics);

/// 一次运行的规模，进 `outcome.yaml`。
struct RunCounts {
    std::size_t packets = 0;
    std::uint64_t frames = 0;
    std::uint64_t keyframes = 0;
    /// 日志缓冲满了丢掉的 debug、info 条数。
    std::uint64_t log_dropped = 0;
};

/// 写 `<dir>/outcome.yaml`：运行结局的三项、规模、原因与说明。
void writeOutcome(const fs::path& dir, const modules::RunOutcome& outcome, const RunCounts& counts);

/// 读回 `<dir>/outcome.yaml` 里"运行是否完整"。读不懂就抛。
[[nodiscard]] bool readComplete(const fs::path& dir);

}  // namespace simpleslam::app::archive
