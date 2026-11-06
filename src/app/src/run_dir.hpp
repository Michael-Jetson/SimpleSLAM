// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file run_dir.hpp
/// 任务记录目录的公用件：单次运行、单独评测与多序列都用。入口层内部的头文件，不是公共接口。
///
/// `outcome.yaml` 与 `result.json` 的写与读放在这里，格式各只有一份。

#include <SimpleSLAM/config/config.hpp>
#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/modules/slam_system.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace simpleslam::app::run_dir {

namespace fs = std::filesystem;

/// 任务记录目录：`outputs.dir`，没写就是 `runs/<任务名>`。
[[nodiscard]] fs::path directory(const config::TaskConfig& config);

/// 已有内容时拒绝写入，免得两次任务的结果混在一起。只查，不动目录。
void refuseOccupied(const fs::path& dir, bool overwrite);

/// 建任务记录目录。执行器自己写的文件先删掉（覆盖时才有），目录里别的文件不动。
void prepare(const fs::path& dir, std::span<const char* const> own_files);

/// 写一个任务记录里的文件，写不进去就抛。
void writeText(const fs::path& path, const std::string& text);

/// 原始配置原样存进 `<dir>/source/`：任务文件、它引用的系统文件、`extra`（如 `.py` 任务），
/// 施加过的覆盖项写成 `overrides.txt`。`task.yaml` 是展开后的完整任务，给程序原样重跑；
/// 这里是人写的原文，给人看当初怎么写的。重名时前面加上它的角色（`system_`、`source_`）。
void writeSources(const fs::path& dir, const config::TaskSource& source, const std::string& extra);

/// YAML 双引号字符串。任务记录里的字符串一律加引号，读回来不会变成别的类型。
[[nodiscard]] std::string quoted(const std::string& text);
[[nodiscard]] std::string quotedList(const std::vector<std::string>& items);

/// `result.json` 的格式版本。字段改了含义或删了字段就加一，读的一方先核对它。
/// 读旧写新：写第 3 版，读的一方也认第 2 版（多出的哈希、完整性来源、门禁等字段不读）。
inline constexpr int kResultSchema = 3;
inline constexpr int kOldestResultSchema = 2;

/// JSON 字符串与数。`result.json` 是给程序读的结论，操作台（Python）读它，不解析 YAML。
[[nodiscard]] std::string jsonString(const std::string& text);
/// 往返无损的最短写法；非有限值写 null（JSON 没有 NaN）。
[[nodiscard]] std::string jsonNumber(double value);

/// 一次运行的规模，进 `outcome.yaml`。
struct RunCounts {
    std::size_t packets = 0;
    std::uint64_t frames = 0;
    std::uint64_t keyframes = 0;
    /// 日志缓冲满了丢掉的 debug、info 条数。
    std::uint64_t log_dropped = 0;
};

/// 写 `<dir>/outcome.yaml`：这次运行的事实。写它的软件版本、运行结局的三项、规模、原因与
/// 说明，以及实际生效、配置里看不出来的输入（`effective`：解析出的打包策略、来源的描述、
/// 算法声明的策略、目标窗口）。
void writeOutcome(const fs::path& dir, const modules::RunOutcome& outcome, const RunCounts& counts,
                  const std::map<std::string, std::string>& effective);

/// 读回 `<dir>/outcome.yaml` 里"运行是否完整"。读不懂就抛。
[[nodiscard]] bool readComplete(const fs::path& dir);

/// 读回运行时来源声明的目标窗口；没声明时为空。v3 写的任务记录把它记在 `identity.yaml` 里。
[[nodiscard]] std::optional<fw::TimeRange> readTargetWindow(const fs::path& dir);

/// 写这份任务记录的软件的大版本号（`outcome.yaml` 的 `software_version`；v3 及以前的在
/// `identity.yaml` 里）；读不出来时为空。
[[nodiscard]] std::optional<int> writtenByMajor(const fs::path& dir);

/// 一个标量在结论里的样子：值、单位、进汇总表时的短名。
struct ScalarValue {
    double value = 0.0;
    std::string unit;
    std::string label;
};

/// 一份任务记录的结论摘要：多序列汇总时读每个子任务记录的 `result.json`。
struct ResultSummary {
    int exit_code = 0;
    std::optional<bool> complete;
    /// 全部标量，键是全名 `<评测函数>.<量>.<统计>`，按评测函数与它给出的先后排列。
    std::vector<std::pair<std::string, ScalarValue>> scalars;
};

/// 读 `<dir>/result.json`。没有、读不懂、格式版本不对都抛 `std::runtime_error`。
[[nodiscard]] ResultSummary readResult(const fs::path& dir);

}  // namespace simpleslam::app::run_dir
