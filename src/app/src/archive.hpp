// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file archive.hpp
/// 任务档案目录的公用件：单次运行与多序列都用。入口层内部的头文件，不是公共接口。
///
/// `outcome.yaml` 的写与读放在这里：单次运行写它，多序列的汇总读它，格式只有一份。

#include <SimpleSLAM/config/config.hpp>
#include <SimpleSLAM/modules/slam_system.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
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

/// 一次运行的规模，进 `outcome.yaml`。
struct RunCounts {
    std::size_t packets = 0;
    std::uint64_t frames = 0;
    std::uint64_t keyframes = 0;
};

/// 写 `<dir>/outcome.yaml`：运行结局的三项、规模、原因与说明。
void writeOutcome(const fs::path& dir, const modules::RunOutcome& outcome, const RunCounts& counts);

/// 读回 `<dir>/outcome.yaml` 里"运行是否完整"。读不懂就抛。
[[nodiscard]] bool readComplete(const fs::path& dir);

}  // namespace simpleslam::app::archive
