// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file diagnostic_file_sink.hpp
/// 诊断落盘：每行一条 JSON（JSONL）。
///
/// 选 JSONL 而不是二进制：诊断是给人和脚本看的，grep / jq / pandas 都能直接吃，
/// 而且写坏一行不会毁掉整个文件。数据量大的是点云，不是诊断。
///
/// kControl 档立刻 flush：它参与降级与复现判断，进程崩了之后最需要看的就是它，
/// 攒在缓冲里等于没记。kMetric 与 kObservation 按缓冲写。

#include <SimpleSLAM/framework/diagnostics.hpp>

#include <fstream>
#include <mutex>
#include <string>

namespace simpleslam::record {

namespace fw = simpleslam::fw;

class FileDiagnosticSink final : public fw::DiagnosticSink {
public:
    FileDiagnosticSink() = default;
    explicit FileDiagnosticSink(const std::string& path) { open(path); }
    ~FileDiagnosticSink() override { close(); }

    FileDiagnosticSink(const FileDiagnosticSink&) = delete;
    FileDiagnosticSink& operator=(const FileDiagnosticSink&) = delete;

    void open(const std::string& path);
    void close();
    [[nodiscard]] bool isOpen() const;

    void submit(const fw::DiagnosticRecord& record) noexcept override;

    [[nodiscard]] std::uint64_t written() const;
    /// 写失败的条数。诊断路径不得影响估计流程，所以失败只计数不抛。
    [[nodiscard]] std::uint64_t failed() const;

private:
    mutable std::mutex mutex_;
    std::ofstream file_;
    std::string path_;
    std::uint64_t written_ = 0;
    std::uint64_t failed_ = 0;
};

}  // namespace simpleslam::record
