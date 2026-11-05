// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file reader.hpp
/// 回放：把录制文件重新变成来源流。
///
/// `ReplaySource` 就是一个普通的 ISensorSource，因此整套系统不知道自己在回放——
/// 这是"回放必须走与在线同一条代码路径"的落实方式，不是巧合。
///
/// 截断处理：录制进程被杀是常态。读到不完整的尾部就停在那里并把 truncated 置位，
/// 前面完好的记录照常可用。把半条记录解码成"零点云"比直接报告截断危险得多。

#include <SimpleSLAM/framework/sensor_source.hpp>
#include <SimpleSLAM/record/format.hpp>
#include <SimpleSLAM/record/writer.hpp>

#include <cstdint>
#include <fstream>
#include <optional>
#include <string>

namespace simpleslam::record {

class RecordReader {
public:
    RecordReader() = default;
    explicit RecordReader(const std::string& path) { open(path); }

    void open(const std::string& path);
    [[nodiscard]] bool isOpen() const { return file_.is_open(); }

    /// 读下一条；文件结束或遇到截断时返回空。
    [[nodiscard]] std::optional<fw::SensorDatum> next();

    [[nodiscard]] const RecordMetadata& metadata() const noexcept { return metadata_; }
    [[nodiscard]] const std::string& rawMetadata() const noexcept { return raw_metadata_; }
    [[nodiscard]] std::uint64_t recordCount() const noexcept { return records_; }
    /// 文件尾部不完整。数据仍然可用到这一点为止，但不完整这件事必须让调用方知道。
    [[nodiscard]] bool truncated() const noexcept { return truncated_; }
    /// 读到了正常的结束标记，且条数与声明相符。
    ///
    /// 与 `!truncated()` 不是一回事：没有结束标记时，"恰好删掉最后一条完整记录"
    /// 看起来就像正常读完。只有这个为真，才能说这份录制是完整的。
    [[nodiscard]] bool complete() const noexcept { return complete_; }
    /// 文件声明的格式版本。
    [[nodiscard]] std::uint16_t formatVersion() const noexcept { return version_; }

private:
    std::ifstream file_;
    std::string path_;
    RecordMetadata metadata_;
    std::string raw_metadata_;
    std::vector<std::uint8_t> payload_;
    std::uint64_t records_ = 0;
    bool truncated_ = false;
    bool complete_ = false;
    /// 文件自己声明的格式版本。解码规则按它走，不按本程序的最新版本走。
    std::uint16_t version_ = kFormatVersion;
};

/// 回放来源。和在线来源在类型上无差别，因此系统装配代码完全一样。
class ReplaySource final : public fw::ISensorSource {
public:
    explicit ReplaySource(const std::string& path) : reader_(path), path_(path) {}

    std::optional<fw::SensorDatum> next() override {
        auto datum = reader_.next();
        if (!datum) finished_ = true;
        return datum;
    }
    [[nodiscard]] std::string name() const override { return "ReplaySource(" + path_ + ")"; }
    /// 回放不改变数据本身的性质，但录制文件里没有存能力描述，所以只填名字。
    /// 需要完整描述的场合，把描述随录制一起存进实验回放包。
    [[nodiscard]] fw::SourceDescriptor descriptor() const override {
        fw::SourceDescriptor descriptor;
        descriptor.name = name();
        return descriptor;
    }

    /// 终态。读完一份带结束标记的文件才是 kCleanEof；只拿到完好前缀是 kIncomplete。
    ///
    /// 上层据此决定还要不要拿这次运行下结论：一个空 optional 说不出"读完了"
    /// 和"读到一半没了"的区别，而严格评测两者必须分开。
    [[nodiscard]] fw::SourceOutcome outcome() const override {
        if (!finished_) return fw::SourceOutcome::kRunning;
        if (reader_.complete()) return fw::SourceOutcome::kCleanEof;
        return fw::SourceOutcome::kIncomplete;
    }

    [[nodiscard]] const RecordMetadata& metadata() const noexcept { return reader_.metadata(); }
    [[nodiscard]] bool truncated() const noexcept { return reader_.truncated(); }
    [[nodiscard]] bool complete() const noexcept { return reader_.complete(); }
    [[nodiscard]] std::uint64_t recordCount() const noexcept { return reader_.recordCount(); }

private:
    RecordReader reader_;
    std::string path_;
    bool finished_ = false;
};

}  // namespace simpleslam::record
