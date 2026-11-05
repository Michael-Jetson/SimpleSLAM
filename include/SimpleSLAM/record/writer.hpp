// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file writer.hpp
/// 录制：把原始传感器流写进容器。
///
/// 录的是**来源流**（SensorDatum），不是打包之后的 SensorPacket。
/// 因为打包本身是被测对象的一部分：改了打包策略之后，回放要重新走一遍打包，
/// 才能回答"这个改动对结果有没有影响"。需要单独调试里程计时，
/// 用同一份录制加固定的打包策略即可得到确定的包序列。

#include <SimpleSLAM/framework/sensor_source.hpp>
#include <SimpleSLAM/record/format.hpp>

#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

namespace simpleslam::record {

namespace fw = simpleslam::fw;

/// 写进文件头的元数据。回放时原样读出，用来核对"这份录制是谁在什么条件下录的"。
struct RecordMetadata {
    std::string name;
    std::string created_at;
    std::string config_hash;
    std::string code_revision;
    std::uint64_t session = 0;
    std::int64_t seed = 0;
    std::string note;
};

class RecordWriter {
public:
    RecordWriter() = default;
    explicit RecordWriter(const std::string& path, const RecordMetadata& metadata = {}) {
        open(path, metadata);
    }
    /// 析构不得抛。close() 要写收尾记录，写失败在这里只能吞掉；
    /// 需要知道收尾成没成功的调用方显式调 close()。
    ~RecordWriter() {
        try {
            close();
        } catch (...) {  // NOLINT(bugprone-empty-catch)
        }
    }

    RecordWriter(const RecordWriter&) = delete;
    RecordWriter& operator=(const RecordWriter&) = delete;
    RecordWriter(RecordWriter&&) = default;
    RecordWriter& operator=(RecordWriter&&) = default;

    void open(const std::string& path, const RecordMetadata& metadata = {});

    /// 收尾：写结束记录、落盘、关闭。
    ///
    /// 全程检查流状态并在失败时抛出。磁盘满、配额超限、文件尺寸上限都会让写入
    /// 静静地置上 badbit——不检查的话，一个只写进去一半的文件会被当成"录完了"，
    /// 而它连结束记录都没有。注意 flush 只保证交给了操作系统，不是断电持久化。
    void close();
    [[nodiscard]] bool isOpen() const { return file_.is_open(); }

    /// 写一条来源数据。返回写入的字节数（含记录头与校验和）。
    std::size_t write(const fw::SensorDatum& datum);

    void write(const fw::ImuSample& sample);
    void write(const fw::LidarScan& scan);
    void write(const fw::CameraFrame& frame);
    void write(const fw::GnssFix& fix);

    [[nodiscard]] std::uint64_t recordCount() const noexcept { return records_; }
    [[nodiscard]] std::uint64_t bytesWritten() const noexcept { return bytes_; }
    [[nodiscard]] const std::string& path() const noexcept { return path_; }

    /// 立刻落盘。录制进程被杀是常态，因此定期 flush 比"最后一次性写"更可靠。
    void flush();

private:
    void emit(RecordKind kind, std::int64_t time_ns, const ByteWriter& payload);

    std::ofstream file_;
    std::string path_;
    ByteWriter scratch_;
    std::uint64_t records_ = 0;
    std::uint64_t bytes_ = 0;
};

/// 录制装饰器：包住任意来源，边取边录。装配时套上即可，来源本身不必知道录制存在。
class RecordingSource final : public fw::ISensorSource {
public:
    RecordingSource(std::unique_ptr<fw::ISensorSource> inner, std::shared_ptr<RecordWriter> writer)
        : inner_(std::move(inner)), writer_(std::move(writer)) {}

    std::optional<fw::SensorDatum> next() override {
        auto datum = inner_->next();
        if (datum && writer_ && writer_->isOpen()) writer_->write(*datum);
        return datum;
    }

    [[nodiscard]] std::string name() const override {
        return "RecordingSource(" + inner_->name() + ")";
    }

    /// 终态与能力描述都由被包住的来源决定：录制只是旁路。
    [[nodiscard]] fw::SourceOutcome outcome() const override { return inner_->outcome(); }
    [[nodiscard]] fw::SourceDescriptor descriptor() const override { return inner_->descriptor(); }

private:
    std::unique_ptr<fw::ISensorSource> inner_;
    std::shared_ptr<RecordWriter> writer_;
};

}  // namespace simpleslam::record
