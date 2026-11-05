// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file format.hpp
/// 录制容器的格式：显式字节序、显式长度、逐条校验和。
///
/// 为什么是自定义格式而不是 MCAP：这个格式不足两百行、不引入新的外部依赖
/// （开发镜像不联网，每多一个依赖就要多一次预先 vendoring），而完整性与截断语义
/// 需要精确控制——哪些字段进校验和、正常结束长什么样、读到一半算什么，
/// 都是这里要自己定的东西。自定义格式**同样会演进**，所以可复现性靠的是版本号、
/// 明确的格式契约和历史样本回归，不是"手写所以不会变"。
///
/// 改换条件写在这里，免得以后靠感觉决定：需要索引或随机访问、需要与外部工具互通、
/// 或格式长到一页读不完时，按实测的体积、性能、构建与维护成本重新比较。
/// 换过去也不必放弃本项目的数据契约——MCAP 允许任意消息编码，容器只承担
/// 通用记录结构与索引。
///
/// 布局（全部小端）：
///
///   文件头： magic[6]="SSREC1" | u16 版本 | u64 标志 | u32 元数据长度 | 元数据（YAML 文本）
///   记录：   u32 种类 | u32 载荷长度 | i64 时刻(ns) | 载荷 | u32 CRC32(记录头 + 载荷)
///   收尾：   一条 kEndOfStream 记录，载荷为 u64 记录条数
///
/// 两条完整性约定：
///
/// 1. **校验和覆盖记录头**。只校验载荷的话，改掉时间戳、种类或长度都不会被发现——
///    而解释载荷正要靠这三个字段。所以 CRC 算在"头 + 载荷"上。
/// 2. **正常结束有显式标记**。没有结束标记时，"恰好删掉最后一条完整记录"与
///    "正常写完"在读取器看来一模一样。收尾记录里写下条数，读取器据此区分
///    `complete()`（确认完整）与只是"读到了一个完好的前缀"。
///
/// 文件仍可在任意记录边界被截断：读取器停在那里并把 truncated() 置位，
/// 前面完好的记录照常可用（录制进程被杀是常态，不是异常）。

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace simpleslam::record {

inline constexpr char kMagic[6] = {'S', 'S', 'R', 'E', 'C', '1'};
/// 格式版本。每一版的含义都写在这里，读取器按文件自己声明的版本解码：
///
/// - 版本 3：元数据里的字符串按 `\\`、`\n`、`\r` 转义；激光记录在设备名之后多一个
///   字节的运动补偿状态。
/// - 版本 2：校验和覆盖记录头，有结束标记；元数据字符串按字面保存（反斜杠不是转义），
///   激光记录没有补偿状态（读出来是 unknown）。仍可读取。
/// - 版本 1：不再接受——它的完整性承诺比实现强，继续读只会把"不完整"当成"完整"。
///
/// 改变任何字节的解释都必须升版本：同一个版本号下换了解码规则，旧文件会被悄悄读错，
/// 而往返测试只覆盖新写的文件，发现不了。
inline constexpr std::uint16_t kFormatVersion = 3;
inline constexpr std::uint16_t kOldestReadableVersion = 2;

/// 记录头的固定长度：u32 种类 + u32 长度 + i64 时刻。
inline constexpr std::size_t kRecordHeaderSize = 16;

enum class RecordKind : std::uint32_t {
    kImuSample = 1,
    kLidarScan = 2,
    kCameraFrame = 3,
    kGnssFix = 4,
    /// 正常收尾。载荷是 u64 记录条数（不含本条）。
    kEndOfStream = 0xFFFFFFFFU,
};

class RecordError : public std::runtime_error {
public:
    explicit RecordError(const std::string& what) : std::runtime_error(what) {}
};

/// CRC32（IEEE 多项式，反射）。逐条算，因此截断与位翻转都能被认出来。
[[nodiscard]] std::uint32_t crc32(const std::uint8_t* data, std::size_t size) noexcept;

/// 显式小端写入。不 memcpy 结构体：结构体布局会随编译器变，录制文件不能随之变。
class ByteWriter {
public:
    void u8(std::uint8_t v) { bytes_.push_back(v); }
    void u16(std::uint16_t v) { little(v, 2); }
    void u32(std::uint32_t v) { little(v, 4); }
    void u64(std::uint64_t v) { little(v, 8); }
    void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
    void i64(std::int64_t v) { u64(static_cast<std::uint64_t>(v)); }
    void f32(float v) {
        std::uint32_t bits = 0;
        static_assert(sizeof(float) == 4);
        __builtin_memcpy(&bits, &v, 4);
        u32(bits);
    }
    void f64(double v) {
        std::uint64_t bits = 0;
        static_assert(sizeof(double) == 8);
        __builtin_memcpy(&bits, &v, 8);
        u64(bits);
    }
    void text(const std::string& v) {
        u32(static_cast<std::uint32_t>(v.size()));
        bytes_.insert(bytes_.end(), v.begin(), v.end());
    }
    void blob(const std::uint8_t* data, std::size_t size) {
        u32(static_cast<std::uint32_t>(size));
        bytes_.insert(bytes_.end(), data, data + size);
    }

    [[nodiscard]] const std::vector<std::uint8_t>& bytes() const noexcept { return bytes_; }
    [[nodiscard]] std::size_t size() const noexcept { return bytes_.size(); }
    void clear() noexcept { bytes_.clear(); }

private:
    void little(std::uint64_t value, int width) {
        for (int i = 0; i < width; ++i) {
            bytes_.push_back(static_cast<std::uint8_t>((value >> (8 * i)) & 0xFFU));
        }
    }
    std::vector<std::uint8_t> bytes_;
};

/// 显式小端读取。越界一律抛，不返回垃圾值。
class ByteReader {
public:
    ByteReader(const std::uint8_t* data, std::size_t size) : data_(data), size_(size) {}

    [[nodiscard]] std::uint8_t u8() { return data_[require(1)]; }
    [[nodiscard]] std::uint16_t u16() { return static_cast<std::uint16_t>(little(2)); }
    [[nodiscard]] std::uint32_t u32() { return static_cast<std::uint32_t>(little(4)); }
    [[nodiscard]] std::uint64_t u64() { return little(8); }
    [[nodiscard]] std::int32_t i32() { return static_cast<std::int32_t>(u32()); }
    [[nodiscard]] std::int64_t i64() { return static_cast<std::int64_t>(u64()); }
    [[nodiscard]] float f32() {
        const std::uint32_t bits = u32();
        float value = 0.0F;
        __builtin_memcpy(&value, &bits, 4);
        return value;
    }
    [[nodiscard]] double f64() {
        const std::uint64_t bits = u64();
        double value = 0.0;
        __builtin_memcpy(&value, &bits, 8);
        return value;
    }
    [[nodiscard]] std::string text() {
        const std::size_t length = u32();
        const std::size_t start = require(length);
        return std::string(reinterpret_cast<const char*>(data_ + start), length);
    }
    [[nodiscard]] std::vector<std::uint8_t> blob() {
        const std::size_t length = u32();
        const std::size_t start = require(length);
        return std::vector<std::uint8_t>(data_ + start, data_ + start + length);
    }

    [[nodiscard]] std::size_t remaining() const noexcept { return size_ - cursor_; }

private:
    std::size_t require(std::size_t count) {
        if (cursor_ + count > size_) throw RecordError("录制记录被截断或损坏");
        const std::size_t start = cursor_;
        cursor_ += count;
        return start;
    }
    std::uint64_t little(int width) {
        const std::size_t start = require(static_cast<std::size_t>(width));
        std::uint64_t value = 0;
        for (int i = 0; i < width; ++i) {
            value |= static_cast<std::uint64_t>(data_[start + static_cast<std::size_t>(i)])
                     << (8 * i);
        }
        return value;
    }

    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t cursor_ = 0;
};

}  // namespace simpleslam::record
