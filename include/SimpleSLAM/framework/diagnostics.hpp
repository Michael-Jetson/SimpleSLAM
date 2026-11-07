// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file diagnostics.hpp
/// 诊断：一个采集入口，两档可靠性。
///
/// 同一个事实只采集一次，日志、指标与健康状态机都是它的消费者，
/// 因此事后不必人肉对齐三份输出的时间轴。可靠性按"是否影响算法与复现"划分：
/// 参与降级/健康判断的记录不能与可丢的调试文本共用丢弃政策。

#include <SimpleSLAM/framework/ids.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace simpleslam::fw {

/// 记录的可靠性档位。
enum class DiagnosticClass : std::uint8_t {
    /// 影响算法决策或复现：降级、跳帧、校正生效帧、健康转移、参数生效版本。
    /// 满载时必须显式处理（背压、停止接纳或宣告记录不完整），不得静默丢弃。
    kControl,
    /// 参与健康判断的原始指标。承诺可重算判断时必须保存。
    kMetric,
    /// 纯观察性文本。可丢、可聚合，丢弃本身要可观测。
    kObservation,
};

/// 所有记录共有的关联头。有了它，三类记录事后可以按同一坐标对齐。
struct DiagnosticHeader {
    Timestamp data_time{0};     ///< 数据时刻（传感器时间）
    Timestamp process_time{0};  ///< 处理时刻（运行时间）
    std::string module;         ///< 产生记录的模块名
    /// 产生记录的来源，按来源设等级的那种键：模块自己是 `odometry`，它的插件是
    /// `odometry.plugin`、`odometry.plugin.map`。不经日志对象产生的记录留空。
    std::string source;
    /// 记录所关于的那条输入的序号，按产生记录的模块自己的输入计：里程计是数据包的
    /// 发布序号，后端与位置识别是关键帧库序号。不针对某条输入的记录填最近处理的那一条。
    std::uint64_t frame_sequence = 0;
    /// 记录是否关于某一条输入。为 false 时 `frame_sequence` 与 `data_time` 没有意义
    /// （运行时自己报的错误不针对哪一帧）。
    bool has_frame = false;
    SessionId session{};
    std::uint64_t parameter_revision = 0;
};

/// 指标载荷：类型化，不退化成任意字符串键值。
/// 数值载荷。字段一律持有自己的内容：记录会被内存汇聚点留存、稍后再写盘，指向
/// 调用方字符串的指针到那时可能早已失效。
struct MetricRecord {
    std::string key;
    double value = 0.0;
    std::string unit;
};

/// 事件载荷：实际生效的决定。
struct EventRecord {
    std::string key;
    std::string detail;
};

struct DiagnosticRecord {
    DiagnosticHeader header;
    DiagnosticClass level = DiagnosticClass::kObservation;
    std::variant<MetricRecord, EventRecord> payload;
};

/// 诊断汇聚点。实现必须自己吞掉异常：诊断路径不得影响估计流程。
class IDiagnosticSink {
public:
    virtual ~IDiagnosticSink() = default;
    virtual void submit(const DiagnosticRecord& record) noexcept = 0;
};

/// 内存汇聚：系统用它收事件与度量（`SlamSystem::diagnostics()`），测试也用它。
///
/// 汇聚点是跨模块共享的：多个模块的执行上下文会同时写它。各模块内部串行
/// 并不能保证它们之间串行，所以这里必须自己加锁：不加锁就是多模块并发写同一个
/// vector。诊断路径的锁只在追加一条记录时持有。
class MemoryDiagnosticSink final : public IDiagnosticSink {
public:
    void submit(const DiagnosticRecord& record) noexcept override {
        try {
            std::lock_guard lock(mutex_);
            records_.push_back(record);
        } catch (...) {  // NOLINT(bugprone-empty-catch)
        }
    }

    /// 返回副本：引用在并发写入下会失效，不提供"借出内部容器"的接口。
    [[nodiscard]] std::vector<DiagnosticRecord> records() const {
        std::lock_guard lock(mutex_);
        return records_;
    }
    [[nodiscard]] std::size_t size() const {
        std::lock_guard lock(mutex_);
        return records_.size();
    }
    void clear() {
        std::lock_guard lock(mutex_);
        records_.clear();
    }

    [[nodiscard]] std::size_t count(DiagnosticClass level) const {
        std::lock_guard lock(mutex_);
        std::size_t n = 0;
        for (const auto& r : records_) {
            if (r.level == level) ++n;
        }
        return n;
    }

    /// 键为 `key` 的度量，按记下的先后。只复制匹配的那几条：在线评测每次只要一个键。
    [[nodiscard]] std::vector<DiagnosticRecord> metrics(std::string_view key) const {
        return matching<MetricRecord>(key);
    }

    /// 键为 `key` 的事件，按记下的先后。
    [[nodiscard]] std::vector<DiagnosticRecord> events(std::string_view key) const {
        return matching<EventRecord>(key);
    }

    /// 某个事件键出现了几次。测试与健康监控都按键计数，不按下标取记录。
    [[nodiscard]] std::size_t countEvent(std::string_view key) const {
        std::lock_guard lock(mutex_);
        std::size_t n = 0;
        for (const auto& r : records_) {
            if (const auto* e = std::get_if<EventRecord>(&r.payload); e != nullptr && e->key == key) {
                ++n;
            }
        }
        return n;
    }

private:
    template <class Payload>
    [[nodiscard]] std::vector<DiagnosticRecord> matching(std::string_view key) const {
        std::lock_guard lock(mutex_);
        std::vector<DiagnosticRecord> out;
        for (const auto& r : records_) {
            if (const auto* p = std::get_if<Payload>(&r.payload); p != nullptr && p->key == key) {
                out.push_back(r);
            }
        }
        return out;
    }

    mutable std::mutex mutex_;
    std::vector<DiagnosticRecord> records_;
};

}  // namespace simpleslam::fw
