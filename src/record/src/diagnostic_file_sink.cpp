// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/record/diagnostic_file_sink.hpp>

#include <cmath>
#include <cstdio>
#include <string>
#include <variant>

namespace simpleslam::record {

namespace {

std::string escape(const std::string& text) {
    std::string out;
    out.reserve(text.size() + 2);
    for (const char c : text) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buffer[8];
                    std::snprintf(buffer, sizeof(buffer), "\\u%04x",
                                  static_cast<unsigned>(static_cast<unsigned char>(c)));
                    out += buffer;
                } else {
                    out += c;
                }
                break;
        }
    }
    return out;
}

/// 数值写成 JSON：全精度（%.17g 能原样读回），非有限值写 null。
/// 流的默认精度只有 6 位有效数字，1.00000049 会写成 1。
std::string jsonNumber(double value) {
    if (!std::isfinite(value)) return "null";
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
    return buffer;
}

const char* nonFiniteText(double value) {
    if (std::isnan(value)) return "nan";
    return value > 0 ? "inf" : "-inf";
}

const char* levelName(fw::DiagnosticClass level) {
    switch (level) {
        case fw::DiagnosticClass::kControl: return "control";
        case fw::DiagnosticClass::kMetric: return "metric";
        case fw::DiagnosticClass::kObservation: return "observation";
    }
    return "observation";
}

}  // namespace

void FileDiagnosticSink::open(const std::string& path) {
    std::lock_guard lock(mutex_);
    if (file_.is_open()) file_.close();
    file_.open(path, std::ios::trunc);
    path_ = path;
    written_ = 0;
    failed_ = 0;
}

void FileDiagnosticSink::close() {
    std::lock_guard lock(mutex_);
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

bool FileDiagnosticSink::isOpen() const {
    std::lock_guard lock(mutex_);
    return file_.is_open();
}

std::uint64_t FileDiagnosticSink::written() const {
    std::lock_guard lock(mutex_);
    return written_;
}

std::uint64_t FileDiagnosticSink::failed() const {
    std::lock_guard lock(mutex_);
    return failed_;
}

void FileDiagnosticSink::submit(const fw::DiagnosticRecord& record) noexcept {
    try {
        std::lock_guard lock(mutex_);
        if (!file_.is_open()) {
            ++failed_;
            return;
        }
        // 两个时刻都写：数据时刻用来与轨迹对齐，处理时刻用来看延迟
        file_ << R"({"level":")" << levelName(record.level) << R"(","module":")"
              << escape(record.header.module) << R"(","source":")"
              << escape(record.header.source) << R"(","data_time":)"
              << record.header.data_time.count() << R"(,"process_time":)"
              << record.header.process_time.count() << R"(,"seq":)"
              << record.header.frame_sequence << R"(,"session":)" << record.header.session.value
              << R"(,"param_revision":)" << record.header.parameter_revision;
        if (const auto* metric = std::get_if<fw::MetricRecord>(&record.payload)) {
            file_ << R"(,"kind":"metric","key":")" << escape(metric->key) << R"(","value":)"
                  << jsonNumber(metric->value);
            // JSON 没有 NaN / inf：值写 null，原样的文字另起一个字段，不写出坏行
            if (!std::isfinite(metric->value)) {
                file_ << R"(,"value_text":")" << nonFiniteText(metric->value) << '"';
            }
            file_ << R"(,"unit":")" << escape(metric->unit) << '"';
        } else if (const auto* event = std::get_if<fw::EventRecord>(&record.payload)) {
            file_ << R"(,"kind":"event","key":")" << escape(event->key) << R"(","detail":")"
                  << escape(event->detail) << '"';
        }
        file_ << "}\n";
        if (record.level == fw::DiagnosticClass::kControl) file_.flush();
        if (!file_) {
            ++failed_;
            return;
        }
        ++written_;
    } catch (...) {  // NOLINT(bugprone-empty-catch)
        // 诊断路径不得影响估计流程：写失败就丢，计数留给 failed()
    }
}

}  // namespace simpleslam::record
