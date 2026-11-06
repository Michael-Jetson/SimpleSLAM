// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include "run_dir.hpp"

#include <yaml-cpp/yaml.h>

#include <charconv>
#include <cmath>
#include <format>
#include <fstream>
#include <set>
#include <stdexcept>
#include <system_error>

#ifndef SIMPLESLAM_VERSION
#define SIMPLESLAM_VERSION "unknown"
#endif

namespace simpleslam::app::run_dir {

namespace {

std::optional<std::int64_t> parseNanoseconds(const YAML::Node& node) {
    if (!node || !node.IsScalar()) return std::nullopt;
    const std::string text = node.Scalar();
    std::int64_t value = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size()) return std::nullopt;
    return value;
}

}  // namespace

fs::path directory(const config::TaskConfig& config) {
    return config.outputs.dir.empty() ? fs::path("runs") / config.task.name
                                      : fs::path(config.outputs.dir);
}

void refuseOccupied(const fs::path& dir, bool overwrite) {
    if (!overwrite && fs::exists(dir) && !fs::is_empty(dir)) {
        throw std::runtime_error(std::format(
            "任务记录目录 {} 已有内容：换一个 outputs.dir，或加 --overwrite", dir.string()));
    }
}

void prepare(const fs::path& dir, std::span<const char* const> own_files) {
    for (const char* name : own_files) fs::remove_all(dir / name);
    fs::create_directories(dir);
}

void writeText(const fs::path& path, const std::string& text) {
    std::ofstream file(path);
    if (!file) throw std::runtime_error("打不开任务记录里的文件: " + path.string());
    file << text;
    if (!file) throw std::runtime_error("写任务记录里的文件失败: " + path.string());
}

std::string quoted(const std::string& text) {
    std::string out = "\"";
    for (const char c : text) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += c; break;
        }
    }
    return out + '"';
}

std::string quotedList(const std::vector<std::string>& items) {
    std::string out = "[";
    for (std::size_t i = 0; i < items.size(); ++i) out += (i == 0 ? "" : ", ") + quoted(items[i]);
    return out + "]";
}

std::string jsonString(const std::string& text) {
    std::string out = "\"";
    for (const char c : text) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    out += std::format("\\u{:04x}", static_cast<unsigned>(c));
                } else {
                    out += c;
                }
        }
    }
    return out + '"';
}

std::string jsonNumber(double value) {
    return std::isfinite(value) ? std::format("{}", value) : std::string("null");
}

void writeOutcome(const fs::path& dir, const modules::RunOutcome& outcome, const RunCounts& counts,
                  const std::map<std::string, std::string>& effective) {
    const auto flag = [](bool value) { return value ? "true" : "false"; };
    std::string inputs;
    for (const auto& [key, value] : effective) {
        inputs += "  " + quoted(key) + ": " + quoted(value) + "\n";
    }
    writeText(dir / "outcome.yaml",
              std::format("software_version: {}\ncomplete: {}\ninput_complete: {}\n"
                          "processing_complete: {}\noutput_complete: {}\npackets: {}\n"
                          "frames: {}\nkeyframes: {}\nlog_dropped: {}\nreasons: {}\nnotes: {}\n"
                          "effective:{}\n{}",
                          quoted(SIMPLESLAM_VERSION), flag(outcome.complete()),
                          flag(outcome.inputComplete()), flag(outcome.processingComplete()),
                          flag(outcome.outputComplete()), counts.packets, counts.frames,
                          counts.keyframes, counts.log_dropped, quotedList(outcome.reasons()),
                          quotedList(outcome.notes()), inputs.empty() ? " {}" : "", inputs));
}

bool readComplete(const fs::path& dir) {
    const fs::path path = dir / "outcome.yaml";
    try {
        const YAML::Node complete = YAML::LoadFile(path.string())["complete"];
        if (!complete) throw std::runtime_error("运行结局里没有 complete: " + path.string());
        return complete.as<bool>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("读不懂运行结局 " + path.string() + ": " + e.what());
    }
}

void writeSources(const fs::path& dir, const config::TaskSource& source, const std::string& extra) {
    const fs::path target = dir / "source";
    fs::create_directories(target);
    std::set<std::string> taken;
    const auto copy = [&](const std::string& file, const std::string& role) {
        if (file.empty() || !fs::is_regular_file(file)) return;
        std::string name = fs::path(file).filename().string();
        if (!taken.insert(name).second) {
            name = role + "_" + name;
            taken.insert(name);
        }
        fs::copy_file(file, target / name, fs::copy_options::overwrite_existing);
    };
    copy(source.task_file, "task");
    copy(source.system_file, "system");
    copy(extra, "source");
    if (!source.overrides.empty()) {
        std::string text;
        for (const auto& item : source.overrides) text += item + '\n';
        writeText(target / "overrides.txt", text);
    }
}

std::optional<fw::TimeRange> readTargetWindow(const fs::path& dir) {
    try {
        const YAML::Node effective = YAML::LoadFile((dir / "outcome.yaml").string())["effective"];
        if (!effective || !effective.IsMap()) return std::nullopt;
        const auto begin = parseNanoseconds(effective["source.evaluation_window_begin_ns"]);
        const auto end = parseNanoseconds(effective["source.evaluation_window_end_ns"]);
        if (!begin || !end) return std::nullopt;
        return fw::TimeRange{fw::Timestamp{*begin}, fw::Timestamp{*end}};
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("读不懂任务记录 " + dir.string() + " 里的运行事实: " + e.what());
    }
}

ResultSummary readResult(const fs::path& dir) {
    const fs::path path = dir / "result.json";
    if (!fs::exists(path)) throw std::runtime_error("没有结论: " + path.string());
    YAML::Node root;
    try {
        root = YAML::LoadFile(path.string());  // JSON 是 YAML 的子集
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("读不懂结论 " + path.string() + ": " + e.what());
    }
    const int schema = root["schema"] ? root["schema"].as<int>() : 0;
    if (schema != kResultSchema) {
        throw std::runtime_error(std::format("结论 {} 的格式版本是 {}，这里读的是第 {} 版",
                                             path.string(), schema, kResultSchema));
    }
    ResultSummary summary;
    summary.exit_code = root["exit_code"].as<int>();
    if (root["complete"] && !root["complete"].IsNull()) {
        summary.complete = root["complete"].as<bool>();
    }
    for (const auto& evaluation : root["evaluations"]) {
        const auto name = evaluation.first.as<std::string>();
        for (const auto& scalar : evaluation.second["scalars"]) {
            const YAML::Node& value = scalar.second["value"];
            if (!value || value.IsNull()) continue;  // 没有结果的量不当零
            summary.scalars.emplace_back(
                name + "." + scalar.first.as<std::string>(),
                ScalarValue{.value = value.as<double>(),
                            .unit = scalar.second["unit"].as<std::string>(),
                            .label = scalar.second["label"].as<std::string>()});
        }
    }
    return summary;
}

}  // namespace simpleslam::app::run_dir
