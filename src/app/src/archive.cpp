// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include "archive.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <format>
#include <fstream>
#include <stdexcept>

namespace simpleslam::app::archive {

fs::path directory(const config::TaskConfig& config) {
    return config.outputs.dir.empty() ? fs::path("runs") / config.task.name
                                      : fs::path(config.outputs.dir);
}

void refuseOccupied(const fs::path& dir, bool overwrite) {
    if (!overwrite && fs::exists(dir) && !fs::is_empty(dir)) {
        throw std::runtime_error(std::format(
            "任务档案目录 {} 已有内容：换一个 outputs.dir，或加 --overwrite", dir.string()));
    }
}

void prepare(const fs::path& dir, std::span<const char* const> own_files) {
    for (const char* name : own_files) fs::remove_all(dir / name);
    fs::create_directories(dir);
}

void writeText(const fs::path& path, const std::string& text) {
    std::ofstream file(path);
    if (!file) throw std::runtime_error("打不开档案文件: " + path.string());
    file << text;
    if (!file) throw std::runtime_error("写档案文件失败: " + path.string());
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

std::map<std::string, double> summaryMetrics(eval::BaselineOutcome outcome,
                                             const std::map<std::string, double>& metrics) {
    std::map<std::string, double> out;
    if (outcome == eval::BaselineOutcome::kInvalid) return out;
    for (const auto& [header, key] : kSummaryColumns) {
        const std::string name(key);
        const auto pairs = metrics.find(name.substr(0, name.find('.')) + ".pairs");
        const auto value = metrics.find(name);
        if (pairs != metrics.end() && pairs->second > 0.0 && value != metrics.end()) {
            out.emplace(name, value->second);
        }
    }
    return out;
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

std::string jsonMetrics(const std::map<std::string, double>& metrics) {
    std::string out = "{";
    for (const auto& [key, value] : metrics) {
        out += (out.size() == 1 ? "" : ", ") + jsonString(key) + ": "
               + (std::isfinite(value) ? std::format("{}", value) : std::string("null"));
    }
    return out + "}";
}

void writeOutcome(const fs::path& dir, const modules::RunOutcome& outcome,
                  const RunCounts& counts) {
    const auto flag = [](bool value) { return value ? "true" : "false"; };
    writeText(dir / "outcome.yaml",
              std::format("complete: {}\ninput_complete: {}\nprocessing_complete: {}\n"
                          "output_complete: {}\npackets: {}\nframes: {}\nkeyframes: {}\n"
                          "reasons: {}\nnotes: {}\n",
                          flag(outcome.complete()), flag(outcome.inputComplete()),
                          flag(outcome.processingComplete()), flag(outcome.outputComplete()),
                          counts.packets, counts.frames, counts.keyframes,
                          quotedList(outcome.reasons()), quotedList(outcome.notes())));
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

}  // namespace simpleslam::app::archive
