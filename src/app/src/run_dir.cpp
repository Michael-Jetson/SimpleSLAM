// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include "run_dir.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <format>
#include <fstream>
#include <stdexcept>

namespace simpleslam::app::run_dir {

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

void writeOutcome(const fs::path& dir, const modules::RunOutcome& outcome,
                  const RunCounts& counts) {
    const auto flag = [](bool value) { return value ? "true" : "false"; };
    writeText(dir / "outcome.yaml",
              std::format("complete: {}\ninput_complete: {}\nprocessing_complete: {}\n"
                          "output_complete: {}\npackets: {}\nframes: {}\nkeyframes: {}\n"
                          "log_dropped: {}\nreasons: {}\nnotes: {}\n",
                          flag(outcome.complete()), flag(outcome.inputComplete()),
                          flag(outcome.processingComplete()), flag(outcome.outputComplete()),
                          counts.packets, counts.frames, counts.keyframes, counts.log_dropped,
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

ResultSummary readResult(const fs::path& dir) {
    const fs::path path = dir / "result.json";
    if (!fs::exists(path)) throw std::runtime_error("没有结论: " + path.string());
    YAML::Node root;
    try {
        root = YAML::LoadFile(path.string());  // JSON 是 YAML 的子集
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("读不懂结论 " + path.string() + ": " + e.what());
    }
    if (!root["schema"] || root["schema"].as<int>() != kResultSchema) {
        throw std::runtime_error(std::format("结论 {} 的格式版本不是 {}", path.string(),
                                             kResultSchema));
    }
    ResultSummary summary;
    summary.status = root["status"].as<std::string>();
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
