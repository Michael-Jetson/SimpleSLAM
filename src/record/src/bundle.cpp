// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/record/bundle.hpp>

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstdio>
#include <fstream>

namespace simpleslam::record {

const InputDigest* ExperimentBundle::input(const std::string& role) const {
    for (const auto& entry : inputs) {
        if (entry.role == role) return &entry;
    }
    return nullptr;
}

std::string fileDigest(const std::string& path, std::uint64_t* size_out) {
    std::ifstream file(path, std::ios::binary);
    if (!file) throw BundleError("打不开待摘要的文件: " + path);

    std::uint64_t hash = 0xcbf29ce484222325ULL;  // FNV-1a 64 偏移基
    std::uint64_t total = 0;
    std::array<char, 1 << 16> buffer{};
    while (file) {
        file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const auto got = static_cast<std::size_t>(file.gcount());
        for (std::size_t i = 0; i < got; ++i) {
            hash ^= static_cast<unsigned char>(buffer[i]);
            hash *= 0x100000001b3ULL;  // FNV-1a 64 质数
        }
        total += got;
    }
    if (size_out != nullptr) *size_out = total;

    char text[24];
    std::snprintf(text, sizeof(text), "%016llx", static_cast<unsigned long long>(hash));
    return text;
}

void writeBundle(const std::string& path, const ExperimentBundle& bundle) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << bundle.name;
    out << YAML::Key << "created_at" << YAML::Value << bundle.created_at;
    out << YAML::Key << "code_revision" << YAML::Value << bundle.code_revision;
    out << YAML::Key << "config_hash" << YAML::Value << bundle.config_hash;
    out << YAML::Key << "seed" << YAML::Value << bundle.seed;
    out << YAML::Key << "session" << YAML::Value << bundle.session;
    out << YAML::Key << "inputs" << YAML::Value << YAML::BeginSeq;
    for (const auto& input : bundle.inputs) {
        out << YAML::BeginMap;
        out << YAML::Key << "role" << YAML::Value << input.role;
        out << YAML::Key << "path" << YAML::Value << input.path;
        out << YAML::Key << "digest" << YAML::Value << input.digest;
        out << YAML::Key << "bytes" << YAML::Value << input.bytes;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "metrics" << YAML::Value << YAML::BeginMap;
    for (const auto& [key, value] : bundle.metrics) out << YAML::Key << key << YAML::Value << value;
    out << YAML::EndMap;
    if (!bundle.note.empty()) out << YAML::Key << "note" << YAML::Value << bundle.note;
    out << YAML::EndMap;

    std::ofstream file(path);
    if (!file) throw BundleError("打不开回放包输出文件: " + path);
    file << out.c_str() << '\n';
    if (!file) throw BundleError("写回放包失败: " + path);
}

ExperimentBundle readBundle(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw BundleError("读取回放包失败 " + path + ": " + e.what());
    }
    ExperimentBundle bundle;
    if (root["name"]) bundle.name = root["name"].as<std::string>();
    if (root["created_at"]) bundle.created_at = root["created_at"].as<std::string>();
    if (root["code_revision"]) bundle.code_revision = root["code_revision"].as<std::string>();
    if (root["config_hash"]) bundle.config_hash = root["config_hash"].as<std::string>();
    if (root["seed"]) bundle.seed = root["seed"].as<std::int64_t>();
    if (root["session"]) bundle.session = root["session"].as<std::uint64_t>();
    if (root["note"]) bundle.note = root["note"].as<std::string>();
    for (const auto& entry : root["inputs"]) {
        InputDigest input;
        input.role = entry["role"].as<std::string>();
        if (entry["path"]) input.path = entry["path"].as<std::string>();
        if (entry["digest"]) input.digest = entry["digest"].as<std::string>();
        if (entry["bytes"]) input.bytes = entry["bytes"].as<std::uint64_t>();
        bundle.inputs.push_back(std::move(input));
    }
    for (const auto& item : root["metrics"]) {
        bundle.metrics.emplace(item.first.as<std::string>(), item.second.as<double>());
    }
    return bundle;
}

std::vector<std::string> diffBundles(const ExperimentBundle& before,
                                     const ExperimentBundle& after) {
    std::vector<std::string> differences;
    if (before.config_hash != after.config_hash) {
        differences.push_back("配置变了: " + before.config_hash + " → " + after.config_hash);
    }
    if (before.seed != after.seed) {
        differences.push_back("种子变了: " + std::to_string(before.seed) + " → "
                              + std::to_string(after.seed));
    }
    if (before.code_revision != after.code_revision) {
        differences.push_back("代码变了: " + before.code_revision + " → " + after.code_revision);
    }
    for (const auto& input : before.inputs) {
        const auto* other = after.input(input.role);
        if (other == nullptr) {
            differences.push_back("输入缺失: " + input.role);
        } else if (other->digest != input.digest) {
            differences.push_back("输入内容变了: " + input.role);
        }
    }
    for (const auto& input : after.inputs) {
        if (before.input(input.role) == nullptr) {
            differences.push_back("多了输入: " + input.role);
        }
    }
    return differences;
}

}  // namespace simpleslam::record
