#pragma once

/// @file config.hpp
/// 配置系统——层级化 YAML 配置加载与访问
///
/// 特性：
///   - 层级化加载：defaults.yaml -> dataset.yaml -> experiment.yaml -> CLI
///   - dot-path 访问（如 "odometry.iekf.max_iterations"）
///   - schema 版本校验
///   - 深度合并（overlay 覆盖 base）

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace simpleslam {

/// 配置加载/访问异常
class ConfigError final : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

class Config final {
public:
    /// 从单个 YAML 文件加载
    static Config load(const std::string& path);

    /// 从基础文件 + 多个覆盖层加载（后者优先级更高）
    static Config loadWithOverlays(const std::string& base_path,
                                   const std::vector<std::string>& overlay_paths);

    /// 通过 dot-path 获取值，路径不存在则抛出 ConfigError
    template <typename T>
    T get(const std::string& dot_path) const {
        auto node = navigate(dot_path);
        if (!node || node.IsNull()) {
            throw ConfigError("Config key not found: " + dot_path);
        }
        try {
            return node.as<T>();
        } catch (const YAML::Exception& e) {
            throw ConfigError("Config type conversion failed: " + dot_path + " (" + e.what() + ")");
        }
    }

    /// 通过 dot-path 获取值，路径不存在则返回默认值
    template <typename T>
    T get(const std::string& dot_path, const T& fallback) const {
        auto node = navigate(dot_path);
        if (!node || node.IsNull()) {
            return fallback;
        }
        try {
            return node.as<T>();
        } catch (const YAML::Exception&) {
            return fallback;
        }
    }

    /// 检查 dot-path 是否存在
    bool has(const std::string& dot_path) const;

    /// 获取指定路径的原始 YAML 节点
    YAML::Node node(const std::string& dot_path) const;

    /// 获取 schema 版本号
    int schemaVersion() const;

    /// 获取根节点
    const YAML::Node& root() const { return root_; }

private:
    YAML::Node root_;

    /// 深度合并两个 YAML 节点（overlay 的值覆盖 base）
    static YAML::Node merge(const YAML::Node& base, const YAML::Node& overlay);

    /// 按 dot-path 导航到目标节点（如 "logging.levels.odometry"）
    YAML::Node navigate(const std::string& dot_path) const;
};

}  // namespace simpleslam
