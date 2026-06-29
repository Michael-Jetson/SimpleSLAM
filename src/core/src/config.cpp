/// @file config.cpp
/// 配置系统实现

#include <SimpleSLAM/core/infra/config.hpp>

#include <string_view>

namespace simpleslam {

Config Config::load(const std::string& path) {
    Config cfg;
    try {
        cfg.root_ = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw ConfigError("Cannot load config: " + path + " (" + e.what() + ")");
    }
    return cfg;
}

Config Config::loadWithOverlays(const std::string& base_path,
                                const std::vector<std::string>& overlay_paths) {
    auto cfg = load(base_path);
    for (const auto& path : overlay_paths) {
        try {
            cfg.root_ = merge(cfg.root_, YAML::LoadFile(path));
        } catch (const YAML::Exception& e) {
            throw ConfigError("Cannot load overlay: " + path + " (" + e.what() + ")");
        }
    }
    return cfg;
}

bool Config::has(const std::string& dot_path) const {
    auto node = navigate(dot_path);
    return node && !node.IsNull();
}

YAML::Node Config::node(const std::string& dot_path) const {
    return navigate(dot_path);
}

int Config::schemaVersion() const {
    auto node = navigate("schema_version");
    if (!node || node.IsNull()) {
        throw ConfigError("Missing required field: schema_version");
    }
    return node.as<int>();
}

YAML::Node Config::merge(const YAML::Node& base, const YAML::Node& overlay) {
    if (!overlay.IsMap() || !base.IsMap()) {
        return YAML::Clone(overlay);
    }
    // 两者都是 Map：逐键深度合并，overlay 优先
    YAML::Node result = YAML::Clone(base);
    for (auto it = overlay.begin(); it != overlay.end(); ++it) {
        const auto key = it->first.as<std::string>();
        if (result[key] && result[key].IsMap() && it->second.IsMap()) {
            result[key] = merge(result[key], it->second);
        } else {
            result[key] = YAML::Clone(it->second);
        }
    }
    return result;
}

YAML::Node Config::navigate(const std::string& dot_path) const {
    if (dot_path.empty()) return root_;

    // 只读查询：引用语义安全，不需要 Clone
    // yaml-cpp 的 operator[] 在只读场景下返回的是内部节点的引用视图
    // 用 reset() 切换引用目标，避免赋值操作的副作用
    YAML::Node current = root_;
    std::string_view remaining(dot_path);

    while (!remaining.empty()) {
        auto dot = remaining.find('.');
        auto key = std::string(remaining.substr(0, dot));
        remaining = (dot == std::string_view::npos) ? "" : remaining.substr(dot + 1);

        if (!current.IsMap() || !current[key]) {
            return {};
        }
        current.reset(current[key]);
    }
    return current;
}

}  // namespace simpleslam
