// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/comm/types.hpp>

#include <string>
#include <string_view>

namespace simpleslam::comm {
namespace {

/// 折叠重复斜杠并去掉末尾斜杠（根 "/" 除外）。
std::string normalize(std::string_view raw) {
    std::string out;
    out.reserve(raw.size() + 1);
    for (char c : raw) {
        if (c == '/' && !out.empty() && out.back() == '/') continue;
        out.push_back(c);
    }
    if (out.size() > 1 && out.back() == '/') out.pop_back();
    return out;
}

}  // namespace

std::string resolveName(std::string_view name, std::string_view module_name) {
    if (name.empty()) return std::string(module_name);

    if (name.front() == '/') return normalize(name);

    const std::string module = normalize(module_name);

    if (name.front() == '~') {
        std::string_view rest = name.substr(1);
        if (!rest.empty() && rest.front() == '/') rest.remove_prefix(1);
        if (rest.empty()) return module;
        return normalize(module + "/" + std::string(rest));
    }

    // 相对名解析到模块所在的命名空间
    const auto slash = module.rfind('/');
    const std::string ns = (slash == 0 || slash == std::string::npos) ? "" : module.substr(0, slash);
    return normalize(ns + "/" + std::string(name));
}

bool isValidResolvedName(std::string_view name) noexcept {
    if (name.empty() || name.front() != '/') return false;
    if (name.size() > 1 && name.back() == '/') return false;
    char previous = '\0';
    for (char c : name) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')
                        || c == '_' || c == '/';
        if (!ok) return false;
        if (c == '/' && previous == '/') return false;  // 空段
        previous = c;
    }
    return true;
}

}  // namespace simpleslam::comm
