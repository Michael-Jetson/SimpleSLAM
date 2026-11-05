// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// @file config_tool.cpp
/// simpleslam-config：配置的命令行入口。
///
/// 它存在的理由只有一个——配置的真源在 C++，所以 schema 也必须由
/// C++ 吐出来。Python 侧读这份输出做校验与补全，因此两边不会各自漂移。

#include <SimpleSLAM/config/loader.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

int usage() {
    std::cerr << R"(用法: simpleslam-config <命令> [参数]

命令:
  schema                     导出配置 schema（JSON），供 Python 与工具使用
  check <配置.yaml> [覆盖...]  校验配置；未知键与类型不符都会报错
  dump  <配置.yaml> [覆盖...]  输出规范化配置（键序固定，配置哈希基于它）
  hash  <配置.yaml> [覆盖...]  输出配置哈希
  default                    输出全部默认值的配置

覆盖写成 路径=值，例如 odometry.max_iterations=40
)";
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string command = argv[1];
    std::vector<std::string> rest(argv + 2, argv + argc);

    try {
        if (command == "schema") {
            std::cout << simpleslam::config::schemaJson();
            return 0;
        }
        if (command == "default") {
            std::cout << simpleslam::config::dumpConfig(simpleslam::config::SlamConfig{});
            return 0;
        }
        if (rest.empty()) return usage();

        const std::string path = rest.front();
        const std::vector<std::string> overrides(rest.begin() + 1, rest.end());
        const auto config = simpleslam::config::loadConfigFile(path, overrides);

        if (command == "check") {
            std::cout << "配置有效: " << path << "\n哈希: "
                      << simpleslam::config::configHash(config) << '\n';
            return 0;
        }
        if (command == "dump") {
            std::cout << simpleslam::config::dumpConfig(config);
            return 0;
        }
        if (command == "hash") {
            std::cout << simpleslam::config::configHash(config) << '\n';
            return 0;
        }
        return usage();
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << '\n';
        return 1;
    }
}
