// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// @file config_tool.cpp
/// simpleslam-config：任务配置的命令行入口，只认内置实现。
///
/// 它存在的理由是配置的真源在 C++，所以 schema 也必须由 C++ 吐出来。Python 侧读这份
/// 输出做校验与补全，因此两边不会各自漂移。用户写的实现要经用户自己的入口
/// （`app::run`）检查，那里的目录里才有它们。

#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/run.hpp>

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    const int code = simpleslam::app::configCommand(args, simpleslam::app::builtinCatalogs(),
                                                    std::cout, std::cerr);
    if (code != -1) return code;
    std::cerr << "simpleslam-config 只做配置命令：schema / default / check / dump / hash\n";
    return 2;
}
