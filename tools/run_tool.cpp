// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// @file run_tool.cpp
/// simpleslam：用内置实现跑任务，也能校验、导出任务配置。
///
/// 要用自己写的算法，照 `examples/lio_skeleton/main.cpp` 写一个入口：
/// 先把实现追加进目录，再调用 `app::run`。

#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/run.hpp>

int main(int argc, char** argv) {
    return simpleslam::app::run(argc, argv, simpleslam::app::builtinCatalogs());
}
