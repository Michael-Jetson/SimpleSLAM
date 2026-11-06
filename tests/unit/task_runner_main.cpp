// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 执行器测试用的入口：登记测试用的里程计，再交给执行器。多序列为每个序列起的子进程是它。

#include "still_odometry.hpp"

#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/run.hpp>

int main(int argc, char** argv) {
    auto catalogs = simpleslam::app::builtinCatalogs();
    catalogs.of<simpleslam::plugin::OdometryBase>().add<simpleslam::test::StillOdometry>();
    return simpleslam::app::run(argc, argv, catalogs);
}
