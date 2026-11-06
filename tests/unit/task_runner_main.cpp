// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 执行器测试用的入口：登记测试用的里程计与评测函数，再交给执行器。多序列为每个序列起的
/// 子进程是它。

#include "count_evaluator.hpp"
#include "still_odometry.hpp"

#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/run.hpp>

int main(int argc, char** argv) {
    auto catalogs = simpleslam::app::builtinCatalogs();
    catalogs.of<simpleslam::plugin::OdometryBase>().add<simpleslam::test::StillOdometry>();
    catalogs.of<simpleslam::eval::EvaluatorBase>().add<simpleslam::test::CountEvaluator>();
    return simpleslam::app::run(argc, argv, catalogs);
}
