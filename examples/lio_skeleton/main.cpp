// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// @file main.cpp
/// 一个能跑的入口：登记你的算法，其余交给任务执行器。
///
/// 任务执行器（`app/run.hpp`）负责读任务与系统 YAML、打开数据集、合并标定、按管线装配、
/// 跑完、写任务档案、对真值评测。这个文件里没有算法：算法在 my_lio.hpp 的 `track()` 里，
/// 换掉它就行。
///
/// 用法：
///   simpleslam-lio-skeleton configs/tasks/kitti_odometry.yaml
///       data.source.path=/data/kitti/odometry/sequences/00   （覆盖项接在同一行后面）
///   simpleslam-lio-skeleton check configs/tasks/kitti_odometry.yaml

#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/run.hpp>
#include <SimpleSLAM/plugins/odometry.hpp>

#include "my_lio.hpp"

int main(int argc, char** argv) {
    auto catalogs = simpleslam::app::builtinCatalogs();
    catalogs.of<simpleslam::plugin::OdometryBase>().add<example::MyLio>();
    return simpleslam::app::run(argc, argv, catalogs);
}
