// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file task.hpp
/// 任务的还原：配置里的插件节点交给目录还原成实现，并换成补齐了默认值的节点。
///
/// 还原后的任务是这次运行的唯一输入：任务记录里的 `task.yaml` 就是它的导出，原样能重跑。
/// 写不写默认值，得到的是同一份导出。

#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/config/plugin.hpp>
#include <SimpleSLAM/eval/evaluator.hpp>
#include <SimpleSLAM/plugins/dataset.hpp>
#include <SimpleSLAM/plugins/graph_optimizer.hpp>
#include <SimpleSLAM/plugins/odometry.hpp>
#include <SimpleSLAM/plugins/place_recognizer.hpp>

#include <map>
#include <string>
#include <vector>

namespace simpleslam::app {

struct ResolvedTask {
    /// 插件节点已换成还原后的节点。
    config::TaskConfig config;
    config::TaskSource source;
    config::Slot<plugin::DatasetReader> data;
    config::Slot<plugin::OdometryBase> odometry;
    config::Slot<plugin::PlaceRecognizerBase> place_recognizer;
    config::Slot<plugin::GraphOptimizerBase> graph_optimizer;
    /// 这次跑的评测函数，按名字。空表示不评测。
    std::map<std::string, config::Slot<eval::EvaluatorBase>> evaluators;
};

/// 还原全部插件节点。名字不在所要求接口的目录里、参数不合法、构造失败，都在这里报错，
/// 带完整路径与可选的实现。
[[nodiscard]] ResolvedTask resolveTask(config::LoadedTask loaded,
                                       const config::Catalogs& catalogs);

/// 读任务文件并还原。
[[nodiscard]] ResolvedTask loadTask(const std::string& path,
                                    const std::vector<std::string>& overrides,
                                    const config::Catalogs& catalogs);

}  // namespace simpleslam::app
