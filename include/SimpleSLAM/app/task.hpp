// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file task.hpp
/// 任务的还原：配置里的插件节点交给目录还原成实现，并换成补齐了默认值的节点。
///
/// 还原后的任务是这次运行的唯一输入：任务记录里的 `task.yaml` 就是它的导出，原样能重跑。
/// 写不写默认值，得到的是同一份导出。参数也在这里核对：收齐选中的组件与模块声明的参数，
/// `system.params` 里的值按声明查名字、类型与范围，没写的补上默认值。

#include <SimpleSLAM/comm/parameters.hpp>
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
    config::Slot<plugin::DatasetReaderBase> data;
    config::Slot<plugin::OdometryBase> odometry;
    config::Slot<plugin::PlaceRecognizerBase> place_recognizer;
    config::Slot<plugin::GraphOptimizerBase> graph_optimizer;
    /// 这次跑的评测函数，按名字。空表示不评测。
    std::map<std::string, config::Slot<eval::EvaluatorBase>> evaluators;
    /// 参数的定义：选中的组件与模块声明的，连同写者。值在 `config.system.params`，已补齐。
    std::vector<comm::ParamDefinition> params;
};

/// 还原全部插件节点并核对参数。名字不在所要求接口的目录里、参数不合法、构造失败、
/// `system.params` 写了没人声明的参数或值不合格，都在这里报错，带完整路径与可选的实现。
[[nodiscard]] ResolvedTask resolveTask(config::LoadedTask loaded,
                                       const config::Catalogs& catalogs);

/// 读任务文件并还原。
[[nodiscard]] ResolvedTask loadTask(const std::string& path,
                                    const std::vector<std::string>& overrides,
                                    const config::Catalogs& catalogs);

/// 只还原评测函数，节点换成补齐了默认值的那一份；数据、方案与后端的插件不碰。
[[nodiscard]] ResolvedTask resolveEvaluators(config::TaskConfig config,
                                             const config::Catalogs& catalogs);

/// 单独评测用：只读任务记录里 `task.yaml` 评测要用的几节（`config::loadEvaluationView`），
/// 只还原评测函数。入口的目录里没有当初的算法、系统文件的写法变了，都照样能评。
[[nodiscard]] ResolvedTask loadEvaluationTask(const std::string& path,
                                              const std::vector<std::string>& overrides,
                                              const config::Catalogs& catalogs);

}  // namespace simpleslam::app
