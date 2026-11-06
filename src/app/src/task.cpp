// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/app/task.hpp>

#include <utility>

namespace simpleslam::app {

namespace {

/// 还原一个插件字段，并把配置里的节点换成补齐了默认值的那一份。
template <class Interface>
config::Slot<Interface> resolveField(config::PluginSpec& spec, const char* path,
                                     const config::Catalogs& catalogs) {
    auto slot = catalogs.resolve<Interface>(spec, path);
    spec = slot.resolved();
    return slot;
}

}  // namespace

ResolvedTask resolveTask(config::LoadedTask loaded, const config::Catalogs& catalogs) {
    ResolvedTask task;
    task.config = std::move(loaded.config);
    task.source = std::move(loaded.source);
    auto& modules = task.config.system.modules;
    task.data =
        resolveField<plugin::DatasetReader>(task.config.data.source, "data.source", catalogs);
    task.odometry = resolveField<plugin::OdometryBase>(
        modules.odometry.plugin, "system.modules.odometry.plugin", catalogs);
    task.place_recognizer = resolveField<plugin::PlaceRecognizerBase>(
        modules.place_recognizer.plugin, "system.modules.place_recognizer.plugin", catalogs);
    task.graph_optimizer = resolveField<plugin::GraphOptimizerBase>(
        modules.graph_optimizer.plugin, "system.modules.graph_optimizer.plugin", catalogs);
    for (auto& [name, spec] : task.config.evaluation.evaluators) {
        const std::string path = "evaluation.evaluators." + name;
        task.evaluators.emplace(name,
                                resolveField<eval::EvaluatorBase>(spec, path.c_str(), catalogs));
    }
    return task;
}

ResolvedTask loadTask(const std::string& path, const std::vector<std::string>& overrides,
                      const config::Catalogs& catalogs) {
    return resolveTask(config::loadTaskFile(path, overrides), catalogs);
}

}  // namespace simpleslam::app
