// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/app/task.hpp>
#include <SimpleSLAM/modules/slam_system.hpp>

#include <algorithm>
#include <string_view>
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
    task.notes = std::move(loaded.renames.notes);
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

    // 参数：收齐选中的组件与模块声明的，核对配置里的值，没写的补上默认值。管线里没有的模块
    // 与插件不算：它们声明的参数这次没人用
    const bool backend = task.config.system.pipeline == "mapping";
    std::vector<comm::ParamComponent> components = modules::builtinModuleParams(backend);
    const auto add = [&](std::string_view path, const auto& slot, std::string_view module) {
        if (slot.params().empty()) return;
        components.push_back(comm::ParamComponent{
            .component = std::string(path) + "（" + slot.type() + "）",
            .module = std::string(module),
            .uses = slot.params()});
    };
    add("system.modules.odometry.plugin", task.odometry, modules::kOdometryModule);
    if (backend) {
        add("system.modules.place_recognizer.plugin", task.place_recognizer,
            modules::kPlaceRecognizerModule);
        add("system.modules.graph_optimizer.plugin", task.graph_optimizer,
            modules::kGraphOptimizerModule);
    }
    // 旧写法给的参数这次没有组件声明（只跑里程计的管线里没有图优化模块）：旧写法当初在这样的
    // 管线里不起作用、也不报错，v5 的任务记录里都写着它们。照旧，只说一句；新写法照常报错
    for (const auto& name : loaded.renames.params) {
        const bool declared = std::ranges::any_of(components, [&](const auto& component) {
            return std::ranges::any_of(component.uses,
                                       [&](const auto& use) { return use.name == name; });
        });
        if (!declared && task.config.system.params.erase(name) > 0) {
            task.notes.push_back("system.params." + name
                                 + " 这次没有组件用到（管线里没有声明它的模块），这一项不起作用");
        }
    }
    try {
        auto resolved = comm::resolveParams(components, task.config.system.params);
        task.config.system.params = std::move(resolved.values);
        task.params = std::move(resolved.definitions);
    } catch (const comm::ParameterError& e) {
        throw config::ConfigError(std::string("system.params：") + e.what());
    }
    return task;
}

ResolvedTask loadTask(const std::string& path, const std::vector<std::string>& overrides,
                      const config::Catalogs& catalogs) {
    return resolveTask(config::loadTaskFile(path, overrides), catalogs);
}

}  // namespace simpleslam::app
