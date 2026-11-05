// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/algo/builtins.hpp>
#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/replay_reader.hpp>
#include <SimpleSLAM/plugins/dataset.hpp>
#include <SimpleSLAM/plugins/graph_optimizer.hpp>
#include <SimpleSLAM/plugins/odometry.hpp>
#include <SimpleSLAM/plugins/place_recognizer.hpp>
#include <SimpleSLAM/plugins/sources/kitti_dataset.hpp>

#include <string>

namespace simpleslam::app {

config::Catalogs builtinCatalogs() {
    config::Catalogs catalogs;
    algo::registerBuiltins(catalogs);

    auto& readers = catalogs.of<plugin::DatasetReader>();
    readers.add<plugin::kitti::OdometryReader>();
    readers.add<plugin::kitti::RawReader>();
    readers.add<ReplayReader>();

    catalogs.of<plugin::PlaceRecognizerBase>().add<plugin::NoPlaceRecognizer>();
    catalogs.of<plugin::GraphOptimizerBase>().add<plugin::NoGraphOptimizer>();
    // 空目录：方案没有内置实现，用户在入口追加
    (void)catalogs.of<plugin::OdometryBase>();
    return catalogs;
}

std::map<std::string, std::string> slotInterfaces() {
    return {
        {"data.source", std::string(plugin::DatasetReader::kInterfaceName)},
        {"system.modules.graph_optimizer.plugin",
         std::string(plugin::GraphOptimizerBase::kInterfaceName)},
        {"system.modules.odometry.plugin", std::string(plugin::OdometryBase::kInterfaceName)},
        {"system.modules.place_recognizer.plugin",
         std::string(plugin::PlaceRecognizerBase::kInterfaceName)},
    };
}

}  // namespace simpleslam::app
