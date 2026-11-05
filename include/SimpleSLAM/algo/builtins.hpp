// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file builtins.hpp
/// 算法库的内置实现：每个接口一份类型表，同时驱动目录登记与契约测试。
///
/// 加一个内置实现只改这里一行，它就既能在 YAML 里用 `type` 选到，也自动受契约测试。
/// 用户自己的实现不进这张表：应用入口在 `registerBuiltins()` 之后追加到目录里。

#include <SimpleSLAM/algo/filter/scan_filter.hpp>
#include <SimpleSLAM/algo/map/voxel_point_map.hpp>
#include <SimpleSLAM/config/plugin.hpp>

#include <tuple>

namespace simpleslam::algo {

using BuiltinScanFilters = std::tuple<PassThrough, VoxelDownsample>;
using BuiltinMaps = std::tuple<VoxelPointMap>;

namespace detail {
template <class I, class... Ts>
void addAll(config::Catalog<I>& catalog, const std::tuple<Ts...>* /*list*/) {
    (catalog.template add<Ts>(), ...);
}
}  // namespace detail

/// 登记全部内置实现。名字重复时抛 `config::ConfigError`。
inline void registerBuiltins(config::Catalogs& catalogs) {
    detail::addAll(catalogs.of<ScanFilter>(), static_cast<const BuiltinScanFilters*>(nullptr));
    detail::addAll(catalogs.of<LinearizableMap>(), static_cast<const BuiltinMaps*>(nullptr));
}

}  // namespace simpleslam::algo
