// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file construction.hpp
/// 插件机制用的部分：槽构造实现时，怎么把来源交给实现的基类。写算法的人用不到它。
///
/// 基类 `Loggable` 先于实现构造，它拿不到实现的配置；所以由槽在构造前设上构造期上下文，
/// 基类从里面取自己的来源节点。构造期间新建的子插件挂在正在构造的那个对象下面；跑起来
/// 以后才新建的子插件，由父插件交来的槽记着父插件是谁。

#include <SimpleSLAM/logging/logger.hpp>

#include <string_view>

namespace simpleslam::logging::detail {

/// 新建一个组件的来源节点。`parent` 为空时是没接上的；`field` 是槽名（键的一段），`type` 是
/// 实现类型。父节点是根（模块、执行器）时显示成类型，否则显示成 `槽名(类型)`。
[[nodiscard]] NodePtr componentNode(const NodePtr& parent, std::string_view field,
                                    std::string_view type);
/// 日志对象的来源节点。
[[nodiscard]] NodePtr nodeOf(const Logger& logger) noexcept;

/// 构造期上下文，按线程嵌套。
class ConstructionScope {
public:
    explicit ConstructionScope(NodePtr node) noexcept;
    ~ConstructionScope();
    ConstructionScope(const ConstructionScope&) = delete;
    ConstructionScope& operator=(const ConstructionScope&) = delete;

    /// 正在构造的对象的来源节点；不在构造期间时为空。
    [[nodiscard]] static NodePtr current() noexcept;
    /// 基类取自己的来源节点：第一个取到槽准备好的那个；之后再有（按值嵌着的组件）是它下面
    /// 不具名的一层，记录算在主人头上；不在构造期间是没接上的。
    [[nodiscard]] static NodePtr take();

private:
    NodePtr node_;
    bool taken_ = false;
    ConstructionScope* outer_;
};

/// 试构造：装配时为了提前报参数错误而构造的那一次。期间的记录全部丢掉，不会多出一份。
class MutedConstruction {
public:
    MutedConstruction();

private:
    ConstructionScope scope_;
};

}  // namespace simpleslam::logging::detail
