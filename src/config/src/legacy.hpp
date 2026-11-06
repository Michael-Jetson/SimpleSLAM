// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file legacy.hpp
/// 旧写法的换算：v2 的任务写法翻译成现在的写法。配置层内部的头文件，不是公共接口。
///
/// 换算只在加载器这一处：手写的任务、Python 渲染出的任务、命令行覆盖项、任务记录里的
/// `task.yaml` 都经过它。换算是纯函数，新旧两种写法得到同一个规范化导出与哈希。
/// 每处换算写一条说明，带上替换用的新写法；换算表以外的未知键照样报错。

#include <SimpleSLAM/config/config.hpp>

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace simpleslam::config::legacy {

/// 换算表认得的旧路径，与它在新写法里的去处。schema 导出它，Python 据此认得旧键。
[[nodiscard]] const std::vector<std::pair<std::string, std::string>>& fields();

/// 任务文件里的旧写法就地换成新写法。在查未知键之前调用；没有旧写法时什么都不做。
void upgradeTree(YAML::Node& root, const std::string& origin, std::vector<std::string>& notes);

/// 一条旧路径的覆盖项换成的新写法。
struct Translated {
    std::vector<std::string> assignments;  ///< 新写法的覆盖项，可能为空（不必改）
    bool rpe_deltas = false;  ///< 改了 RPE 的间隔：应用之后要看两个间隔是不是都为 0
};

/// 覆盖项的路径是旧路径就换算，不是返回空。要看当前的配置：改 `ate` 的参数先得有 `ate`。
[[nodiscard]] std::optional<Translated> translateAssignment(const TaskConfig& config,
                                                            const std::string& path,
                                                            const std::string& text,
                                                            std::vector<std::string>& notes);

/// v2 里 RPE 的两个间隔都为 0 表示不算 RPE：应用完旧写法的间隔之后，按这个意思去掉 `rpe`。
void dropRpeWithoutDelta(TaskConfig& config, std::vector<std::string>& notes);

}  // namespace simpleslam::config::legacy
