// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file frame_context.hpp
/// 一帧的处理上下文：这一帧成立所依赖的那些"当时的版本"。
///
/// 插件不碰资源（`docs/architecture.md` 三分法）。算法需要外参、需要知道自己在哪一代参数下运行，
/// 但它不该持有资源句柄去自行取快照——那样一帧之内可以取到两份不同的标定，
/// 产物上标的又是第三份。
///
/// 所以由模块在帧边界取**一次**快照，打包成这个不可变上下文传进算法：
/// 算法用它、产物标它，三者必然是同一份。
///
/// 参数同理：带的是**快照本身**，不只是版本号。算法每帧从这里读阈值，结果上标的
/// 参数版本才名副其实——只传版本号、值却在启动时拷进了成员的话，版本变了，
/// 用的值没变，标签就是假的。

#include <SimpleSLAM/comm/parameters.hpp>
#include <SimpleSLAM/framework/calibration.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace simpleslam::fw {

struct FrameContext {
    /// 本帧生效的标定快照。内容不可变，版本号在 `calibration->revision`。
    Calibration::Snapshot calibration;
    /// 本帧生效的参数快照。内容不可变。
    comm::ParamSnapshotPtr params;
    /// 本帧生效的参数版本（等于 params->revision）。
    std::uint64_t parameter_revision = 0;
    /// 当前估计状态与包的传播起点之间缺少的输入。来源包本身不改写；
    /// 框架将这段缺口纳入 supportGapPolicy，降级算法可在 context() 里检查它。
    std::optional<TimeRange> missing_propagation;

    /// 从本帧的参数快照取值；没有这个名字或类型不符时返回默认值。
    template <class T>
    [[nodiscard]] T param(const std::string& name, T default_value) const {
        if (!params) return default_value;
        if (auto value = params->get<T>(name)) return *value;
        return default_value;
    }

    [[nodiscard]] std::uint64_t calibrationRevision() const noexcept {
        return calibration ? calibration->revision : 0;
    }

    /// 取外参。未登记或来源未定时返回空——这一点与 CalibrationData 一致：
    /// 默认成单位阵会把"忘了标定"变成"标定过"。
    [[nodiscard]] std::optional<SE3d> T_body_sensor(const SensorName& sensor) const {
        if (!calibration) return std::nullopt;
        return calibration->T_body_sensor(sensor);
    }
};

}  // namespace simpleslam::fw
