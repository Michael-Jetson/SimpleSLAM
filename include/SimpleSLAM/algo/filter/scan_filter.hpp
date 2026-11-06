// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file scan_filter.hpp
/// 扫描滤波：一帧点云进、一帧点云出，与估计状态无关。
///
/// 滤波没有状态，`apply()` 是 const，同样的输入得到同样的输出。公开入口非虚（NVI）：先核对
/// 输入各列长度一致、坐标有限，调实现，再核对输出同样成立；实现的错在接缝处就被拦下。
/// 输出不多于输入；可选列（强度、逐点时间、线号）跟着保留下来的点一起走。

#include <SimpleSLAM/framework/sensor_data.hpp>
#include <SimpleSLAM/logging/logger.hpp>

#include <string_view>

namespace simpleslam::algo {

class ScanFilter : public logging::Loggable {
public:
    static constexpr std::string_view kInterfaceName = "ScanFilter";

    ScanFilter() = default;
    ScanFilter(const ScanFilter&) = delete;
    ScanFilter& operator=(const ScanFilter&) = delete;
    virtual ~ScanFilter() = default;

    /// 输入列长度不一致或有非有限坐标时抛 `std::invalid_argument`；
    /// 实现的输出不合法时抛 `std::logic_error`。
    [[nodiscard]] fw::PointCloud apply(const fw::PointCloud& in) const;

    [[nodiscard]] virtual std::string_view typeName() const noexcept = 0;

private:
    [[nodiscard]] virtual fw::PointCloud doApply(const fw::PointCloud& in) const = 0;
};

/// 原样输出。槽可选时用它占位，免得到处判断"有没有滤波"。
class PassThrough final : public ScanFilter {
public:
    static constexpr std::string_view kTypeName = "none";

    struct Config {
        template <class V>
        void visit(V& /*v*/) {}
    };

    explicit PassThrough(const Config& /*config*/ = {}) {}

    [[nodiscard]] std::string_view typeName() const noexcept override { return kTypeName; }

private:
    [[nodiscard]] fw::PointCloud doApply(const fw::PointCloud& in) const override { return in; }
};

/// 体素降采样：每个体素留第一个落进来的点。
///
/// 留"第一个点"而不是体素中心或均值：结果是原始测量里的点，可选列不用插值；
/// 只取决于输入顺序，同样的输入每次得到同样的输出。KISS-ICP 也是这样做的。
class VoxelDownsample final : public ScanFilter {
public:
    static constexpr std::string_view kTypeName = "voxel_downsample";

    struct Config {
        double voxel_size_m = 0.5;

        template <class V>
        void visit(V& v) {
            v("voxel_size_m", voxel_size_m, "体素边长（米），每个体素只留一个点");
        }
    };

    /// 体素边长必须是正的有限数，否则抛 `std::invalid_argument`。
    explicit VoxelDownsample(const Config& config);

    [[nodiscard]] std::string_view typeName() const noexcept override { return kTypeName; }

private:
    [[nodiscard]] fw::PointCloud doApply(const fw::PointCloud& in) const override;

    Config config_;
};

}  // namespace simpleslam::algo
