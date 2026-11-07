// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file dataset.hpp
/// 数据集读取器：一个任务的数据从这里来。
///
/// 读取器把四样东西一起给出来：来源、自带的标定、真值（可以没有）、推荐的打包策略。
/// 分开取，就会出现"换了数据集却忘了改打包策略"这种错。它是任务里 `data.source`
/// 那个槽的接口，按类型名在目录里选，参数按实现的字段清单严格校验。

#include <SimpleSLAM/framework/calibration.hpp>
#include <SimpleSLAM/framework/sensor_source.hpp>
#include <SimpleSLAM/framework/time.hpp>
#include <SimpleSLAM/framework/trajectory.hpp>
#include <SimpleSLAM/logging/logger.hpp>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace simpleslam::plugin {

namespace fw = simpleslam::fw;

/// 打开一份数据集的结果。
struct Dataset {
    std::unique_ptr<fw::ISensorSource> source;
    fw::CalibrationData calibration;
    std::optional<fw::Trajectory> ground_truth;
    /// 读取器按实测数据给出的推荐打包策略。给不出来时为空，用系统配置的。
    std::optional<fw::PacketizerPolicy> recommended_policy;
    /// 帧数。不读一遍就不知道时为空：零帧与不知道不能长得一样。
    std::optional<std::size_t> frames;
    /// 评测时在真值跟踪的传感器上比较（把估计换算过去），而不是把真值换算到 body。
    /// 数据集的真值本来就在相机系、又要求按相机系比较时为真。
    bool evaluate_at_ground_truth_sensor = false;
};

/// 打开数据集时由任务给出的条件：影响读取方式，但不是读取器自己的参数。
struct DatasetRequest {
    /// 有外部标定文件。读取器不要拿自带的外参去换算真值，按真值原生的坐标系给出，
    /// 评测时再按生效的外参换算——两份外参可能对不上。
    bool external_calibration = false;
    /// 实际生效的打包策略的时间偏置余量。惯性流要留到"末帧终点加上它之后的第一条样本"，
    /// 末帧才等得到右括点。
    fw::Duration post_support{0};
};

/// 数据集读取器：把一种数据格式读成框架的统一数据，来源、标定、真值、推荐的打包策略
/// 一起给出。任务里 `data.source` 那个槽的接口，每种格式一个实现。
///
/// 时刻只用数据自带的：每条数据的时刻取自格式本身（KITTI odometry 的 `times.txt`、
/// raw 的 `timestamps.txt`），不按标称周期或行号编。格式里没有时刻、缺了或与帧数
/// 对不上，就报错。数据里坏掉的时刻默认也报错；要用这份数据，就显式打开实现自己的
/// 修补选项，修补过的帧记进来源声明（`SourceDescriptor::repaired_frames` 与说明）。
class DatasetReaderBase : public logging::Loggable {
public:
    static constexpr std::string_view kInterfaceName = "DatasetReaderBase";

    virtual ~DatasetReaderBase() = default;
    DatasetReaderBase(const DatasetReaderBase&) = delete;
    DatasetReaderBase& operator=(const DatasetReaderBase&) = delete;

    /// 打开数据集。读不了就抛异常并写明路径与原因，不返回半个数据集；
    /// 实现没给出来源是实现的错，抛 `std::logic_error`。
    [[nodiscard]] Dataset open(const DatasetRequest& request) const;

    [[nodiscard]] virtual std::string name() const = 0;

protected:
    DatasetReaderBase() = default;

private:
    [[nodiscard]] virtual Dataset doOpen(const DatasetRequest& request) const = 0;
};

}  // namespace simpleslam::plugin
