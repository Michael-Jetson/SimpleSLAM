// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file evaluator.hpp
/// 评测函数：只读输入、只返回结果的插件。
///
/// 任务里列一组带名字的评测函数（`evaluation.evaluators`），没列就不评测。评测函数与算法插件
/// 是同一套机制：类型名、配置聚合体与字段清单、在入口的目录里登记，用户写的和内置的一样选。
///
/// 评测函数**只读**输入（`IEvaluationInputs`），跑完立刻评与事后对任务记录补评读的是同一批
/// 文件，数字一样。它**只返回值**（`Evaluation`），写文件、格式与版本号由框架定一次。

#include <SimpleSLAM/framework/trajectory.hpp>
#include <SimpleSLAM/logging/logger.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace simpleslam::eval {

namespace fw = simpleslam::fw;

/// 一个标量结果。名字是 `<量>.<统计>`，结果里的全名是 `<评测函数的名字>.<量>.<统计>`。
struct Scalar {
    std::string name;
    double value = 0.0;
    std::string unit;   ///< 空表示没有单位
    std::string label;  ///< 进汇总表的短名，如 ATE_t；空表示不进
    /// 多序列汇总时的权重：每个序列都给了，平均就按它加权（`kitti` 给段数，与官方把各序列的
    /// 段合在一起平均一致）；空表示按序列平均。
    std::optional<double> weight;
};

/// 表格的一列。
struct Column {
    std::string name;
    std::string unit;
};

/// 表格里的一格：时刻这类必须精确的量用整数，其余用实数；非有限的实数写出时留空。
using Cell = std::variant<std::int64_t, double>;

/// 一张逐样本的表，写成 `evaluation/<评测函数的名字>.<表名>.csv`。
struct Table {
    std::string name;
    std::vector<Column> columns;
    std::vector<std::vector<Cell>> rows;
};

/// 一个评测函数的结果。
struct Evaluation {
    /// 有效样本数。为零表示这次没测出东西：结论里写无结果，没有标量。
    std::size_t samples = 0;
    std::vector<Scalar> scalars;
    std::vector<Table> tables;
    /// 说明：实际用的对齐方式、换算过的坐标系等，读结论的人要知道的前提。
    std::vector<std::string> notes;

    void add(std::string name, double value, std::string unit, std::string label = {},
             std::optional<double> weight = std::nullopt) {
        scalars.push_back(Scalar{.name = std::move(name),
                                 .value = value,
                                 .unit = std::move(unit),
                                 .label = std::move(label),
                                 .weight = weight});
    }
};

/// 评测函数要哪些输入。执行器在运行之前核对，缺了就报错，不等跑完才发现。
struct Needs {
    bool groundtruth = false;
};

/// 一对可比的轨迹：跟踪同一个传感器，可以直接关联。
struct ComparablePair {
    fw::Trajectory estimate;
    fw::Trajectory reference;
    /// 为了可比做过的换算，进结果的说明。
    std::vector<std::string> notes;
};

/// 模块与插件用 `log_.metric` 记下的一个值。
struct MetricSample {
    /// 它所关于的那条输入：记下它的模块自己的输入序号与数据时刻。不针对某条输入时为空。
    std::optional<std::uint64_t> frame;
    std::optional<fw::Timestamp> data_time;
    std::string source;  ///< 来源的键，如 `odometry.plugin`
    double value = 0.0;
    std::string unit;
};

/// 模块与插件用 `log_.event` 记下的一个事实。
struct EventSample {
    std::optional<std::uint64_t> frame;
    std::optional<fw::Timestamp> data_time;
    std::string module;  ///< 记下它的模块
    std::string source;  ///< 来源的键；不经日志对象记下的为空
    std::string detail;
};

/// 评测函数看到的输入。跑完之后由任务记录提供；在线评测时由运行中的系统提供快照。
///
/// `pair` 与 `targetWindow` 每种输入都有。度量、事件与单独一条估计是可选的能力：
/// 默认实现抛 `std::runtime_error`，说明这份输入没有它。
class IEvaluationInputs {
public:
    virtual ~IEvaluationInputs() = default;

    /// 一条估计（`odometry`、`online`、`keyframes`）与真值，换算到同一个跟踪的传感器。
    /// 没有真值、没有这条估计时抛 `std::runtime_error`。
    [[nodiscard]] virtual ComparablePair pair(std::string_view estimate) const = 0;

    /// 来源声明的目标窗口：这次运行应当产出位姿的时间范围。没声明时为空。
    [[nodiscard]] virtual std::optional<fw::TimeRange> targetWindow() const = 0;

    /// 一条估计本身，不要真值：只看轨迹自己的评测函数用。没有这条估计时抛。
    [[nodiscard]] virtual fw::Trajectory estimate(std::string_view name) const;

    /// 键为 `key` 的全部度量，按记下的先后排列；一条都没有时为空。
    [[nodiscard]] virtual std::vector<MetricSample> metrics(std::string_view key) const;

    /// 键为 `key` 的全部事件，按记下的先后排列；一条都没有时为空。
    [[nodiscard]] virtual std::vector<EventSample> events(std::string_view key) const;
};

class EvaluatorBase : public logging::Loggable {
public:
    static constexpr std::string_view kInterfaceName = "EvaluatorBase";

    virtual ~EvaluatorBase() = default;
    EvaluatorBase(const EvaluatorBase&) = delete;
    EvaluatorBase& operator=(const EvaluatorBase&) = delete;

    [[nodiscard]] virtual Needs needs() const = 0;

    /// 只读输入，只返回值。没有结果的量不给（不写 NaN）：非有限的标量在结论里写 null。
    [[nodiscard]] virtual Evaluation evaluate(const IEvaluationInputs& inputs) const = 0;

protected:
    EvaluatorBase() = default;
};

}  // namespace simpleslam::eval
