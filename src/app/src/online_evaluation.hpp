// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file online_evaluation.hpp
/// 在线评测：跑着的时候按数据时间每隔一段，对"到目前为止"的轨迹调一批评测函数，结果记成
/// 度量、终端上每次一行。入口层内部的头文件，不是公共接口。
///
/// 评测函数与跑完之后评的是同一批、同一份参数；输入换成运行中的快照（`EvaluationInputs`
/// 的另一个实现），评测函数不管来源。定时器走运行时的时钟：离线时是数据时间，结果可复现；
/// 在线时在本模块自己的线程上，不占算法的线程。数字是截至此刻的过程量，不进结论。

#include <SimpleSLAM/comm/module.hpp>
#include <SimpleSLAM/eval/evaluator.hpp>
#include <SimpleSLAM/framework/calibration.hpp>
#include <SimpleSLAM/framework/diagnostics.hpp>
#include <SimpleSLAM/framework/odometry_result.hpp>
#include <SimpleSLAM/modules/topics.hpp>
#include <SimpleSLAM/modules/trajectory_module.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace simpleslam::app {

/// 模块名：度量的来源是 `evaluation`。
inline constexpr std::string_view kOnlineEvaluationModule = "/slam/evaluation";

/// 在线评的一个评测函数：名字（指标名的第一段）、类型与实例。
struct OnlineEvaluator {
    std::string name;
    std::string type;
    std::unique_ptr<eval::EvaluatorBase> evaluator;
};

class OnlineEvaluationModule final : public comm::Module {
public:
    struct Setup {
        std::vector<OnlineEvaluator> evaluators;
        comm::Duration interval{};
        /// 换算到比较用的传感器上的真值。数据没有真值时为空，要真值的评测函数就报错。
        std::optional<fw::Trajectory> reference;
        /// 运行开始时生效的标定：估计跟踪 body 而真值跟踪别的传感器时，按它换算。
        fw::CalibrationData calibration;
        const modules::SlamResources* resources = nullptr;
        /// 系统的诊断汇聚点：度量与事件从这里读。
        const fw::MemoryDiagnosticSink* diagnostics = nullptr;
    };

    explicit OnlineEvaluationModule(Setup setup);

protected:
    /// 订阅里程计结果自己攒轨迹（与轨迹模块同一种攒法），排下周期定时器。
    void onInit() override;

private:
    void onOdometry(const fw::OdometryResult& result);
    void evaluateNow();

    Setup setup_;
    modules::TrajectoryRecorder recorder_;
    std::uint64_t last_packet_ = 0;
    fw::Timestamp last_time_{0};
};

}  // namespace simpleslam::app
