// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file graph_optimizer.hpp
/// 图优化算法接口。
///
/// 这是**插件**（ADR-0004 三分法）：纯算法，不碰总线、不碰资源。
/// 位姿图的唯一写者是图优化**模块**，由写令牌在类型上保证
/// （`framework/resources.hpp`、`modules/graph_optimizer_module.hpp`）。
///
/// 优化器面对的是扁平的求解快照 GraphProblem，不是真源的存储结构：
/// 节点是一个普通 vector（改它就是给解），边只读，id → 下标的索引已经建好。
/// 真源怎么存与求解器面对什么形状分开，因此换存储不必改算法。

#include <SimpleSLAM/framework/pose_graph.hpp>

#include <cstdint>
#include <string>

namespace simpleslam::plugin {

namespace fw = simpleslam::fw;

class GraphOptimizerBase {
public:
    /// 一次求解的结局。"没收敛"与"数值失败"是两件事：前者的解仍然可用
    /// （迭代次数用尽但在往好处走），后者根本不该被采纳。用一个 bool 表达不了。
    enum class Status : std::uint8_t {
        kConverged,     ///< 收敛
        kMaxIterations, ///< 迭代次数用尽，解可用
        kNumericFailure,///< 数值失败（发散、奇异、出现非有限值），解不可用
        kDegenerate,    ///< 问题本身不可解（无边、全自由），解无意义
    };

    struct Report {
        Status status = Status::kConverged;
        std::uint32_t iterations = 0;
        double initial_cost = 0.0;
        double final_cost = 0.0;
        std::size_t nodes = 0;
        std::size_t edges = 0;

        [[nodiscard]] bool converged() const noexcept { return status == Status::kConverged; }
        /// 解能不能采纳。这才是模块要看的那一位。
        [[nodiscard]] bool usable() const noexcept {
            return status == Status::kConverged || status == Status::kMaxIterations;
        }
    };

    virtual ~GraphOptimizerBase() = default;
    GraphOptimizerBase(const GraphOptimizerBase&) = delete;
    GraphOptimizerBase& operator=(const GraphOptimizerBase&) = delete;

    /// 优化一张图：框架生成求解快照、调用算法、回写解。
    /// 调用者已持有唯一写令牌，算法内部不需要加锁。
    ///
    /// 解只在 `usable()` 时回写。数值失败的解不进真源——图宁可停在上一次的位姿，
    /// 也不能带着一批发散的值继续往下走。
    Report optimize(fw::PoseGraphData& graph) {
        fw::GraphProblem problem = graph.toProblem();
        Report report = onOptimize(problem);
        report.nodes = problem.nodeCount();
        report.edges = problem.edgeCount();
        if (report.usable()) graph.adoptSolution(problem);
        return report;
    }

    [[nodiscard]] virtual std::string name() const { return "GraphOptimizerBase"; }

protected:
    GraphOptimizerBase() = default;

    /// 算法本体：改 problem.nodes 里的位姿。不得增删节点。
    virtual Report onOptimize(fw::GraphProblem& problem) = 0;
};

}  // namespace simpleslam::plugin
