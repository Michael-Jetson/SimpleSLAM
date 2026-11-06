// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file graph_optimizer_module.hpp
/// 图优化模块：位姿图的唯一写者。
///
/// "唯一写者"由写令牌在类型上保证，不靠评审。
///
/// 两条约定：
///
/// 1. **校正事件携带解本身**。发布的是一份不可变的图快照，它自己带身份，
///    消费者不必"先校验身份、再去读最新图"——那两步之间可以插进新的解。
/// 2. **不可用的解不进真源，也不发校正**。"没收敛"与"数值失败"是两回事，
///    前者的解仍可用，后者会把发散的位姿带进图里。

#include <SimpleSLAM/comm/module.hpp>
#include <SimpleSLAM/plugins/graph_optimizer.hpp>
#include <SimpleSLAM/framework/solution.hpp>
#include <SimpleSLAM/modules/topics.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace simpleslam::modules {

/// 里程计边的约定：算法没有给出跨关键帧的相对约束时，兜底的边用这个扰动约定。权重是参数
/// （`GraphOptimizerModule::kTranslationWeight`、`kRotationWeight`）。
struct OdometryEdgePolicy {
    fw::PerturbationConvention convention{};
};

class GraphOptimizerModule final : public comm::Module {
public:
    /// 里程计边的兜底权重：算法没给跨关键帧约束时用，如实标为启发式，不冒充协方差之逆。
    /// 运行中可以改，下一条边就用新值。
    static constexpr comm::Param<double> kTranslationWeight{
        .name = "graph.odometry_translation_weight",
        .fallback = 100.0,
        .doc = "里程计边的兜底平移权重（1/m²，启发式）",
        .min = comm::kPositive,
        .runtime = true};
    static constexpr comm::Param<double> kRotationWeight{
        .name = "graph.odometry_rotation_weight",
        .fallback = 400.0,
        .doc = "里程计边的兜底旋转权重（1/rad²，启发式）",
        .min = comm::kPositive,
        .runtime = true};
    static constexpr std::array kParams{comm::reads(kTranslationWeight),
                                        comm::reads(kRotationWeight)};

    /// 取走位姿图的写令牌：位姿图从此只有本模块能写。
    GraphOptimizerModule(std::unique_ptr<plugin::GraphOptimizerBase> algorithm,
                         SlamResources& resources,
                         OdometryEdgePolicy edge_policy = {});

    [[nodiscard]] plugin::GraphOptimizerBase& algorithm() noexcept { return *algorithm_; }
    [[nodiscard]] std::uint64_t optimizations() const noexcept { return solutions_; }
    [[nodiscard]] std::uint64_t committedConstraints() const noexcept { return committed_; }
    /// 内容不合法或端点不存在、没有进图的约束数。
    [[nodiscard]] std::uint64_t rejectedConstraints() const noexcept { return rejected_; }
    /// 求解失败（优化器异常或解不可用）的次数。
    [[nodiscard]] std::uint64_t failedSolutions() const noexcept { return failed_solutions_; }
    /// 因为图里混入了不同标定的节点而跳过求解的次数。
    [[nodiscard]] std::uint64_t skippedForMixedCalibration() const noexcept {
        return skipped_mixed_;
    }

    /// 后端有没有追平，按**输入版本**判断：图里每一条必需约束（回环、先验）是否都进了
    /// 最后一次成功的求解。返回没进去的条数。
    ///
    /// 只数节点不够：新增一条回环边并不增加节点数。反过来，最后一次求解之后只追加了
    /// 里程计节点也不算没追平——新节点的初值由上一节点的校正后位姿沿里程计延拓，本来
    /// 就是当前校正下的位置，不需要为它们重新优化。边只增不改，所以"进没进最后一次
    /// 求解"就是"下标是否在那次求解的边数之内"。
    [[nodiscard]] std::size_t unsolvedRequiredConstraints() const;

protected:
    void onInit() override;
    /// 主动核对一次补读游标，不等下一条通知；还有必需约束没进成功解的，再求解一次。
    void onFinish() override;

private:
    void onKeyframe(const fw::KeyframeEvent& event);
    /// 按游标从关键帧库补读，追加节点与里程计边。返回是否有新节点。
    bool catchUpKeyframes();
    void appendNode(fw::PoseGraphData& graph, const fw::KeyframePtr& frame,
                    const fw::KeyframePtr& previous_frame);
    /// 算法给的跨关键帧约束能不能直接当边用。
    ///
    /// 两端都要核对：`previous` 指的确实是这条边的起点，起止时刻也要对得上
    /// 实际的两个关键帧。只核 id 的话，一条时刻错位的约束会被当成这两个节点之间的
    /// 测量接进图里。信息矩阵含非有限值、来源未声明，同样不能用——
    /// 这时退回启发式权重，如实标明，而不是把一个来历不明的矩阵塞进图。
    [[nodiscard]] bool usableConstraint(const fw::Keyframe& frame,
                                        const fw::Keyframe& previous) const;
    void onLoop(const fw::LoopConstraint& constraint);
    /// 把一条约束送进真源。内容不合法或端点不存在就拒绝：非有限的测量、不合法的
    /// 信息矩阵会让整张图一起发散；两端是同一个节点的"回环"不约束任何东西。
    bool commitConstraint(const fw::LoopConstraint& constraint);
    /// 优化并发布一份自洽的解快照：身份写进图本身，事件携带图。
    ///
    /// 解不可用（数值失败或问题退化）时不发校正：没有解就没有"新的世界系"，
    /// 发一个带着旧位姿的校正事件只会让下游以为图动过。
    ///
    /// 解的标定身份写的是**这张图自己的**版本，不是当前最新：消费方拿它和自己当前的
    /// 标定比，版本不同就不采纳。图里混进了不同标定的节点时干脆不求解——那样的解
    /// 对应的不是任何一份标定。
    ///
    /// 优化器抛异常、解不可用、约束已提交是三件独立的事，各有各的诊断：
    /// 约束进没进真源，与这里求解成败无关。
    void optimizeAndPublish();
    static const char* statusKey(plugin::GraphOptimizerBase::Status status) noexcept;

    std::unique_ptr<plugin::GraphOptimizerBase> algorithm_;
    SlamResources* resources_;
    fw::WriteToken<fw::PoseGraphData> graph_writer_;
    OdometryEdgePolicy edge_policy_;
    comm::Publisher<fw::CorrectionEvent> correction_pub_;
    fw::KeyframePtr last_frame_;
    std::uint64_t consumed_sequence_ = 0;
    /// 最后一次成功求解时图里的边数。
    std::size_t last_solved_edges_ = 0;
    std::uint64_t solutions_ = 0;
    std::uint64_t committed_ = 0;
    std::uint64_t rejected_ = 0;
    std::uint64_t failed_solutions_ = 0;
    std::uint64_t skipped_mixed_ = 0;
};

}  // namespace simpleslam::modules
