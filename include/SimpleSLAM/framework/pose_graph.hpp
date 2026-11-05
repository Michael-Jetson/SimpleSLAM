// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file pose_graph.hpp
/// 位姿图：全局位姿业务状态的真源（不变量 2）。
///
/// 求解器内部的图只是缓存，可从这里重建。位姿图按版本发布不可变快照，
/// 版本号即 epoch：消费者拿 epoch 判断校正是否基于自己认识的那张图。
///
/// 两条存储上的约定：
/// - 节点与边只增不改结构，用 AppendLog 存，避免每加一个关键帧就整份复制索引；
/// - 优化会整体改写节点位姿，所以给求解器的是一个扁平的求解快照（GraphProblem），
///   解回写一次，图重建一次。"求解器面对什么形状"与"真源怎么存"分开。
///
/// 还有一条身份上的约定：这份图自己带 SolutionIdentity。
/// 校正通知携带的是图快照本身（solution.hpp），不是"去读最新图"的指令——
/// 后者在并发下会把 S1 的身份贴到 S2 的数据上。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/append_log.hpp>
#include <SimpleSLAM/framework/events.hpp>
#include <SimpleSLAM/framework/ids.hpp>
#include <SimpleSLAM/framework/resources.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <Eigen/Eigenvalues>

#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace simpleslam::fw {

/// 位姿是否全为有限值。求解发散会产出 NaN/inf，它们必须在进真源之前被挡住。
[[nodiscard]] inline bool isFinite(const SE3d& pose) noexcept {
    return pose.translation().allFinite() && pose.quat().coeffs().allFinite()
           && pose.quat().norm() > 0.0;
}

/// 信息矩阵能不能当图边权重用：有限、对称、半正定。
///
/// 有限不等于合法：一个不对称或有负特征值的矩阵照样全是有限数，进了求解器会把
/// 代价函数变成非凸甚至无下界。退化方向允许零信息，所以只要求半正定，不要求正定。
[[nodiscard]] inline bool isValidInformation(const Mat6d& information) {
    if (!information.allFinite()) return false;
    const double scale = information.cwiseAbs().maxCoeff();
    if (scale == 0.0) return true;  // 全零：没有信息，但不违法
    if ((information - information.transpose()).cwiseAbs().maxCoeff() > 1e-9 * scale) {
        return false;
    }
    const Eigen::SelfAdjointEigenSolver<Mat6d> solver(information);
    return solver.eigenvalues().minCoeff() >= -1e-9 * scale;
}

enum class EdgeKind : std::uint8_t { kOdometry, kLoop, kPrior };

struct PoseGraphNode {
    KeyframeId id{};
    Timestamp time{0};
    SE3d T_world_body = SE3d::Identity();
    bool fixed = false;  ///< gauge 固定点
    /// 建立这个节点的关键帧当时用的标定版本。
    std::uint64_t calibration_revision = 0;
};

struct PoseGraphEdge {
    EdgeKind kind = EdgeKind::kOdometry;
    /// 一次性约束的稳定身份；里程计边可以为空（它由关键帧序列隐含决定）。
    ConstraintId id{};
    KeyframeId from{};
    KeyframeId to{};  ///< kPrior 时无效
    SE3d measurement = SE3d::Identity();
    Mat6d information = Mat6d::Identity();
    /// 信息矩阵的来源与扰动约定。缺了任何一个，这个 6×6 都没有确定含义。
    InformationSource information_source = InformationSource::kHeuristic;
    PerturbationConvention convention{};

    [[nodiscard]] bool heuristicWeight() const noexcept { return isHeuristic(information_source); }
};

/// 求解快照：给求解器的扁平视图，id → 下标已经建好。
///
/// 写优化算法的人只面对这个结构：不需要知道真源怎么存，也不需要自己建索引。
/// 节点位姿是求解器要写的东西；**边是只读的**——类型上就是 const，算法改不了它。
/// 临时改了边而真源不记录，解就对应着一张不存在的图。
struct GraphProblem {
    std::vector<PoseGraphNode> nodes;
    const std::vector<PoseGraphEdge> edges;
    std::unordered_map<KeyframeId, std::size_t> index;

    GraphProblem() = default;
    GraphProblem(std::vector<PoseGraphNode> solved_nodes, std::vector<PoseGraphEdge> fixed_edges)
        : nodes(std::move(solved_nodes)), edges(std::move(fixed_edges)) {
        index.reserve(nodes.size());
        for (std::size_t i = 0; i < nodes.size(); ++i) index.emplace(nodes[i].id, i);
    }

    [[nodiscard]] std::size_t nodeCount() const noexcept { return nodes.size(); }
    [[nodiscard]] std::size_t edgeCount() const noexcept { return edges.size(); }

    [[nodiscard]] PoseGraphNode* node(KeyframeId id) {
        auto it = index.find(id);
        return it == index.end() ? nullptr : &nodes[it->second];
    }
    [[nodiscard]] const PoseGraphNode* node(KeyframeId id) const {
        auto it = index.find(id);
        return it == index.end() ? nullptr : &nodes[it->second];
    }
};

struct PoseGraphData {
    SessionId session{};
    /// 这份图自己的身份。随图一起快照，不由通知另行携带。
    SolutionIdentity identity;
    /// 节点按 id 升序追加，因此可二分查找；边按提交顺序追加。
    AppendLog<PoseGraphNode> nodes;
    AppendLog<PoseGraphEdge> edges;
    /// 已提交的一次性约束身份。用于去重：同一条回环重发不会变成两条边。
    std::set<ConstraintId> committed_constraints;
    /// 图覆盖到的关键帧序号，用于"稍旧但兼容"的判断。
    std::uint64_t covered_until = 0;
    /// 这张图建立时用的标定版本（取自第一个节点）。
    ///
    /// **一张图只属于一份标定**。节点与边是在各自那一刻的外参下算出来的；标定换代
    /// 之后再往里加节点，图就成了两份标定的混合——取一个当前快照、给解重贴当前版本，
    /// 并不能把旧约束升级成新约束。所以混入不同版本的节点时置 calibration_mixed，
    /// 此后这张图不再产出校正，直到有按新标定重建受影响约束的策略。
    std::uint64_t calibration_revision = 0;
    bool calibration_mixed = false;

    /// 追加一个节点。必须按 id 升序，否则二分查找与确定性遍历都不成立。
    void addNode(const PoseGraphNode& node) {
        if (nodes.size() != 0 && !(nodes.back().id < node.id)) {
            throw std::logic_error("位姿图节点必须按 id 升序追加");
        }
        nodes.push_back(node);
    }

    [[nodiscard]] const PoseGraphNode* node(KeyframeId id) const {
        std::size_t lo = 0;
        std::size_t hi = nodes.size();
        while (lo < hi) {
            const std::size_t mid = lo + (hi - lo) / 2;
            const auto& candidate = nodes.at(mid);
            if (candidate.id == id) return &candidate;
            if (candidate.id < id) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        return nullptr;
    }

    [[nodiscard]] bool hasConstraint(ConstraintId id) const {
        return id.valid() && committed_constraints.count(id) != 0;
    }

    [[nodiscard]] std::size_t nodeCount() const noexcept { return nodes.size(); }
    [[nodiscard]] std::size_t edgeCount() const noexcept { return edges.size(); }

    /// 生成求解快照。O(n)，每次优化一次，不在每加一个关键帧时发生。
    [[nodiscard]] GraphProblem toProblem() const {
        return GraphProblem(nodes.flatten(), edges.flatten());
    }

    /// 回写解。
    ///
    /// 接受的条件是"同一批节点的新位姿"：数量、id 集合与顺序都要对得上，
    /// 且每个位姿都是有限值。只数个数是不够的——换了 id、乱了序、或者求解发散出
    /// NaN，都会得到一张看似合法、实则不是这张图的解的图。
    ///
    /// **只采纳位姿**：节点的其余字段（时刻、是否固定、标定版本）是图的事实，
    /// 不是解的一部分，求解器改了也不算数。固定节点的位姿本身也不许动——
    /// 那是这张图的 gauge，动了它整张图就换了一个参考系。
    void adoptSolution(const GraphProblem& problem) {
        if (problem.nodes.size() != nodes.size()) {
            throw std::logic_error("优化器改变了节点数量：解与图不对应");
        }
        std::vector<PoseGraphNode> adopted = nodes.flatten();
        for (std::size_t i = 0; i < problem.nodes.size(); ++i) {
            const auto& solved = problem.nodes[i];
            auto& node = adopted[i];
            if (!(solved.id == node.id)) {
                throw std::logic_error("优化器改变了节点身份或顺序：解与图不对应");
            }
            if (!isFinite(solved.T_world_body)) {
                throw std::logic_error("解中含非有限位姿：数值失败，不得采纳");
            }
            if (node.fixed) {
                const SE3d delta = node.T_world_body.inverse().compose(solved.T_world_body);
                if (delta.translation().norm() > 1e-9
                    || std::abs(delta.quat().w()) < 1.0 - 1e-12) {
                    throw std::logic_error("优化器移动了固定节点：解换了 gauge，不得采纳");
                }
                continue;
            }
            node.T_world_body = solved.T_world_body;
        }
        nodes = AppendLog<PoseGraphNode>::fromVector(std::move(adopted));
    }
};

using PoseGraph = Resource<PoseGraphData>;

/// 当前状态：高频的里程计位姿与低频的世界校正分开写，读者自行组合。
///
/// T_world_body = T_world_odom * T_odom_body
struct CurrentStateData {
    Timestamp time{0};
    SE3d T_odom_body = SE3d::Identity();
    SE3d T_world_odom = SE3d::Identity();
    TrackingStatus status = TrackingStatus::kInitializing;
    SolutionIdentity correction_identity;

    [[nodiscard]] SE3d T_world_body() const { return T_world_odom.compose(T_odom_body); }
};

using CurrentState = Resource<CurrentStateData>;

}  // namespace simpleslam::fw
