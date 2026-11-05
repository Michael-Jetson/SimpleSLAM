// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/framework/pose_graph.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "matrix_checks.hpp"

namespace simpleslam::fw {

bool isValidInformation(const Mat6d& information) {
    return information.allFinite()
           && detail::definiteness(information) == detail::Definiteness::kSymmetricPsd;
}

void PoseGraphData::addNode(const PoseGraphNode& node) {
    if (nodes.size() != 0 && !(nodes.back().id < node.id)) {
        throw std::logic_error("位姿图节点必须按 id 升序追加");
    }
    nodes.push_back(node);
}

const PoseGraphNode* PoseGraphData::node(KeyframeId id) const {
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

GraphProblem PoseGraphData::toProblem() const {
    return GraphProblem(nodes.flatten(), edges.flatten());
}

void PoseGraphData::adoptSolution(const GraphProblem& problem) {
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
            if (delta.translation().norm() > 1e-9 || std::abs(delta.quat().w()) < 1.0 - 1e-12) {
                throw std::logic_error("优化器移动了固定节点：解换了 gauge，不得采纳");
            }
            continue;
        }
        node.T_world_body = solved.T_world_body;
    }
    nodes = AppendLog<PoseGraphNode>::fromVector(std::move(adopted));
}

}  // namespace simpleslam::fw
