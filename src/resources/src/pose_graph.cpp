#include <SimpleSLAM/resources/pose_graph.hpp>

#include <mutex>

namespace simpleslam {

void PoseGraph::addNode(PoseGraphNode node) {
    std::unique_lock lock(mutex_);
    nodes_[node.id] = std::move(node);
}

void PoseGraph::addEdge(PoseGraphEdge edge) {
    std::unique_lock lock(mutex_);
    edges_.push_back(std::move(edge));
}

void PoseGraph::updatePoses(const std::unordered_map<uint64_t, SE3d>& optimized) {
    std::unique_lock lock(mutex_);
    for (const auto& [id, pose] : optimized) {
        auto it = nodes_.find(id);
        if (it != nodes_.end()) {
            it->second.pose = pose;
        }
    }
}

std::optional<PoseGraphNode> PoseGraph::getNode(uint64_t id) const {
    std::shared_lock lock(mutex_);
    auto it = nodes_.find(id);
    if (it == nodes_.end()) return std::nullopt;
    return it->second;
}

std::vector<PoseGraphNode> PoseGraph::allNodes() const {
    std::shared_lock lock(mutex_);
    std::vector<PoseGraphNode> result;
    result.reserve(nodes_.size());
    for (const auto& [_, node] : nodes_) {
        result.push_back(node);
    }
    return result;
}

std::vector<PoseGraphEdge> PoseGraph::allEdges() const {
    std::shared_lock lock(mutex_);
    return edges_;
}

std::vector<PoseGraphEdge> PoseGraph::edgesOfType(EdgeType type) const {
    std::shared_lock lock(mutex_);
    std::vector<PoseGraphEdge> result;
    for (const auto& e : edges_) {
        if (e.type == type) result.push_back(e);
    }
    return result;
}

size_t PoseGraph::nodeCount() const {
    std::shared_lock lock(mutex_);
    return nodes_.size();
}

size_t PoseGraph::edgeCount() const {
    std::shared_lock lock(mutex_);
    return edges_.size();
}

bool PoseGraph::empty() const {
    std::shared_lock lock(mutex_);
    return nodes_.empty();
}

PoseGraph::Snapshot PoseGraph::snapshot() const {
    std::shared_lock lock(mutex_);
    Snapshot snap;
    snap.nodes.reserve(nodes_.size());
    for (const auto& [_, node] : nodes_) {
        snap.nodes.push_back(node);
    }
    snap.edges = edges_;
    return snap;
}

}  // namespace simpleslam
