#pragma once

/// @file pose_graph.hpp
/// 位姿图数据结构——关键帧节点 + 边（里程计/回环/先验约束）
///
/// 并发模型：shared_mutex 保护
///   写者（Odometry 追加、LoopDetector 追加、PGO 批量更新）持锁 < 1ms
///   读者（可视化、评测）持 shared_lock

#include <SimpleSLAM/core/types/common.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>

#include <cstdint>
#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

namespace simpleslam {

/// 位姿图节点
struct PoseGraphNode {
    uint64_t id;
    Timestamp timestamp;
    SE3d pose;   ///< T_world_body（当前最优估计）
};

/// 边的类型
enum class EdgeType : uint8_t {
    Odometry,   ///< 相邻帧间的里程计约束
    Loop,       ///< 回环闭合约束
    Prior,      ///< 绝对先验（GNSS 等）
};

/// 位姿图边
struct PoseGraphEdge {
    uint64_t from_id;
    uint64_t to_id;
    SE3d T_from_to;         ///< 相对变换测量
    Mat6d information;       ///< 信息矩阵（协方差逆）
    EdgeType type;
};

/// 线程安全的位姿图
class PoseGraph {
public:
    // ── 写操作（unique_lock，持锁 < 1ms）──

    void addNode(PoseGraphNode node);
    void addEdge(PoseGraphEdge edge);

    /// PGO 优化完成后批量更新节点位姿
    void updatePoses(const std::unordered_map<uint64_t, SE3d>& optimized);

    // ── 读操作（shared_lock）──

    [[nodiscard]] std::optional<PoseGraphNode> getNode(uint64_t id) const;
    [[nodiscard]] std::vector<PoseGraphNode> allNodes() const;
    [[nodiscard]] std::vector<PoseGraphEdge> allEdges() const;
    [[nodiscard]] std::vector<PoseGraphEdge> edgesOfType(EdgeType type) const;

    [[nodiscard]] size_t nodeCount() const;
    [[nodiscard]] size_t edgeCount() const;
    [[nodiscard]] bool empty() const;

    /// 深拷贝快照（可视化用，拷贝后释放锁，不阻塞前端）
    struct Snapshot {
        std::vector<PoseGraphNode> nodes;
        std::vector<PoseGraphEdge> edges;
    };
    [[nodiscard]] Snapshot snapshot() const;

private:
    mutable std::shared_mutex mutex_;
    std::unordered_map<uint64_t, PoseGraphNode> nodes_;
    std::vector<PoseGraphEdge> edges_;
};

}  // namespace simpleslam
