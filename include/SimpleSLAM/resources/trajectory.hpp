#pragma once

/// @file trajectory.hpp
/// 位姿轨迹记录——追加、查询、TUM/KITTI 格式导出
///
/// 用于评测和可视化。支持按时间戳线性插值查询。

#include <SimpleSLAM/core/types/common.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>

#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace simpleslam {

class Trajectory {
public:
    struct PoseEntry {
        Timestamp timestamp;
        SE3d pose;
    };

    /// 追加一个位姿（时间戳应单调递增）
    void append(Timestamp ts, const SE3d& pose);

    [[nodiscard]] size_t size() const;
    [[nodiscard]] bool empty() const;

    /// 获取所有位姿条目
    [[nodiscard]] std::vector<PoseEntry> entries() const;

    /// 按时间戳查询位姿（两个最近关键帧间线性插值）
    [[nodiscard]] std::optional<SE3d> poseAt(Timestamp ts) const;

    /// 导出 TUM 格式：timestamp tx ty tz qx qy qz qw
    void exportTUM(const std::string& path) const;

    /// 导出 KITTI 格式：每行 12 个值（3×4 变换矩阵，行优先）
    void exportKITTI(const std::string& path) const;

    void clear();

private:
    mutable std::mutex mutex_;
    std::vector<PoseEntry> poses_;
};

}  // namespace simpleslam
