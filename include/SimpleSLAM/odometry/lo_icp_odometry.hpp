#pragma once

/// @file lo_icp_odometry.hpp
/// 纯 LiDAR 里程计——VoxelHashTarget + IcpSolver 的编排器
///
/// 对标 KISS-ICP Pipeline：降采样 → 恒速预测 → ICP 迭代 → 地图更新 → 发布。
/// 继承 OdometryBase，通过 OfflineRunner 或 ROS2 Node 驱动。

#include <SimpleSLAM/core/math/icp_solver.hpp>
#include <SimpleSLAM/core/math/point_ops.hpp>
#include <SimpleSLAM/odometry/keyframe_selector.hpp>
#include <SimpleSLAM/odometry/odometry_base.hpp>
#include <SimpleSLAM/odometry/voxel_hash_target.hpp>

#include <memory>

namespace simpleslam {

/// 纯 LO 里程计配置
struct LoIcpConfig {
    VoxelHashTargetConfig target;
    IcpConfig solver;
    KeyframeCriteria keyframe;
    float downsample_voxel_size = 1.5f;  ///< 配准用降采样体素大小
};

/// 纯 LiDAR 里程计——ICP scan-to-map
class LoIcpOdometry final : public OdometryBase {
public:
    explicit LoIcpOdometry(const LoIcpConfig& config = {})
        : config_(config)
        , target_(config.target)
        , solver_(config.solver)
        , kf_selector_(config.keyframe) {}

    OdometryResult processLidar(const LidarScan& scan) override {
        // 降采样——配准用稀疏集，地图更新用原始扫描
        auto source = voxelDownsample(scan, config_.downsample_voxel_size);

        // 恒速模型预测初始位姿
        SE3d pose = last_pose_ * delta_;

        // 第一帧：无地图可配准，直接插入
        if (target_.empty()) {
            target_.update(scan, pose);
            last_pose_ = pose;
            return OdometryResult{scan.timestamp, pose, {}, TrackingStatus::Initializing};
        }

        // ICP 迭代：match → solve → 左更新位姿 → 收敛检查
        for (int i = 0; i < solver_.config().max_iterations; ++i) {
            result_.clear();
            target_.match(source, pose, result_);
            if (result_.num_valid < 6) break;

            auto dx = solver_.solveOneStep(result_);
            pose = SE3d::Exp(dx) * pose;

            if (dx.norm() < solver_.config().convergence_threshold) break;
        }

        // 用原始扫描更新地图（不是降采样后的——保证地图稠密度）
        target_.update(scan, pose);
        target_.removeIfFar(pose.translation(), config_.target.max_range * 2.0);

        // 更新恒速模型
        delta_ = last_pose_.inverse() * pose;
        last_pose_ = pose;

        // 关键帧判定
        if (kf_selector_.shouldSelect(pose, scan.timestamp)) {
            kf_selector_.update(pose, scan.timestamp);
            KeyframeData kf;
            kf.id = next_kf_id_++;
            kf.timestamp = scan.timestamp;
            kf.pose = pose;
            kf.scan = std::make_shared<const LidarScan>(scan);
            publishKeyframe(kf);
        }

        // 发布位姿结果
        OdometryResult res;
        res.timestamp = scan.timestamp;
        res.pose = pose;
        res.status = TrackingStatus::Tracking;
        publishResult(res);
        return res;
    }

    void reset() override {
        target_.clear();
        last_pose_ = SE3d{};
        delta_ = SE3d{};
        kf_selector_.reset();
        next_kf_id_ = 0;
    }

    [[nodiscard]] std::string_view name() const override {
        return "LoIcpOdometry";
    }

private:
    LoIcpConfig config_;
    VoxelHashTarget target_;
    IcpSolver solver_;
    KeyframeSelector kf_selector_;

    SE3d last_pose_{};
    SE3d delta_{};
    MatchResult result_;     ///< 复用缓冲区，避免每帧堆分配
    uint64_t next_kf_id_{0};
};

}  // namespace simpleslam
