#pragma once

/// @file current_state.hpp
/// 当前位姿状态双缓冲——Odometry 高频写 T_odom，PGO 低频写 T_correction
///
/// 最终位姿 = T_correction * T_odom
/// 两者互不阻塞，读者组合两者得到最终位姿。

#include <SimpleSLAM/core/types/common.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>

#include <memory>
#include <mutex>

namespace simpleslam {

class CurrentState {
public:
    /// Odometry 里程计位姿快照
    struct OdomSnapshot {
        Timestamp timestamp{0.0};
        SE3d pose{};            ///< T_world_body（里程计输出）
        Mat6d covariance{};
    };

    // ── 里程计位姿（单写者：Odometry 线程）──

    void publishOdom(Timestamp ts, const SE3d& pose, const Mat6d& covariance = Mat6d::Zero()) {
        auto snap = std::make_shared<const OdomSnapshot>(OdomSnapshot{ts, pose, covariance});
        std::lock_guard lock(odom_mutex_);
        odom_ = std::move(snap);
    }

    [[nodiscard]] std::shared_ptr<const OdomSnapshot> readOdom() const {
        std::lock_guard lock(odom_mutex_);
        return odom_;
    }

    // ── 后端校正（单写者：PGO 线程）──

    void publishCorrection(const SE3d& correction) {
        std::lock_guard lock(correction_mutex_);
        correction_ = std::make_shared<const SE3d>(correction);
    }

    [[nodiscard]] SE3d readCorrection() const {
        std::lock_guard lock(correction_mutex_);
        return correction_ ? *correction_ : SE3d{};
    }

    // ── 最终校正后位姿 ──

    /// T_corrected = T_correction * T_odom
    [[nodiscard]] SE3d correctedPose() const {
        auto odom = readOdom();
        auto corr = readCorrection();
        if (!odom) return corr;
        return corr * odom->pose;
    }

private:
    mutable std::mutex odom_mutex_;
    std::shared_ptr<const OdomSnapshot> odom_;

    mutable std::mutex correction_mutex_;
    std::shared_ptr<const SE3d> correction_;
};

}  // namespace simpleslam
