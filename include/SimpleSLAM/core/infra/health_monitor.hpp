#pragma once

/// @file health_monitor.hpp
/// 系统级健康状态机——综合前端/后端指标做出系统级判断
///
/// 与 OdometryResult::TrackingStatus（帧级前端状态）是不同层级：
///   - TrackingStatus: Initializing | Tracking | Degraded | Lost  （每帧输出）
///   - SystemHealth:   OK | Degraded | Lost | Failed              （系统全局）
///
/// 状态转移基于连续帧计数 + 可配置阈值。
/// Failed 是终态，需要显式 reset() 恢复。
/// 恢复是阶梯式：Lost → Degraded → OK（不直接跳级）。

#include <SimpleSLAM/core/types/odometry_result.hpp>

#include <cstdint>

namespace simpleslam {

/// 系统级健康状态
enum class SystemHealth : uint8_t {
    OK,        ///< 正常运行
    Degraded,  ///< 部分退化（如配准质量下降、协方差偏大）
    Lost,      ///< 跟踪丢失（尝试恢复中）
    Failed,    ///< 不可恢复故障（终态，需 reset）
};

/// 每帧健康指标——由 Odometry 输出或 Runner 收集
struct HealthMetrics {
    TrackingStatus tracking_status{TrackingStatus::Initializing};
    int valid_match_count{0};
    double covariance_trace{0.0};
    double frame_time_ms{0.0};
};

/// 健康判定阈值
struct HealthThresholds {
    int degraded_frames_threshold = 5;     ///< 连续坏帧数达到此值 → Degraded
    int lost_frames_threshold = 20;        ///< 连续坏帧数达到此值 → Lost
    double max_covariance_trace = 100.0;   ///< 协方差迹超此值视为坏帧
    int recovery_frames = 3;               ///< 连续好帧数达到此值 → 降级恢复
};

/// 健康监控器——状态机，每帧调用 update()
class HealthMonitor final {
public:
    explicit HealthMonitor(const HealthThresholds& thresholds = {})
        : thresholds_(thresholds) {}

    /// 输入一帧指标，更新状态机
    void update(const HealthMetrics& metrics) {
        if (state_ == SystemHealth::Failed) return;

        bool is_good_frame =
            metrics.tracking_status == TrackingStatus::Tracking &&
            metrics.covariance_trace <= thresholds_.max_covariance_trace;

        if (is_good_frame) {
            ++consecutive_good_frames_;
            consecutive_bad_frames_ = 0;
        } else {
            ++consecutive_bad_frames_;
            consecutive_good_frames_ = 0;
        }

        switch (state_) {
            case SystemHealth::OK:
                if (consecutive_bad_frames_ >= thresholds_.degraded_frames_threshold) {
                    state_ = SystemHealth::Degraded;
                }
                break;

            case SystemHealth::Degraded:
                if (consecutive_bad_frames_ >= thresholds_.lost_frames_threshold) {
                    state_ = SystemHealth::Lost;
                } else if (consecutive_good_frames_ >= thresholds_.recovery_frames) {
                    state_ = SystemHealth::OK;
                    consecutive_good_frames_ = 0;
                }
                break;

            case SystemHealth::Lost:
                if (metrics.tracking_status == TrackingStatus::Lost &&
                    consecutive_bad_frames_ >= thresholds_.lost_frames_threshold * 2) {
                    state_ = SystemHealth::Failed;
                } else if (consecutive_good_frames_ >= thresholds_.recovery_frames) {
                    state_ = SystemHealth::Degraded;
                    consecutive_good_frames_ = 0;
                }
                break;

            case SystemHealth::Failed:
                break;
        }
    }

    [[nodiscard]] SystemHealth state() const { return state_; }

    /// 重置到 OK 状态（从 Failed 恢复的唯一途径）
    void reset() {
        state_ = SystemHealth::OK;
        consecutive_bad_frames_ = 0;
        consecutive_good_frames_ = 0;
    }

    [[nodiscard]] const HealthThresholds& thresholds() const { return thresholds_; }
    [[nodiscard]] int consecutiveBadFrames() const { return consecutive_bad_frames_; }
    [[nodiscard]] int consecutiveGoodFrames() const { return consecutive_good_frames_; }

private:
    HealthThresholds thresholds_;
    SystemHealth state_{SystemHealth::OK};
    int consecutive_bad_frames_{0};
    int consecutive_good_frames_{0};
};

}  // namespace simpleslam
