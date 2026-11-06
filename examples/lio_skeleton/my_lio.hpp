// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file my_lio.hpp
/// 手写 LIO 的骨架。把 `track()` 里的内容换掉就是你的算法。
///
/// 这个文件里没有一行是"框架要求你写的样板"——订阅、真源、校正回流、诊断、
/// 评测、录制都在外面。要写日志就调成员 `log_`：`log_.info(…)`、`log_.warn(…)`。
/// 你要做的只有三件事：
///
///   1. 用包里的 IMU 区间做预积分 / 状态传播；
///   2. 用传播出来的帧内轨迹把点云去畸变（`fw::deskew` 已经把外参进出各一次的
///      那两步做好了，见 framework/deskew.hpp）；
///   3. 把去畸变后的点与地图配准，得到本帧位姿。
///
/// 还有两件可选但强烈建议做的：
///   - 填 `registration_information` 并声明它的来源（传播 / 启发式）；
///   - 关键帧时填 `constraint_to_last_keyframe`。不填的话图边会用启发式权重，
///     框架会如实标注，但精度会差——那个 6×6 只有你知道怎么算。

#include <SimpleSLAM/framework/calibration.hpp>
#include <SimpleSLAM/framework/deskew.hpp>
#include <SimpleSLAM/plugins/odometry.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>

namespace example {

namespace fw = simpleslam::fw;
namespace plugin = simpleslam::plugin;
using simpleslam::SE3d;
using simpleslam::Vec3d;
using simpleslam::Mat6d;

/// 占位实现：IMU 只用来估帧内角速度与比力，位姿靠匀速外推。
/// 它跑得通、能出轨迹、能被评测——但它不是 LIO，因为没有配准。
/// 换掉 `track()` 即可；其余部分（关键帧判定、相对约束）可以留着用。
class MyLio final : public plugin::OdometryBase {
public:
    /// 任务 YAML 里写 `type: my_lio` 选它；入口把它追加进里程计方案的目录。
    static constexpr std::string_view kTypeName = "my_lio";

    /// 算法自己的参数：字段清单写一次，YAML 的严格校验、默认值补齐、schema 都从它来。
    struct Config {
        double keyframe_distance_m = 1.0;
        double keyframe_rotation_deg = 15.0;

        template <class V>
        void visit(V& v) {
            v("keyframe_distance_m", keyframe_distance_m, "关键帧的平移阈值（米）");
            v("keyframe_rotation_deg", keyframe_rotation_deg, "关键帧的旋转阈值（度）");
        }
    };

    /// 阈值为负时抛 `std::invalid_argument`：装配时就报，不等到跑起来。
    explicit MyLio(const Config& config) : config_(config) {
        if (!(config_.keyframe_distance_m >= 0.0) || !(config_.keyframe_rotation_deg >= 0.0)) {
            throw std::invalid_argument("my_lio 的关键帧阈值不能为负");
        }
    }

    [[nodiscard]] std::string name() const override { return "MyLio"; }

    /// 缺支撑的包怎么办，必须自己选。LIO 的初始化要一帧支撑完整的数据：初始化之前
    /// 缺支撑就等，初始化之后照常跟踪但结果降级（框架会把 kTracking 压成 kDegraded）。
    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kWaitForInitialization;
    }

protected:
    plugin::TrackResult track(const fw::SensorPacket& packet) override {
        plugin::TrackResult result;
        result.status = fw::TrackingStatus::kTracking;

        // ── 1. 取本帧的 IMU 区间 ───────────────────────────────────────────
        // 打包器已经保证：传播从"上次已接纳的时刻"起算（packet.propagation），支撑需求
        // 还覆盖整帧扫描（packet.imu_required），实际支撑两端各有一个跨越端点的样本可供
        // 插值（packet.imu_coverage），盖不住需求就在 packet.gap 里标出。
        // 缺支撑的包不会以 kTracking 结束：supportGapPolicy() 选了什么，框架就执行什么。
        const auto imu = packet.imu.view();
        const double dt = frameInterval(packet);

        Vec3d angular_velocity = Vec3d::Zero();
        for (const auto& sample : imu) {
            angular_velocity += Vec3d(sample.gyro[0], sample.gyro[1], sample.gyro[2]);
        }
        if (!imu.empty()) angular_velocity /= static_cast<double>(imu.size());
        // 写日志用成员 log_：来源（/slam/odometry › my_lio）、帧序号与时刻是自动的。
        // debug 默认只进任务记录里的 run.log，终端上不显示
        log_.debug("{} 个 IMU 样本，帧间隔 {:.3f} s", imu.size(), dt);

        // TODO(你): 这里换成真正的 IMU 预积分 / ESIKF 预测。
        SE3d::Tangent twist;
        twist.coeffs() << velocity_.x(), velocity_.y(), velocity_.z(), angular_velocity.x(),
            angular_velocity.y(), angular_velocity.z();

        // ── 2. 去畸变 ─────────────────────────────────────────────────────
        // 外参进出各一次这两步由框架做；帧内轨迹由你给。
        // 这里给的是"匀速"，真实实现应当换成预积分出来的轨迹。
        //
        // 外参从本帧上下文取，不自己去读资源：一帧之内只能有一份标定，
        // 它的版本已经随产物发布出去了。
        if (const auto* scan = packet.lidar(); scan != nullptr) {
            if (const auto T_body_lidar = extrinsic(scan->sensor)) {
                fw::DeskewContext deskew_context;
                deskew_context.reference_time = packet.reference_time;
                deskew_context.calibration_revision = context().calibrationRevision();
                deskew_context.sensor = scan->sensor;
                deskew_context.motion_compensation = scan->motion_compensation;
                // 轨迹覆盖如实给：角速度取自本包的 IMU，只在它实际支撑的区间内可信。
                // 匀速模型对任意偏移都算得出数，但那不等于有数据支撑；没有 IMU 时
                // 只有参考时刻本身算覆盖，扫描里的其余点会让这一帧报"覆盖不到"
                deskew_context.trajectory_coverage =
                    imu.empty() ? fw::TimeRange{fw::Timestamp{0}, fw::Timestamp{0}}
                                : fw::TimeRange{
                                      fw::Timestamp{packet.imu_coverage.begin - packet.reference_time},
                                      fw::Timestamp{packet.imu_coverage.end - packet.reference_time}};
                auto [corrected, deskewed] = fw::deskew(scan->points, *T_body_lidar,
                                                        fw::ConstantTwist(twist), deskew_context);
                // 补没补是有明确答案的：数据集已经补偿过的扫描原样保留，那不是降级；
                // 缺逐点时间、时间列损坏才是"这一帧没能补偿"
                if (!deskewed.compensated()) result.status = fw::TrackingStatus::kDegraded;
                // TODO(你): 把 corrected 降采样后与地图配准，得到 T_odom_body。
                (void)corrected;
            }
        }

        // ── 3. 位姿 ───────────────────────────────────────────────────────
        SE3d::Tangent step = twist;
        step.coeffs() *= dt;
        pose_ = pose_.rplus(step);
        result.T_odom_body = pose_;

        // 信息矩阵与它的来源。声明成启发式就是启发式，框架不会替你改判。
        result.registration_information = Mat6d::Identity() * 10.0;
        result.registration_information_source = fw::InformationSource::kHeuristic;

        // 跨关键帧的相对约束：只有算法知道它跨了几帧、用没用局部地图。
        if (hasKeyframe()) {
            fw::RelativeConstraint constraint;
            constraint.from_time = lastKeyframeTime();
            constraint.to_time = packet.reference_time;
            constraint.measurement = lastKeyframePose().inverse().compose(pose_);
            constraint.information = Mat6d::Identity() * 10.0;
            constraint.source = fw::InformationSource::kHeuristic;
            result.constraint_to_last_keyframe = constraint;
        }

        result.inliers = static_cast<std::uint32_t>(packet.lidar() != nullptr
                                                        ? packet.lidar()->points.size()
                                                        : 0);
        result.iterations = 1;
        last_time_ = packet.reference_time;
        had_frame_ = true;
        return result;
    }

    /// 关键帧判定：走过一定距离或转过一定角度。框架已经把上一关键帧的位姿留好了。
    /// 阈值来自配置（插件参数第一版不能在运行期调）。
    bool selectKeyframe(const plugin::TrackResult& tracked,
                        const fw::SensorPacket& packet) override {
        (void)packet;
        if (!hasKeyframe()) return true;  // 第一帧必须是关键帧，否则图没有起点
        const double distance_m = config_.keyframe_distance_m;
        const double rotation_rad = config_.keyframe_rotation_deg * kPi / 180.0;
        const SE3d relative = lastKeyframePose().inverse().compose(tracked.T_odom_body);
        const double translation = relative.translation().norm();
        const double rotation = 2.0 * std::acos(std::min(1.0, std::abs(relative.quat().w())));
        return translation > distance_m || rotation > rotation_rad;
    }

    /// 后端给了新解。需要按校正重建局部地图的实现在这里做，框架保证是在帧边界。
    void onCorrection(const SE3d& T_world_odom, const fw::PoseGraphData& graph) override {
        (void)T_world_odom;
        (void)graph;
        // TODO(你): 若局部地图存在世界系里，在这里按新解重建。
    }

    void onReset() override {
        pose_ = SE3d::Identity();
        velocity_ = Vec3d::Zero();
        had_frame_ = false;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    [[nodiscard]] double frameInterval(const fw::SensorPacket& packet) const {
        if (!had_frame_) return 0.0;
        const double dt = static_cast<double>((packet.reference_time - last_time_).count()) * 1e-9;
        return dt > 0.0 && dt < 1.0 ? dt : 0.0;  // 缺口或时钟跳变时不外推
    }

    Config config_;
    SE3d pose_ = SE3d::Identity();
    Vec3d velocity_ = Vec3d::Zero();
    fw::Timestamp last_time_{0};
    bool had_frame_ = false;
};

}  // namespace example
