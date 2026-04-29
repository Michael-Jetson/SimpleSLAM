#pragma once

/// @file odometry_base.hpp
/// 前端里程计抽象基类——提供回调槽、Topic 发布器、生命周期管理
///
/// 具体实现（LoIcpOdometry、LioIekfOdometry 等）继承此类，
/// 通过 C++20 concept 约束的内部组件获得零开销配准性能。
/// OdometryBase 提供公共基础设施，不含任何算法逻辑。

#include <SimpleSLAM/core/infra/callback_slot.hpp>
#include <SimpleSLAM/core/infra/topic.hpp>
#include <SimpleSLAM/core/infra/topic_hub.hpp>
#include <SimpleSLAM/core/infra/topic_names.hpp>
#include <SimpleSLAM/core/types/event_types.hpp>
#include <SimpleSLAM/core/types/keyframe.hpp>
#include <SimpleSLAM/core/types/odometry_result.hpp>
#include <SimpleSLAM/core/types/sensor_data.hpp>

#include <span>
#include <string>
#include <string_view>

namespace simpleslam {

/// 前端里程计抽象基类——NOT final，具体实现继承此类
class OdometryBase {
public:
    virtual ~OdometryBase() = default;

    /// 生命周期初始化——Runner 组装完成后调用
    /// 基类实现创建 odom/keyframe Publisher，派生类应先调用基类再做自身初始化
    virtual void initialize(TopicHub& hub) {
        odom_pub_ = hub.createPublisherImpl<OdometryResult>(
            std::string(topic_names::kSlamOdometry), QoS::Latest);
        keyframe_pub_ = hub.createPublisherImpl<KeyframeEvent>(
            std::string(topic_names::kSlamKeyframe));
    }

    /// 处理纯 LiDAR 帧（LO 模式必须覆盖）
    virtual OdometryResult processLidar(const LidarScan& scan) = 0;

    /// 处理 LiDAR + IMU（LIO 模式覆盖；默认忽略 IMU 委托 processLidar）
    virtual OdometryResult processLidarImu(const LidarScan& scan,
                                            std::span<const ImuSample> imu) {
        (void)imu;
        return processLidar(scan);
    }

    /// 重置里程计状态（跟踪丢失后重建图）
    virtual void reset() = 0;

    /// 关闭——释放资源
    virtual void shutdown() {}

    /// 实现名称（用于日志和配置匹配）
    [[nodiscard]] virtual std::string_view name() const = 0;

    // ── 同步回调槽——允许外部代码在处理流程中介入 ──

    /// IMU 前向传播完成后（v2.5 GNSS 注入用）
    CallbackSlot<const SE3d&> on_after_prediction;

    /// IEKF/BA 更新完成后（退化分析、配准质量监控）
    CallbackSlot<const OdometryResult&> on_after_update;

    /// 关键帧产出时（触发回环检测等后端服务）
    CallbackSlot<const KeyframeData&> on_keyframe;

    OdometryBase(const OdometryBase&) = delete;
    OdometryBase& operator=(const OdometryBase&) = delete;

protected:
    OdometryBase() = default;

    /// 发布位姿结果——先触发 on_after_update 回调（同步），再发布到 Topic（入队）
    void publishResult(const OdometryResult& result) {
        on_after_update.emit(result);
        if (odom_pub_.valid()) odom_pub_.publish(result);
    }

    /// 发布关键帧——先触发 on_keyframe 回调（同步），再发布 KeyframeEvent 到 Topic
    void publishKeyframe(const KeyframeData& kf) {
        on_keyframe.emit(kf);
        if (keyframe_pub_.valid()) {
            keyframe_pub_.emplace(kf.id, kf.timestamp, kf.pose, kf.scan, kf.image);
        }
    }

    Publisher<OdometryResult> odom_pub_;
    Publisher<KeyframeEvent> keyframe_pub_;
};

}  // namespace simpleslam
