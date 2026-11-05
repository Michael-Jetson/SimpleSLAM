// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file deskew.hpp
/// 逐点去畸变：把每个点从它自己的采样时刻搬到参考时刻。
///
/// 这是 LIO 里最容易写错的几行，错了也不会崩——只会让精度悄悄差一截，
/// 所以放在框架里写一次，而不是每个算法各写一遍。
///
/// 四处容易错的地方，这里都摊开：
///
/// 1. **外参要进出各一次**。点在传感器系，运动在 body 系，所以是
///    `p_ref_sensor = T_body_sensor⁻¹ ∘ T_ref_body(t) ∘ T_body_sensor ∘ p_sensor`。
///    少一次就等于把外参当成了单位阵。
/// 2. **时间基准是逐点偏移，不是帧时间**。`time_offset_ns` 是相对包的参考时刻的
///    偏移（通常为负，因为参考时刻取扫描终点）。
/// 3. **参考时刻处的点不该被动**。`T_ref_body(0)` 必须是单位阵，
///    调用方给的轨迹函数要满足这一点，否则整帧会被平移一个常量。
/// 4. **补没补要有明确答案**。缺时间与时间列损坏都会让点云原样返回，
///    静默返回等于让调用方以为补过了。结果因此带状态、带参考时刻、带标定版本。
///    还要先问清楚数据集有没有补偿过（`SourceDescriptor::motion_compensation`）：
///    对补偿过的扫描再补一次，等于把同一段自车运动扣了两遍。
///
/// 轨迹函数由算法提供：匀速假设、IMU 预积分、或上一帧的速度外推都行——
/// 框架不替算法选，只保证几何变换本身是对的。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/sensor_data.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace simpleslam::fw {

/// 一次去畸变的结局。
///
/// "变换了几个点"说不清为什么是零：没有逐点时间、时间列长度对不上、点云本来就空，
/// 三种情况都返回零，而处置完全不同。变量名叫 corrected 也证明不了这一帧补偿过。
enum class DeskewStatus : std::uint8_t {
    kCorrected,            ///< 完成
    kAlreadyCompensated,   ///< 数据集已经补偿过：原样保留，不再补一次
    kNoPointTimes,         ///< 没有逐点时间，无从补偿
    kMalformedTimes,       ///< 有时间列但长度与点数不符：数据坏了，不是"没有时间"
    kTrajectoryNotCovered, ///< 有点的采样时刻落在轨迹覆盖区间之外：外推不是补偿
    kEmpty,                ///< 点云为空
};

/// 一次去畸变的前提：变换到哪一刻、用的哪版外参、这帧补没补过、轨迹覆盖多长。
///
/// 这些都由调用方从本帧的包与 FrameContext 里取好传进来，结果原样带回——
/// 结果里的时刻与版本因此是"实际用了什么"，不是事后补填的标签。
struct DeskewContext {
    Timestamp reference_time{0};
    std::uint64_t calibration_revision = 0;
    std::string sensor;  ///< 输出点云所在的传感器系（参考时刻的那一个）
    MotionCompensation motion_compensation = MotionCompensation::kUnknown;
    /// 轨迹函数**有数据支撑**的偏移区间（相对参考时刻，闭区间）。
    ///
    /// 给了就逐点核对，覆盖不到的点让整帧返回 kTrajectoryNotCovered；退化区间只盖住
    /// 它那一刻——没有惯性支撑时就是这样，不能因为区间退化就跳过核对。不给表示调用方
    /// 显式担保覆盖。模型对任意偏移都算得出数，不等于数据对任意偏移都有支撑。
    std::optional<TimeRange> trajectory_coverage;
};

/// 去畸变的结果。连同输出点云所处的系、参考时刻与所用标定版本一起交代清楚。
struct DeskewResult {
    DeskewStatus status = DeskewStatus::kNoPointTimes;
    std::size_t points = 0;                  ///< 实际变换的点数
    Timestamp reference_time{0};             ///< 变换到哪一刻
    std::uint64_t calibration_revision = 0;  ///< 用的是哪一版外参
    std::string output_frame;                ///< 输出点云所在的传感器系

    [[nodiscard]] bool corrected() const noexcept { return status == DeskewStatus::kCorrected; }
    /// 这一帧的点现在是不是已经在参考时刻：补过了，或者本来就补过。
    [[nodiscard]] bool compensated() const noexcept {
        return status == DeskewStatus::kCorrected
               || status == DeskewStatus::kAlreadyCompensated;
    }
};

/// 原地去畸变。
///
/// @param points        待变换的点云（传感器系）。必须有 `time_offset_ns` 列。
/// @param T_body_sensor 传感器到 body 的外参。
/// @param T_ref_body_at 可调用对象：给定相对参考时刻的偏移，返回
///                      "该时刻的 body 位姿 → 参考时刻的 body 位姿"的变换。
///                      约定 `T_ref_body_at(Duration{0}) == Identity()`。
///
/// 轨迹函数必须覆盖整段偏移区间。外推出去的那部分不是补偿，是编造，
/// 所以覆盖与否由提供轨迹的一方负责，这里不替它兜底。
///
/// 扫描期间外参视为不变（静态标定）。时变外参要另写公式，不能靠这一个变换表达。
/// 动态物体的运动也不在补偿范围内——自车刚体变换消不掉别人的运动。
template <class TrajectoryFn>
DeskewResult deskewInPlace(PointCloud& points, const SE3d& T_body_sensor,
                           TrajectoryFn&& T_ref_body_at, const DeskewContext& context = {}) {
    DeskewResult result;
    result.reference_time = context.reference_time;
    result.calibration_revision = context.calibration_revision;
    result.output_frame = context.sensor;
    if (points.empty()) {
        result.status = DeskewStatus::kEmpty;
        return result;
    }
    if (context.motion_compensation == MotionCompensation::kCompensated) {
        result.status = DeskewStatus::kAlreadyCompensated;
        return result;
    }
    if (points.time_offset_ns.empty()) {
        result.status = DeskewStatus::kNoPointTimes;
        return result;
    }
    if (points.time_offset_ns.size() != points.size()) {
        result.status = DeskewStatus::kMalformedTimes;
        return result;
    }
    if (context.trajectory_coverage) {
        // 先整帧核对覆盖再动点：半帧补偿、半帧外推的结果比原样返回更糟
        // 闭区间：参考时刻本身（偏移 0）必须算在覆盖之内
        const auto& cover = *context.trajectory_coverage;
        for (const auto offset : points.time_offset_ns) {
            const Timestamp t{offset};
            if (t < cover.begin || t > cover.end) {
                result.status = DeskewStatus::kTrajectoryNotCovered;
                return result;
            }
        }
    }

    const SE3d T_sensor_body = T_body_sensor.inverse();
    for (std::size_t i = 0; i < points.size(); ++i) {
        const Duration offset{points.time_offset_ns[i]};
        const SE3d T_ref_body = T_ref_body_at(offset);
        // 传感器系 → body 系 → 参考时刻 body 系 → 参考时刻传感器系
        const SE3d transform = T_sensor_body.compose(T_ref_body).compose(T_body_sensor);
        const Vec3d p(static_cast<double>(points.x[i]), static_cast<double>(points.y[i]),
                      static_cast<double>(points.z[i]));
        const Vec3d q = transform.act(p);
        points.x[i] = static_cast<float>(q.x());
        points.y[i] = static_cast<float>(q.y());
        points.z[i] = static_cast<float>(q.z());
    }
    result.status = DeskewStatus::kCorrected;
    result.points = points.size();
    return result;
}

/// 拷贝一份再去畸变。原始点是真源的一部分，不能被改（keyframe.hpp）。
/// 结果里带着状态，调用方据它判断这一帧到底补没补。
template <class TrajectoryFn>
[[nodiscard]] std::pair<PointCloud, DeskewResult> deskew(const PointCloud& points,
                                                         const SE3d& T_body_sensor,
                                                         TrajectoryFn&& T_ref_body_at,
                                                         const DeskewContext& context = {}) {
    PointCloud out = points;
    auto result = deskewInPlace(out, T_body_sensor, std::forward<TrajectoryFn>(T_ref_body_at),
                                context);
    return {std::move(out), result};
}

/// 最省事的轨迹函数：帧内匀速。`T_ref_body(Δt) = Exp(Δt · ξ)`，
/// ξ 是"参考时刻的 body 速度"（前 3 为平移、后 3 为旋转，右乘本体扰动）。
///
/// 真实 LIO 应当换成 IMU 预积分给出的轨迹；这个只用来起步与对照。
class ConstantTwist {
public:
    explicit ConstantTwist(const SE3d::Tangent& twist_per_second) : twist_(twist_per_second) {}

    SE3d operator()(Duration offset) const {
        const double seconds = static_cast<double>(offset.count()) * 1e-9;
        SE3d::Tangent scaled = twist_;
        scaled.coeffs() *= seconds;
        return scaled.exp();
    }

private:
    SE3d::Tangent twist_;
};

}  // namespace simpleslam::fw
