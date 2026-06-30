#pragma once

/// @file graph_epoch.hpp
/// GraphEpoch —— PoseGraph 版本号强类型（ADR-002 / 蓝图不变量4）。
///
/// 后端校正携带"其计算所基于的图版本"；消费者（SubMapManager / Odometry）
/// 据此拒绝过期或基于错误版本的校正。对标 GTSAM iSAM2 的 update_count_。
/// 强类型（非裸 uint64_t）：编译期防止与 keyframe_id / submap_id 等混用。

#include <cstdint>

namespace simpleslam {

/// 图版本号。单调递增——每次 PoseGraph 结构性变更 +1。
/// scoped enum 原生支持 < <= == 等关系运算，可直接比较新旧。
enum class GraphEpoch : uint64_t {};

/// 取底层数值（日志、序列化、容器键用）。
[[nodiscard]] constexpr uint64_t epochValue(GraphEpoch e) noexcept {
    return static_cast<uint64_t>(e);
}

}  // namespace simpleslam
