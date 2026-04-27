#pragma once

/// @file common.hpp
/// 框架基础类型定义

#include <cstdint>
#include <type_traits>

namespace simpleslam {

/// 时间戳类型：双精度浮点，单位秒
/// 传感器数据中为 UTC 纪元秒；系统内部计时为 steady_clock 相对秒数
using Timestamp = double;

/// 带强度的三维点（单精度，节省内存，配准精度足够）
struct PointXYZI {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float intensity{0.0f};
};

// 保证 PointXYZI 内存布局紧凑——与 GPU buffer / PCL 互操作时需要
static_assert(sizeof(PointXYZI) == 4 * sizeof(float),
              "PointXYZI must be tightly packed (16 bytes)");
static_assert(std::is_trivially_copyable_v<PointXYZI>,
              "PointXYZI must be trivially copyable for zero-copy transfer");

}  // namespace simpleslam
