#pragma once

/// @file sensor_data.hpp
/// 传感器数据类型——LidarScan / ImageFrame / ImuSample
///
/// LidarScan 是"一帧传感器数据的快照"，不是通用点云容器，也不是地图数据结构。
/// 地图由 RegistrationTarget 内部管理。
///
/// 设计要点：
///   - SOA 布局：每个属性独立数组，配准热路径只遍历 points
///   - 三级字段：必须 -> 一等可选 -> 泛型扩展
///   - 有序/无序通过 ScanLayout 元数据区分，不改存储方式
///   - 所有可选数组要么空（nullopt），要么 size() == points.size()
///
/// 生命周期：
///   SensorIO 创建 -> point_ops 处理 -> shared_ptr<const> 零拷贝共享 -> 后端消费

#include <SimpleSLAM/core/types/common.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>

#include <any>
#include <array>
#include <cassert>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace simpleslam {

// ── 基础类型 ──

/// RGBA 颜色（4 字节对齐，GPU/渲染兼容）
using ColorRGBA = std::array<uint8_t, 4>;

/// 点云的空间组织方式
struct ScanLayout {
    enum class Type : uint8_t {
        Unorganized,  ///< 散乱点（Livox、降采样后的点云）
        Organized,    ///< H*W 网格（机械式 LiDAR、RGB-D）
    };

    Type type = Type::Unorganized;
    uint32_t height = 0;  ///< 行数（线束数）。Unorganized 时 = 0
    uint32_t width  = 0;  ///< 列数（方位角步数）。Unorganized 时 = 0

    /// 2D -> 1D 索引转换（仅 Organized 有效）
    [[nodiscard]] size_t index(uint32_t row, uint32_t col) const {
        assert(type == Type::Organized && row < height && col < width);
        return static_cast<size_t>(row) * width + col;
    }

    /// 1D -> 2D 索引转换（仅 Organized 有效）
    [[nodiscard]] std::pair<uint32_t, uint32_t> rowCol(size_t idx) const {
        assert(type == Type::Organized && width > 0);
        return {static_cast<uint32_t>(idx / width),
                static_cast<uint32_t>(idx % width)};
    }

    /// 有序点云中，NaN 坐标标识无回波位置
    [[nodiscard]] static bool isValid(const Eigen::Vector3f& p) {
        return p.allFinite();
    }
};

// ── LidarScan ──

struct LidarScan {

    // ── 必须字段 ──

    Timestamp timestamp{0.0};                          ///< 扫描起始时间（UTC 纪元秒）
    std::vector<Eigen::Vector3f> points;               ///< 点坐标（传感器坐标系）

    // ── 组织信息 ──

    ScanLayout layout;

    // ── 一等可选字段（覆盖 >95% 场景）──

    std::optional<std::vector<float>> point_offsets;    ///< 每点时间偏移（相对于 timestamp 的秒数）
    std::optional<std::vector<float>> intensities;      ///< 反射强度（不限范围的 float 原始值）
    std::optional<std::vector<ColorRGBA>> colors;       ///< RGBA 颜色
    std::optional<std::vector<Eigen::Vector3f>> normals;///< 法向量
    std::optional<std::vector<uint16_t>> rings;         ///< 线束编号（0-based）

    // ── 泛型扩展（罕见字段）──

    /// 扩展属性表——不在热路径使用（any_cast 有运行时开销）
    /// 某个 extras 字段被 >=3 个核心模块使用时应提升为一等字段
    std::unordered_map<std::string, std::any> extras;

    // ── 便利方法 ──

    [[nodiscard]] size_t size() const { return points.size(); }
    [[nodiscard]] bool empty() const { return points.empty(); }
    [[nodiscard]] bool isOrganized() const { return layout.type == ScanLayout::Type::Organized; }

    [[nodiscard]] bool hasPointOffsets() const { return point_offsets.has_value() && !point_offsets->empty(); }
    [[nodiscard]] bool hasIntensities() const { return intensities.has_value() && !intensities->empty(); }
    [[nodiscard]] bool hasColors() const { return colors.has_value() && !colors->empty(); }
    [[nodiscard]] bool hasNormals() const { return normals.has_value() && !normals->empty(); }
    [[nodiscard]] bool hasRings() const { return rings.has_value() && !rings->empty(); }
    [[nodiscard]] bool hasExtra(const std::string& key) const { return extras.count(key) > 0; }

    /// 安全获取扩展字段（类型不匹配或不存在时返回 nullptr）
    template <typename T>
    [[nodiscard]] const T* tryGetExtra(const std::string& key) const {
        auto it = extras.find(key);
        if (it == extras.end()) return nullptr;
        return std::any_cast<T>(&it->second);
    }

    /// 预分配已启用的字段（先 emplace 需要的可选字段，再调用 reserve）
    void reserve(size_t n) {
        points.reserve(n);
        if (point_offsets) point_offsets->reserve(n);
        if (intensities)   intensities->reserve(n);
        if (colors)        colors->reserve(n);
        if (normals)       normals->reserve(n);
        if (rings)         rings->reserve(n);
    }

    /// 一致性校验（debug 断言用）
    [[nodiscard]] bool isConsistent() const {
        const auto n = points.size();
        if (point_offsets && point_offsets->size() != n) return false;
        if (intensities  && intensities->size()  != n) return false;
        if (colors       && colors->size()       != n) return false;
        if (normals      && normals->size()      != n) return false;
        if (rings        && rings->size()        != n) return false;
        if (isOrganized() && static_cast<size_t>(layout.height) * layout.width != n) return false;
        return true;
    }
};

// ── 颜色工具 ──

namespace color_utils {

inline Eigen::Vector3f toFloat(const ColorRGBA& c) {
    constexpr float kInv255 = 1.0f / 255.0f;
    return {c[0] * kInv255, c[1] * kInv255, c[2] * kInv255};
}

inline ColorRGBA fromFloat(const Eigen::Vector3f& c) {
    auto toByte = [](float v) -> uint8_t {
        return static_cast<uint8_t>(std::clamp(v * 255.0f, 0.0f, 255.0f));
    };
    return {toByte(c.x()), toByte(c.y()), toByte(c.z()), 255};
}

inline ColorRGBA fromPCLPacked(uint32_t packed) {
    return {static_cast<uint8_t>((packed >> 16) & 0xFF),
            static_cast<uint8_t>((packed >> 8)  & 0xFF),
            static_cast<uint8_t>( packed        & 0xFF),
            static_cast<uint8_t>((packed >> 24) & 0xFF)};
}

}  // namespace color_utils

// ── ImageFrame ──

/// 轻量图像容器（不依赖 OpenCV），v1.5 时加 toCvMat()/fromCvMat() 适配
struct ImageFrame {
    Timestamp timestamp{0.0};
    uint32_t width = 0, height = 0;
    uint8_t channels = 1;           ///< 1=灰度, 3=RGB, 4=RGBA
    std::vector<uint8_t> data;      ///< 行优先 HWC 布局
    uint8_t camera_id = 0;

    [[nodiscard]] size_t pixelCount() const { return static_cast<size_t>(width) * height; }
    [[nodiscard]] bool empty() const { return data.empty(); }
    [[nodiscard]] bool isConsistent() const { return data.size() == static_cast<size_t>(width) * height * channels; }
};

// ── IMU 数据 ──

struct ImuSample {
    Timestamp timestamp{0.0};
    Vec3d acc{0.0, 0.0, 0.0};   ///< 加速度（m/s^2，含重力）
    Vec3d gyro{0.0, 0.0, 0.0};  ///< 角速度（rad/s）
};

using ImuBatch = std::vector<ImuSample>;

}  // namespace simpleslam
