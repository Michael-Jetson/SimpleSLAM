#pragma once

/// @file scan_utils.hpp
/// 点云过滤的公共帮助器——extractByIndices 和 RangeImageView
///
/// extractByIndices 是所有过滤函数（voxelDownsample、rangeFilter 等）的内部共享实现，
/// 保证可选字段（intensities、normals、rings 等）在过滤后保持同步。

#include <SimpleSLAM/core/types/sensor_data.hpp>

#include <Eigen/Core>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace simpleslam {

/// 按索引提取子集——所有过滤函数的共享帮助器
/// 正确同步所有已启用的可选字段。结果 layout 设为 Unorganized（过滤破坏有序结构）。
inline LidarScan extractByIndices(const LidarScan& input,
                                  std::span<const size_t> indices) {
    LidarScan result;
    result.timestamp = input.timestamp;
    result.layout.type = ScanLayout::Type::Unorganized;

    const size_t n = indices.size();
    result.points.resize(n);
    for (size_t i = 0; i < n; ++i) {
        result.points[i] = input.points[indices[i]];
    }

    if (input.hasPointOffsets()) {
        auto& dst = result.point_offsets.emplace();
        dst.resize(n);
        for (size_t i = 0; i < n; ++i) dst[i] = (*input.point_offsets)[indices[i]];
    }

    if (input.hasIntensities()) {
        auto& dst = result.intensities.emplace();
        dst.resize(n);
        for (size_t i = 0; i < n; ++i) dst[i] = (*input.intensities)[indices[i]];
    }

    if (input.hasColors()) {
        auto& dst = result.colors.emplace();
        dst.resize(n);
        for (size_t i = 0; i < n; ++i) dst[i] = (*input.colors)[indices[i]];
    }

    if (input.hasNormals()) {
        auto& dst = result.normals.emplace();
        dst.resize(n);
        for (size_t i = 0; i < n; ++i) dst[i] = (*input.normals)[indices[i]];
    }

    if (input.hasRings()) {
        auto& dst = result.rings.emplace();
        dst.resize(n);
        for (size_t i = 0; i < n; ++i) dst[i] = (*input.rings)[indices[i]];
    }

    assert(result.isConsistent());
    return result;
}

/// 有序点云的 2D 只读视图
/// 不持有数据——引用的 LidarScan 必须比本对象活得更长
class RangeImageView {
public:
    explicit RangeImageView(const LidarScan& scan)
        : scan_(scan) {
        assert(scan.isOrganized());
    }

    [[nodiscard]] const Eigen::Vector3f& at(uint32_t row, uint32_t col) const {
        return scan_.points[scan_.layout.index(row, col)];
    }

    [[nodiscard]] bool isValid(uint32_t row, uint32_t col) const {
        return ScanLayout::isValid(at(row, col));
    }

    [[nodiscard]] uint32_t height() const { return scan_.layout.height; }
    [[nodiscard]] uint32_t width() const { return scan_.layout.width; }

private:
    const LidarScan& scan_;
};

}  // namespace simpleslam
