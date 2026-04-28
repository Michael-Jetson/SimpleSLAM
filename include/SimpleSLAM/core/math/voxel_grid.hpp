#pragma once

/// @file voxel_grid.hpp
/// 体素网格数据结构——VoxelCoord / VoxelHash / VoxelMap
///
/// 用于体素降采样和体素哈希地图的基础设施。
/// 公共接口不暴露 tsl::robin_map 类型（P1 接口最小化）。

#include <Eigen/Core>
#include <tsl/robin_map.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace simpleslam {

/// 三维整数体素坐标
struct VoxelCoord {
    int32_t x{0}, y{0}, z{0};

    bool operator==(const VoxelCoord&) const = default;
};

/// 体素坐标哈希仿函数（FNV-1a 混合三个坐标）
struct VoxelHash {
    size_t operator()(const VoxelCoord& v) const {
        // FNV-1a 64-bit
        size_t h = 14695981039346656037ULL;
        auto mix = [&h](int32_t val) {
            const auto* bytes = reinterpret_cast<const uint8_t*>(&val);
            for (size_t i = 0; i < sizeof(int32_t); ++i) {
                h ^= static_cast<size_t>(bytes[i]);
                h *= 1099511628211ULL;
            }
        };
        mix(v.x);
        mix(v.y);
        mix(v.z);
        return h;
    }

    /// Morton Z-order 编码——将空间相邻的体素映射到数值相邻的键
    /// 改善缓存命中率（Surfel-LIO 已验证效果）
    /// 输入坐标须为非负，建议先偏移到正数范围
    static uint64_t mortonEncode(uint32_t x, uint32_t y, uint32_t z) {
        auto spread = [](uint32_t v) -> uint64_t {
            uint64_t r = v & 0x1FFFFF;  // 21 bits
            r = (r | (r << 32)) & 0x1F00000000FFFF;
            r = (r | (r << 16)) & 0x1F0000FF0000FF;
            r = (r | (r <<  8)) & 0x100F00F00F00F00F;
            r = (r | (r <<  4)) & 0x10C30C30C30C30C3;
            r = (r | (r <<  2)) & 0x1249249249249249;
            return r;
        };
        return spread(x) | (spread(y) << 1) | (spread(z) << 2);
    }
};

/// 连续空间坐标 -> 体素网格坐标（向负无穷取整）
inline VoxelCoord toVoxelCoord(const Eigen::Vector3f& point, float voxel_size) {
    assert(voxel_size > 0.0f);
    const float inv = 1.0f / voxel_size;
    return {
        static_cast<int32_t>(std::floor(point.x() * inv)),
        static_cast<int32_t>(std::floor(point.y() * inv)),
        static_cast<int32_t>(std::floor(point.z() * inv)),
    };
}

/// 单个体素——存储落入该格子的点和预计算质心
struct Voxel {
    std::vector<Eigen::Vector3f> points;

    [[nodiscard]] size_t numPoints() const { return points.size(); }
    [[nodiscard]] bool empty() const { return points.empty(); }

    /// 计算质心
    [[nodiscard]] Eigen::Vector3f centroid() const {
        assert(!empty());
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        for (const auto& p : points) sum += p;
        return sum / static_cast<float>(points.size());
    }
};

/// 体素哈希地图——基于 tsl::robin_map 的高性能哈希表
class VoxelMap {
public:
    /// 向指定体素中添加一个点
    void insert(const VoxelCoord& coord, const Eigen::Vector3f& point) {
        map_[coord].points.push_back(point);
    }

    /// 查找体素，不存在返回 nullptr
    [[nodiscard]] const Voxel* find(const VoxelCoord& coord) const {
        auto it = map_.find(coord);
        return it != map_.end() ? &it->second : nullptr;
    }

    /// 可变版本查找（tsl::robin_map 用 it.value() 获取可变引用）
    [[nodiscard]] Voxel* find(const VoxelCoord& coord) {
        auto it = map_.find(coord);
        return it != map_.end() ? &it.value() : nullptr;
    }

    [[nodiscard]] bool contains(const VoxelCoord& coord) const {
        return map_.count(coord) > 0;
    }

    [[nodiscard]] size_t size() const { return map_.size(); }
    [[nodiscard]] bool empty() const { return map_.empty(); }
    void clear() { map_.clear(); }

    /// 列出所有已占据的体素坐标
    [[nodiscard]] std::vector<VoxelCoord> occupiedCoords() const {
        std::vector<VoxelCoord> coords;
        coords.reserve(map_.size());
        for (const auto& [coord, _] : map_) {
            coords.push_back(coord);
        }
        return coords;
    }

    /// 遍历所有体素（只读）
    template <typename Func>
    void forEach(Func&& fn) const {
        for (const auto& [coord, voxel] : map_) {
            fn(coord, voxel);
        }
    }

private:
    tsl::robin_map<VoxelCoord, Voxel, VoxelHash> map_;
};

}  // namespace simpleslam
