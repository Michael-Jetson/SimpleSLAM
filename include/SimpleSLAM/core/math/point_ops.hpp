#pragma once

/// @file point_ops.hpp
/// 点云工具函数——降采样、过滤、裁剪、变换、离群点移除
///
/// 设计约定（P6）：
///   - 全部为自由函数（不是 LidarScan 的成员方法）
///   - 过滤类函数返回新 LidarScan（不修改原数据，P2 值语义）
///   - 内部共享 extractByIndices 保证可选字段同步

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/core/types/sensor_data.hpp>

#include <Eigen/Core>

namespace simpleslam {

/// 体素降采样——每个体素输出质心点
/// @param voxel_size 体素边长（米），必须 > 0
LidarScan voxelDownsample(const LidarScan& input, float voxel_size);

/// 距离过滤——保留距原点在 [min_range, max_range] 内的点
LidarScan rangeFilter(const LidarScan& input, float min_range, float max_range);

/// 轴对齐包围盒裁剪——保留在 [min_pt, max_pt] 内的点
LidarScan cropBox(const LidarScan& input,
                  const Eigen::Vector3f& min_pt,
                  const Eigen::Vector3f& max_pt);

/// 刚体变换——对所有点（和法向量）施加 SE3 变换
/// 点坐标：p' = T.act(p)
/// 法向量：n' = R * n（只用旋转部分）
LidarScan transformScan(const LidarScan& input, const SE3d& transform);

/// 统计离群点移除——移除 KNN 均距超过 mean + stddev_multiplier * σ 的点
/// @param k_neighbors     近邻数量
/// @param stddev_multiplier 标准差倍率阈值
LidarScan statisticalOutlierRemoval(const LidarScan& input,
                                    int k_neighbors,
                                    float stddev_multiplier);

}  // namespace simpleslam
