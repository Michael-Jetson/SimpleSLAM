#pragma once

/// @file normal_estimation.hpp
/// 法向量估计——KNN PCA 方法
///
/// 原地修改 LidarScan 的 normals 字段（命名为动词，P6）。
/// 假设 scan 在传感器坐标系——法向量朝向传感器原点。

#include <SimpleSLAM/core/types/sensor_data.hpp>

namespace simpleslam {

/// 为点云中每个点估计法向量（KNN + PCA）
/// @param scan          输入/输出——normals 字段会被填充
/// @param k_neighbors   PCA 使用的近邻数量（建议 10~20）
void estimateNormals(LidarScan& scan, int k_neighbors = 10);

}  // namespace simpleslam
