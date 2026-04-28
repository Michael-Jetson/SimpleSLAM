#pragma once

/// @file pcd_io.hpp
/// PCD 二进制文件读写——用于点云持久化和工具链互操作
///
/// 支持 PCL 标准 PCD 二进制格式的子集：
///   写入：points (必须) + intensities (可选) + normals (可选)
///   读取：x,y,z (必须) + intensity (可选) + normal_x,y,z (可选)

#include <SimpleSLAM/core/types/sensor_data.hpp>

#include <string>

namespace simpleslam::pcd_io {

/// 将 LidarScan 写入二进制 PCD 文件
void writePCD(const LidarScan& scan, const std::string& path);

/// 从二进制 PCD 文件读取 LidarScan
LidarScan readPCD(const std::string& path);

}  // namespace simpleslam::pcd_io
