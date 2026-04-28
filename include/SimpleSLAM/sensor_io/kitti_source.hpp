#pragma once

/// @file kitti_source.hpp
/// KITTI 数据集读取器
///
/// 读取 velodyne/ 目录下的 .bin 文件（float32 x4: x,y,z,intensity）
/// 可选读取 times.txt 获取时间戳

#include <SimpleSLAM/sensor_io/sensor_source.hpp>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace simpleslam {

class KittiSource final : public ISensorSource {
public:
    /// @param sequence_path 序列根目录（包含 velodyne/ 子目录和可选的 times.txt）
    explicit KittiSource(const std::string& sequence_path);

    [[nodiscard]] bool hasNext() const override;
    std::optional<LidarScan> nextScan() override;
    [[nodiscard]] Timestamp currentTimestamp() const override;

    /// 当前帧索引（0-based）
    [[nodiscard]] size_t currentIndex() const { return current_index_; }

    /// 总帧数
    [[nodiscard]] size_t totalFrames() const { return bin_files_.size(); }

private:
    std::vector<std::filesystem::path> bin_files_;
    std::vector<Timestamp> timestamps_;
    size_t current_index_{0};

    static LidarScan readBinFile(const std::filesystem::path& path, Timestamp ts);
    static std::vector<Timestamp> loadTimestamps(const std::filesystem::path& path);
};

}  // namespace simpleslam
