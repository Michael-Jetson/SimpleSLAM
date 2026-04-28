#pragma once

/// @file euroc_source.hpp
/// EuRoC MAV 数据集读取器——视觉+惯性数据源（无 LiDAR）
///
/// 读取 cam0/data.csv + imu0/data.csv
/// 时间戳为纳秒，内部转换为 Timestamp（double 秒）

#include <SimpleSLAM/sensor_io/sensor_source.hpp>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace simpleslam {

class EurocSource final : public ISensorSource {
public:
    /// @param dataset_path mav0/ 目录路径
    explicit EurocSource(const std::string& dataset_path);

    [[nodiscard]] bool hasNext() const override;
    std::optional<LidarScan> nextScan() override;       ///< 始终返回 nullopt
    std::optional<ImuSample> nextImu() override;
    std::optional<ImageFrame> nextImage() override;
    [[nodiscard]] Timestamp currentTimestamp() const override;

    [[nodiscard]] size_t totalImages() const { return image_entries_.size(); }
    [[nodiscard]] size_t totalImuSamples() const { return imu_samples_.size(); }

private:
    struct ImageEntry {
        Timestamp timestamp;
        std::filesystem::path path;
    };

    std::vector<ImageEntry> image_entries_;
    std::vector<ImuSample> imu_samples_;
    size_t image_index_{0};
    size_t imu_index_{0};

    static std::vector<ImageEntry> loadImageCsv(
        const std::filesystem::path& csv_path,
        const std::filesystem::path& data_dir);
    static std::vector<ImuSample> loadImuCsv(
        const std::filesystem::path& csv_path);
    static ImageFrame readGrayscaleImage(
        const std::filesystem::path& path, Timestamp ts);
};

}  // namespace simpleslam
