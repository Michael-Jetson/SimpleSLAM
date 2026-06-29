#include <SimpleSLAM/sensor_io/kitti_source.hpp>

#include <algorithm>
#include <cassert>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace simpleslam {

KittiSource::KittiSource(const std::string& sequence_path) {
    namespace fs = std::filesystem;
    const fs::path seq_dir(sequence_path);
    const fs::path velodyne_dir = seq_dir / "velodyne";

    if (!fs::is_directory(velodyne_dir)) {
        throw std::runtime_error(
            "KittiSource: velodyne directory not found: " + velodyne_dir.string());
    }

    for (const auto& entry : fs::directory_iterator(velodyne_dir)) {
        if (entry.path().extension() == ".bin") {
            bin_files_.push_back(entry.path());
        }
    }

    // 按文件名排序（000000.bin, 000001.bin, ...）
    std::sort(bin_files_.begin(), bin_files_.end());

    if (bin_files_.empty()) {
        throw std::runtime_error(
            "KittiSource: no .bin files found in " + velodyne_dir.string());
    }

    // 尝试加载时间戳
    const fs::path times_file = seq_dir / "times.txt";
    if (fs::exists(times_file)) {
        timestamps_ = loadTimestamps(times_file);
    }

    // 时间戳不足时用帧号 * 0.1s 补齐
    if (timestamps_.size() < bin_files_.size()) {
        const size_t start = timestamps_.size();
        timestamps_.resize(bin_files_.size());
        for (size_t i = start; i < bin_files_.size(); ++i) {
            timestamps_[i] = static_cast<Timestamp>(i) * 0.1;
        }
    }
}

bool KittiSource::hasNext() const {
    return current_index_ < bin_files_.size();
}

std::optional<LidarScan> KittiSource::nextScan() {
    if (!hasNext()) return std::nullopt;
    auto scan = readBinFile(bin_files_[current_index_], timestamps_[current_index_]);
    ++current_index_;
    return scan;
}

Timestamp KittiSource::currentTimestamp() const {
    if (current_index_ < timestamps_.size()) {
        return timestamps_[current_index_];
    }
    return 0.0;
}

LidarScan KittiSource::readBinFile(const std::filesystem::path& path, Timestamp ts) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("KittiSource: cannot open " + path.string());
    }

    file.seekg(0, std::ios::end);
    const auto file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    // KITTI .bin 格式：连续的 float32 四元组 (x, y, z, intensity)
    constexpr size_t kPointBytes = 4 * sizeof(float);
    const size_t num_points = static_cast<size_t>(file_size) / kPointBytes;

    LidarScan scan;
    scan.timestamp = ts;
    scan.points.resize(num_points);
    auto& intensities = scan.intensities.emplace();
    intensities.resize(num_points);

    std::vector<float> buffer(num_points * 4);
    file.read(reinterpret_cast<char*>(buffer.data()),
              static_cast<std::streamsize>(num_points * kPointBytes));

    for (size_t i = 0; i < num_points; ++i) {
        scan.points[i] = Eigen::Vector3f(buffer[i * 4], buffer[i * 4 + 1], buffer[i * 4 + 2]);
        intensities[i] = buffer[i * 4 + 3];
    }

    assert(scan.isConsistent());
    return scan;
}

std::vector<Timestamp> KittiSource::loadTimestamps(const std::filesystem::path& path) {
    std::vector<Timestamp> timestamps;
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        double ts = 0.0;
        if (iss >> ts) {
            timestamps.push_back(ts);
        }
    }
    return timestamps;
}

}  // namespace simpleslam
