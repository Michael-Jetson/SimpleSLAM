#include <SimpleSLAM/sensor_io/euroc_source.hpp>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace simpleslam {

EurocSource::EurocSource(const std::string& dataset_path) {
    namespace fs = std::filesystem;
    const fs::path mav_dir(dataset_path);

    // 加载相机数据（cam0）
    const auto cam_csv = mav_dir / "cam0" / "data.csv";
    const auto cam_data = mav_dir / "cam0" / "data";
    if (fs::exists(cam_csv) && fs::is_directory(cam_data)) {
        image_entries_ = loadImageCsv(cam_csv, cam_data);
    }

    // 加载 IMU 数据
    const auto imu_csv = mav_dir / "imu0" / "data.csv";
    if (fs::exists(imu_csv)) {
        imu_samples_ = loadImuCsv(imu_csv);
    }

    if (image_entries_.empty() && imu_samples_.empty()) {
        throw std::runtime_error(
            "EurocSource: no data found in " + dataset_path);
    }
}

bool EurocSource::hasNext() const {
    return image_index_ < image_entries_.size() ||
           imu_index_ < imu_samples_.size();
}

std::optional<LidarScan> EurocSource::nextScan() {
    return std::nullopt;  // EuRoC 无 LiDAR
}

std::optional<ImuSample> EurocSource::nextImu() {
    if (imu_index_ >= imu_samples_.size()) return std::nullopt;
    return imu_samples_[imu_index_++];
}

std::optional<ImageFrame> EurocSource::nextImage() {
    if (image_index_ >= image_entries_.size()) return std::nullopt;
    const auto& entry = image_entries_[image_index_++];
    return readGrayscaleImage(entry.path, entry.timestamp);
}

Timestamp EurocSource::currentTimestamp() const {
    Timestamp ts = std::numeric_limits<double>::max();
    if (imu_index_ < imu_samples_.size()) {
        ts = std::min(ts, imu_samples_[imu_index_].timestamp);
    }
    if (image_index_ < image_entries_.size()) {
        ts = std::min(ts, image_entries_[image_index_].timestamp);
    }
    return ts;
}

std::vector<EurocSource::ImageEntry> EurocSource::loadImageCsv(
    const std::filesystem::path& csv_path,
    const std::filesystem::path& data_dir) {
    std::vector<ImageEntry> entries;
    std::ifstream file(csv_path);
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        std::string ts_str, filename;
        if (std::getline(iss, ts_str, ',') && std::getline(iss, filename)) {
            // 去除前导空格
            while (!filename.empty() && filename[0] == ' ') filename.erase(0, 1);
            // EuRoC 时间戳是纳秒
            double ts_ns = std::stod(ts_str);
            entries.push_back({ts_ns * 1e-9, data_dir / filename});
        }
    }

    std::sort(entries.begin(), entries.end(),
              [](const ImageEntry& a, const ImageEntry& b) {
                  return a.timestamp < b.timestamp;
              });
    return entries;
}

std::vector<ImuSample> EurocSource::loadImuCsv(
    const std::filesystem::path& csv_path) {
    std::vector<ImuSample> samples;
    std::ifstream file(csv_path);
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        std::string token;
        std::vector<double> values;
        while (std::getline(iss, token, ',')) {
            values.push_back(std::stod(token));
        }
        // 格式：timestamp[ns], gx, gy, gz, ax, ay, az
        if (values.size() >= 7) {
            ImuSample s;
            s.timestamp = values[0] * 1e-9;
            s.gyro = Vec3d(values[1], values[2], values[3]);
            s.acc = Vec3d(values[4], values[5], values[6]);
            samples.push_back(s);
        }
    }

    std::sort(samples.begin(), samples.end(),
              [](const ImuSample& a, const ImuSample& b) {
                  return a.timestamp < b.timestamp;
              });
    return samples;
}

ImageFrame EurocSource::readGrayscaleImage(
    const std::filesystem::path& path, Timestamp ts) {
    // 简易 PGM(P5) / PNG 读取——v1.0 只读 PGM 灰度图
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error(
            "EurocSource: cannot open image: " + path.string());
    }

    ImageFrame frame;
    frame.timestamp = ts;
    frame.channels = 1;

    // 尝试 PGM P5 格式
    std::string magic;
    file >> magic;
    if (magic == "P5") {
        file >> frame.width >> frame.height;
        int maxval;
        file >> maxval;
        file.get();  // 跳过换行符
        frame.data.resize(static_cast<size_t>(frame.width) * frame.height);
        file.read(reinterpret_cast<char*>(frame.data.data()),
                  static_cast<std::streamsize>(frame.data.size()));
    } else {
        // PNG 等其他格式：v1.5 引入 OpenCV 后支持
        // v1.0 仅记录文件路径，返回空帧
        frame.width = 0;
        frame.height = 0;
    }

    return frame;
}

}  // namespace simpleslam
