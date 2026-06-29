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

    // 时间戳：times.txt 存在则必须逐帧对齐；缺失才回退合成 10Hz。
    const fs::path times_file = seq_dir / "times.txt";
    if (fs::exists(times_file)) {
        timestamps_ = loadTimestamps(times_file);
        if (timestamps_.size() != bin_files_.size()) {
            // 存在但行数不符 = 数据集不完整/损坏。拒绝用合成时间戳静默掩盖
            // （否则真实前缀 + 合成后缀混合，dt/运动补偿被无声破坏）。
            throw std::runtime_error(
                "KittiSource: times.txt 行数 (" + std::to_string(timestamps_.size()) +
                ") 与 .bin 帧数 (" + std::to_string(bin_files_.size()) +
                ") 不匹配: " + times_file.string());
        }
    } else {
        // times.txt 缺失：回退合成 10Hz 时间线（部分 KITTI 序列确无 times.txt）
        timestamps_.resize(bin_files_.size());
        for (size_t i = 0; i < bin_files_.size(); ++i) {
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
    // tellg() 失败为 -1；非 kPointBytes 整数倍 = 文件截断/损坏。
    // 不守卫则 size_t(-1) 触发巨量分配，或整除截断后静默丢弃尾部字节。
    if (file_size <= 0 || static_cast<size_t>(file_size) % kPointBytes != 0) {
        throw std::runtime_error(
            "KittiSource: " + path.string() + " 大小非法 (" +
            std::to_string(static_cast<long long>(file_size)) + " 字节，应为 " +
            std::to_string(kPointBytes) + " 的正整数倍)——文件可能截断/损坏");
    }
    const size_t num_points = static_cast<size_t>(file_size) / kPointBytes;

    LidarScan scan;
    scan.timestamp = ts;
    scan.points.resize(num_points);
    auto& intensities = scan.intensities.emplace();
    intensities.resize(num_points);

    std::vector<float> buffer(num_points * 4);
    const auto want = static_cast<std::streamsize>(num_points * kPointBytes);
    file.read(reinterpret_cast<char*>(buffer.data()), want);
    // 短读不查 → buffer 默认 0 → 静默注入幻影 (0,0,0) 零强度点进配准。
    if (file.gcount() != want) {
        throw std::runtime_error(
            "KittiSource: " + path.string() + " 短读（期望 " + std::to_string(want) +
            " 字节，实读 " + std::to_string(file.gcount()) + "）——拒绝用幻影零点填充");
    }

    for (size_t i = 0; i < num_points; ++i) {
        scan.points[i] = Eigen::Vector3f(buffer[i * 4], buffer[i * 4 + 1], buffer[i * 4 + 2]);
        intensities[i] = buffer[i * 4 + 3];
    }

    assert(scan.isConsistent());
    return scan;
}

std::vector<Timestamp> KittiSource::loadTimestamps(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("KittiSource: cannot open " + path.string());
    }
    std::vector<Timestamp> timestamps;
    std::string line;
    size_t line_no = 0;
    while (std::getline(file, line)) {
        ++line_no;
        if (!line.empty() && line.back() == '\r') line.pop_back();  // CRLF
        if (line.find_first_not_of(" \t") == std::string::npos) continue;  // 纯空白行
        std::istringstream iss(line);
        double ts = 0.0;
        // 畸形行不能静默丢弃——否则后续每帧错配上一行的时间戳（整条尾部错位）。
        if (!(iss >> ts)) {
            throw std::runtime_error(
                "KittiSource: times.txt 第 " + std::to_string(line_no) +
                " 行无法解析为时间戳: '" + line + "'");
        }
        timestamps.push_back(ts);
    }
    return timestamps;
}

}  // namespace simpleslam
