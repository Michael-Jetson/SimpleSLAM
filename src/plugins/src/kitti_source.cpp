// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/plugins/sources/kitti.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <utility>

namespace simpleslam::plugin::kitti {

namespace {

/// 典型相邻间隔：取中位数。中位数不被个别长缺口带偏——拿最大间隔或总跨度去推
/// "这条流本来的样子"，一次断流就会把阈值放宽到接受断流本身。
fw::Duration typicalGap(const std::vector<fw::Timestamp>& times) {
    if (times.size() < 2) return fw::Duration{0};
    std::vector<fw::Duration> gaps;
    gaps.reserve(times.size() - 1);
    for (std::size_t i = 1; i < times.size(); ++i) gaps.push_back(times[i] - times[i - 1]);
    const auto middle = gaps.begin() + static_cast<std::ptrdiff_t>(gaps.size() / 2);
    std::nth_element(gaps.begin(), middle, gaps.end());
    return *middle;
}

/// 实测采样率：按典型间隔换算，而不是按声明的标称值。
double measuredRateHz(const std::vector<fw::Timestamp>& times) {
    const auto gap = typicalGap(times);
    if (gap.count() <= 0) return 0.0;
    return 1.0 / std::chrono::duration<double>(gap).count();
}

/// 实测最大相邻间隔。两端有样本不等于中间没有长缺口。
fw::Duration maxGap(const std::vector<fw::Timestamp>& times) {
    fw::Duration largest{0};
    for (std::size_t i = 1; i < times.size(); ++i) {
        const auto gap = times[i] - times[i - 1];
        if (gap > largest) largest = gap;
    }
    return largest;
}

namespace fs = std::filesystem;
constexpr double kPi = 3.14159265358979323846;

// ── 文本与文件 ──

std::vector<std::string> readLines(const fs::path& path) {
    std::ifstream file(path);
    if (!file) throw KittiError("打不开文件: " + path.string());
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        lines.push_back(std::move(line));
    }
    return lines;
}

/// 解析一行空白分隔的数。每一个记号都必须完整地是一个有限数：
/// 流式读取遇到第一个坏记号就静静停下，前面读到几个算几个——坏行因此被当成短行，
/// 错误在后面的某个下标越界处才以别的面目出现。
std::vector<double> parseNumbers(const std::string& text) {
    std::istringstream stream(text);
    std::vector<double> numbers;
    std::string token;
    while (stream >> token) {
        std::size_t used = 0;
        double value = 0.0;
        try {
            value = std::stod(token, &used);
        } catch (const std::exception&) {
            throw KittiError("不是一个数: " + token);
        }
        if (used != token.size() || !std::isfinite(value)) {
            throw KittiError("不是一个有限数: " + token);
        }
        numbers.push_back(value);
    }
    return numbers;
}

/// 检查一个 3×3 矩阵确实是旋转，再转成四元数。
///
/// 数据集文件里的矩阵同样可能写坏；直接归一化四元数，一个带尺度或含反射的矩阵
/// 会变成一个"合法"但错误的旋转。与标定文件读取用同一个判据。
Eigen::Quaterniond checkedRotation(const Eigen::Matrix3d& rotation, const std::string& origin) {
    if (!rotation.allFinite()) throw KittiError("旋转矩阵含非有限值: " + origin);
    if (!(rotation * rotation.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-4)) {
        throw KittiError("旋转矩阵不正交: " + origin);
    }
    if (rotation.determinant() < 0.0) throw KittiError("旋转矩阵含反射: " + origin);
    return Eigen::Quaterniond(rotation).normalized();
}

std::vector<fs::path> sortedFiles(const fs::path& directory, const char* extension) {
    if (!fs::is_directory(directory)) {
        throw KittiError("目录不存在: " + directory.string());
    }
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.path().extension() == extension) files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());  // 000000.bin, 000001.bin, ...
    // 文件名就是帧号：它必须从 0 起连续，否则"第 i 个文件对应第 i 行时间"就不成立，
    // 少一个文件会让之后每一帧都配错时间
    for (std::size_t i = 0; i < files.size(); ++i) {
        const std::string stem = files[i].stem().string();
        std::size_t used = 0;
        unsigned long long number = 0;
        try {
            number = std::stoull(stem, &used);
        } catch (const std::exception&) {
            throw KittiError("文件名不是帧号: " + files[i].string());
        }
        if (used != stem.size() || number != i) {
            throw KittiError("帧号不连续：第 " + std::to_string(i) + " 个文件是 "
                             + files[i].filename().string());
        }
    }
    if (files.empty()) {
        throw KittiError("目录里没有 " + std::string(extension) + " 文件: " + directory.string());
    }
    return files;
}

/// KITTI 的 3×4 行主序矩阵 → SE3。旋转部分从文件读出来不一定严格正交，
/// 所以走四元数归一化，而不是假装它已经是旋转矩阵。
SE3d poseFromRowMajor(const std::vector<double>& n, const std::string& origin,
                      std::size_t offset = 0) {
    if (n.size() < offset + 12) throw KittiError("位姿需要 12 个数: " + origin);
    Eigen::Matrix3d rotation;
    rotation << n[offset + 0], n[offset + 1], n[offset + 2],
                n[offset + 4], n[offset + 5], n[offset + 6],
                n[offset + 8], n[offset + 9], n[offset + 10];
    const Vec3d translation(n[offset + 3], n[offset + 7], n[offset + 11]);
    return SE3d(translation, checkedRotation(rotation, origin));
}

/// `R: 9 个数` + `T: 3 个数` 这种布局（raw 的标定文件）。
SE3d poseFromRT(const std::vector<std::string>& lines, const fs::path& origin) {
    std::vector<double> r;
    std::vector<double> t;
    for (const auto& line : lines) {
        if (line.rfind("R:", 0) == 0) r = parseNumbers(line.substr(2));
        else if (line.rfind("T:", 0) == 0) t = parseNumbers(line.substr(2));
    }
    if (r.size() != 9 || t.size() != 3) throw KittiError("标定文件的 R/T 个数不对: " + origin.string());
    Eigen::Matrix3d rotation;
    rotation << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
    return SE3d(Vec3d(t[0], t[1], t[2]), checkedRotation(rotation, origin.string()));
}

// ── 时间 ──

fw::Timestamp secondsToTimestamp(double seconds) {
    return fw::Timestamp{static_cast<std::int64_t>(std::llround(seconds * 1e9))};
}

/// raw 的时间戳形如 `2011-09-26 13:02:25.964389445`（UTC）。
/// 手工解析而不用 std::get_time：后者受 locale 影响，而且拿不到纳秒。
fw::Timestamp parseRawTimestamp(const std::string& text) {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    char fraction[16] = {0};
    int consumed = 0;
    const int matched = std::sscanf(text.c_str(), "%4d-%2d-%2d %2d:%2d:%2d.%15[0-9]%n", &year,
                                    &month, &day, &hour, &minute, &second, fraction, &consumed);
    if (matched < 7) throw KittiError("无法解析时间戳（要求 YYYY-MM-DD HH:MM:SS.小数）: " + text);
    // 尾随字符与各字段范围都要核：sscanf 只读到它认得的地方为止
    const auto rest = text.find_first_not_of(" \t", static_cast<std::size_t>(consumed));
    if (rest != std::string::npos) throw KittiError("时间戳后面有多余字符: " + text);
    if (month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0
        || minute > 59 || second < 0 || second > 60) {
        throw KittiError("时间戳字段越界: " + text);
    }

    std::tm tm{};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = second;
    const std::time_t epoch_seconds = timegm(&tm);  // UTC，不受 TZ 影响

    std::string digits(fraction);
    if (digits.size() > 9) throw KittiError("时间戳的小数位超过纳秒精度: " + text);
    digits.resize(9, '0');  // 补齐到纳秒
    const std::int64_t nanos = std::stoll(digits);
    return fw::Timestamp{static_cast<std::int64_t>(epoch_seconds) * 1000000000LL + nanos};
}

std::vector<fw::Timestamp> readRawTimestamps(const fs::path& path) {
    std::vector<fw::Timestamp> stamps;
    std::size_t row = 0;
    for (const auto& line : readLines(path)) {
        ++row;
        if (line.find_first_not_of(" \t") == std::string::npos) continue;  // 空白行
        const auto stamp = parseRawTimestamp(line);
        // 重复的时间戳与倒退一样不可接受：dt 为零会让积分与插值除以零
        if (!stamps.empty() && stamp <= stamps.back()) {
            throw KittiError("时间戳第 " + std::to_string(row) + " 行没有严格递增: "
                             + path.string());
        }
        stamps.push_back(stamp);
    }
    return stamps;
}

// ── 点云 ──

/// 按方位角反推逐点时间。Velodyne 匀速旋转，方位角就是帧内相对时刻。
/// 参考时刻取扫描终点，所以偏移落在 (-period, 0]。
std::int32_t pointTimeOffset(float x, float y, std::int64_t period_ns) {
    const double yaw = -std::atan2(static_cast<double>(y), static_cast<double>(x));
    double ratio = 0.5 * (yaw / kPi + 1.0);
    ratio = std::clamp(ratio, 0.0, 1.0);
    return static_cast<std::int32_t>(std::llround((ratio - 1.0) * static_cast<double>(period_ns)));
}

fw::PointCloud readVelodyneBin(const fs::path& path, bool derive_times, std::int64_t period_ns) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) throw KittiError("打不开点云: " + path.string());
    const std::streamsize bytes = file.tellg();
    if (bytes < 0 || bytes % (4 * sizeof(float)) != 0) {
        throw KittiError("点云文件长度不是 4×float 的整数倍: " + path.string());
    }
    file.seekg(0, std::ios::beg);

    const std::size_t count = static_cast<std::size_t>(bytes) / (4 * sizeof(float));
    std::vector<float> raw(count * 4);
    file.read(reinterpret_cast<char*>(raw.data()), bytes);
    if (file.gcount() != bytes) throw KittiError("点云读取不完整: " + path.string());

    fw::PointCloud points;
    points.reserve(count);
    points.intensity.reserve(count);
    if (derive_times) points.time_offset_ns.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const float x = raw[i * 4 + 0];
        const float y = raw[i * 4 + 1];
        const float z = raw[i * 4 + 2];
        points.push(x, y, z);
        points.intensity.push_back(raw[i * 4 + 3]);
        if (derive_times) points.time_offset_ns.push_back(pointTimeOffset(x, y, period_ns));
    }
    return points;
}

// ── 大地坐标 ──

/// KITTI devkit 的 Mercator 投影：首帧纬度定尺度，之后都用同一个尺度。
struct Mercator {
    double scale = 1.0;
    [[nodiscard]] Vec3d project(double lat_deg, double lon_deg, double alt_m) const {
        constexpr double kEarthRadius = 6378137.0;
        const double x = scale * lon_deg * kPi * kEarthRadius / 180.0;
        const double y = scale * kEarthRadius
                         * std::log(std::tan((90.0 + lat_deg) * kPi / 360.0));
        return Vec3d(x, y, alt_m);
    }
};

Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Vec3d::UnitZ())
                              * Eigen::AngleAxisd(pitch, Vec3d::UnitY())
                              * Eigen::AngleAxisd(roll, Vec3d::UnitX()))
        .normalized();
}

/// WGS84 大地坐标 → ECEF。GnssFix 的位置按公约就在 ECEF。
std::array<double, 3> geodeticToEcef(double lat_deg, double lon_deg, double alt_m) {
    constexpr double kA = 6378137.0;
    constexpr double kF = 1.0 / 298.257223563;
    constexpr double kE2 = kF * (2.0 - kF);
    const double lat = lat_deg * kPi / 180.0;
    const double lon = lon_deg * kPi / 180.0;
    const double sin_lat = std::sin(lat);
    const double n = kA / std::sqrt(1.0 - kE2 * sin_lat * sin_lat);
    return {(n + alt_m) * std::cos(lat) * std::cos(lon),
            (n + alt_m) * std::cos(lat) * std::sin(lon),
            (n * (1.0 - kE2) + alt_m) * sin_lat};
}

/// oxts 的一行：30 个字段。这里只取建图用得到的那些，下标按 devkit 的 dataformat.txt。
struct OxtsRecord {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    /// ax/ay/az、wx/wy/wz：**车体系**的比力与角速度。body = IMU 时要的是这一组。
    std::array<double, 3> accel_body{};
    std::array<double, 3> gyro_body{};
    /// af/al/au、wf/wl/wu：厂商把上面那组经 roll/pitch 旋到水平系之后的结果。
    /// 留着是为了诊断与对照，不要拿它当 body 系的量用。
    std::array<double, 3> accel_horizontal{};
    std::array<double, 3> gyro_horizontal{};
    int satellites = 0;
    double position_accuracy = 0.0;
    int navstat = 0;
};

OxtsRecord parseOxts(const fs::path& path) {
    const auto lines = readLines(path);
    if (lines.empty()) throw KittiError("oxts 文件是空的: " + path.string());
    const auto n = parseNumbers(lines.front());
    if (n.size() < 26) {
        throw KittiError("oxts 一行至少 26 个字段，实得 " + std::to_string(n.size()) + ": "
                         + path.string());
    }
    OxtsRecord record;
    record.lat = n[0];
    record.lon = n[1];
    record.alt = n[2];
    record.roll = n[3];
    record.pitch = n[4];
    record.yaw = n[5];
    // oxts 一行的字段序（KITTI 原始 README）：
    //   11..13 ax ay az   车体系比力
    //   14..16 af al au   经 roll/pitch 旋到水平系之后的比力
    //   17..19 wx wy wz   车体系角速度
    //   20..22 wf wl wu   水平系角速度
    // body = IMU，所以取车体系那一组。
    record.accel_body = {n[11], n[12], n[13]};
    record.accel_horizontal = {n[14], n[15], n[16]};
    record.gyro_body = {n[17], n[18], n[19]};
    record.gyro_horizontal = {n[20], n[21], n[22]};
    record.position_accuracy = n[23];
    record.navstat = static_cast<int>(n[25]);
    record.satellites = n.size() > 26 ? static_cast<int>(n[26]) : 0;
    return record;
}

std::pair<std::size_t, std::size_t> applyRange(std::size_t total, std::size_t skip,
                                               std::size_t limit) {
    const std::size_t begin = std::min(skip, total);
    const std::size_t end = limit == 0 ? total : std::min(total, begin + limit);
    return {begin, end};
}

}  // namespace

// ─────────────────────────────── odometry ───────────────────────────────

struct OdometrySource::Impl {
    OdometryOptions options;
    fs::path sequence;
    std::vector<fs::path> scans;
    std::vector<fw::Timestamp> times;
    fw::CalibrationData calibration;
    fw::Trajectory ground_truth;
    bool has_ground_truth = false;
    std::size_t first = 0;
    std::size_t cursor = 0;
    std::size_t last = 0;
};

OdometrySource::OdometrySource(const std::string& sequence_path, OdometryOptions options)
    : impl_(std::make_unique<Impl>()) {
    impl_->options = std::move(options);
    impl_->sequence = fs::path(sequence_path);
    impl_->scans = sortedFiles(impl_->sequence / "velodyne", ".bin");

    // 时刻只用数据集自带的 times.txt，必须逐帧对齐。缺它、行数不符都说明数据集不完整，
    // 拿标称周期编出来的时间掩盖过去，只会让 dt 与运动补偿悄悄失真。
    const fs::path times_file = impl_->sequence / "times.txt";
    if (!fs::exists(times_file)) {
        throw KittiError("缺 times.txt：KITTI 里程计的时刻在官方标定包（data_odometry_calib.zip）"
                         "里，和 calib.txt 在一起: " + times_file.string());
    }
    std::size_t row = 0;
    for (const auto& line : readLines(times_file)) {
        ++row;
        if (line.find_first_not_of(" \t") == std::string::npos) continue;  // 空白行
        std::vector<double> numbers;
        try {
            numbers = parseNumbers(line);
        } catch (const KittiError& e) {
            throw KittiError("times.txt 第 " + std::to_string(row) + " 行: " + e.what());
        }
        if (numbers.size() != 1) {
            throw KittiError("times.txt 第 " + std::to_string(row) + " 行应当恰好是一个数: "
                             + times_file.string());
        }
        const auto stamp = secondsToTimestamp(numbers.front());
        if (!impl_->times.empty() && stamp <= impl_->times.back()) {
            // 时刻必须严格递增：不递增的时间戳会让 dt 变成零或负，
            // 后面的运动补偿与插值全部失去意义
            throw KittiError("times.txt 第 " + std::to_string(row) + " 行时刻没有递增: "
                             + times_file.string());
        }
        impl_->times.push_back(stamp);
    }
    if (impl_->times.size() != impl_->scans.size()) {
        throw KittiError("times.txt 行数 (" + std::to_string(impl_->times.size())
                         + ") 与点云帧数 (" + std::to_string(impl_->scans.size())
                         + ") 不符: " + times_file.string());
    }

    const auto [begin, end] = applyRange(impl_->scans.size(), impl_->options.skip_scans,
                                         impl_->options.max_scans);
    impl_->first = begin;
    impl_->cursor = begin;
    impl_->last = end;

    // ── 标定 ──
    // odometry 序列没有 IMU，body 就取激光自身。此时激光的外参是一个**已知的**
    // 恒等变换（body 的定义所致），而不是"未知外参被猜成单位阵"，所以来源是
    // kDataset 不是 kAssumed——后者会让下游以为这里缺一份真正的标定。
    impl_->calibration.device = "kitti/odometry/" + impl_->sequence.filename().string();
    fw::Extrinsic lidar;
    lidar.source = fw::CalibrationSource::kDataset;
    impl_->calibration.extrinsics[impl_->options.lidar_name] = lidar;

    fw::LidarIntrinsics lidar_intrinsics;
    lidar_intrinsics.rings = 64;
    lidar_intrinsics.rate_hz =
        impl_->options.scan_period.count() > 0
            ? 1e9 / static_cast<double>(impl_->options.scan_period.count())
            : 10.0;
    impl_->calibration.lidars[impl_->options.lidar_name] = lidar_intrinsics;

    SE3d T_cam0_velo = SE3d::Identity();
    bool has_tr = false;
    const fs::path calib_file = impl_->sequence / "calib.txt";
    if (fs::exists(calib_file)) {
        for (const auto& line : readLines(calib_file)) {
            const auto colon = line.find(':');
            if (colon == std::string::npos) continue;
            const std::string key = line.substr(0, colon);
            if (key != "Tr") continue;
            T_cam0_velo = poseFromRowMajor(parseNumbers(line.substr(colon + 1)),
                                           calib_file.string());
            has_tr = true;
        }
    }
    if (has_tr) {
        fw::Extrinsic camera;
        camera.T_body_sensor = T_cam0_velo.inverse();  // body = velo
        camera.source = fw::CalibrationSource::kDataset;
        impl_->calibration.extrinsics["cam0"] = camera;
    }

    // ── 真值 ──
    fs::path poses = impl_->options.poses_path.empty()
                         ? impl_->sequence.parent_path().parent_path() / "poses"
                               / (impl_->sequence.filename().string() + ".txt")
                         : fs::path(impl_->options.poses_path);
    if (fs::exists(poses)) {
        if (impl_->options.ground_truth_frame == GroundTruthFrame::kBody && !has_tr) {
            // 没有 Tr 就做不了相机系 → 激光系的转换。此时若照旧返回，
            // 拿到手的是**相机位姿**却挂着 body 的名字，误差里凭空多出一段杆臂。
            throw KittiError("缺少 calib.txt 的 Tr，无法把相机系真值转到 body 系: "
                             + poses.string() + "（要原始相机系请显式指定 kCamera）");
        }
        // 真值只覆盖**选中的扫描窗口**，世界原点重置到第一帧选中扫描——与 raw 读取器
        // 同一个政策。覆盖完整序列的话，限帧运行会在覆盖率里被当成"提前终止"；
        // 原点不重置的话，跳过开头几帧后不对齐的比较就差一个常量位姿。
        const auto lines = readLines(poses);
        std::size_t index = 0;
        std::optional<SE3d> origin;
        for (const auto& line : lines) {
            if (line.find_first_not_of(" \t") == std::string::npos) continue;
            if (index >= impl_->last) break;
            if (index < impl_->first) {
                ++index;
                continue;
            }
            SE3d pose = poseFromRowMajor(parseNumbers(line), poses.string());
            if (impl_->options.ground_truth_frame == GroundTruthFrame::kBody) {
                // poses/XX.txt 是左相机在首帧相机系下的位姿。
                // 激光（= body）系轨迹：T_velo0_veloi = Tr⁻¹ · T_cam0_cami · Tr
                pose = T_cam0_velo.inverse().compose(pose).compose(T_cam0_velo);
            }
            if (!origin) origin = pose;
            // 时刻与发出的包的参考时刻一致，因此评测不会差一个扫描周期
            impl_->ground_truth.push(impl_->times[index], origin->inverse().compose(pose));
            ++index;
        }
        impl_->has_ground_truth = impl_->ground_truth.size() > 0;
        impl_->ground_truth.setFrame(fw::TrajectoryFrame::kWorld);
        // 跟踪的传感器写明：body 系真值跟踪的是 body（= 激光），相机系真值跟踪的是 cam0
        impl_->ground_truth.setTracked(
            impl_->options.ground_truth_frame == GroundTruthFrame::kBody ? "body" : "cam0");
        impl_->ground_truth.setName(poses.string());
    }
}

OdometrySource::~OdometrySource() = default;
OdometrySource::OdometrySource(OdometrySource&&) noexcept = default;
OdometrySource& OdometrySource::operator=(OdometrySource&&) noexcept = default;

std::optional<fw::SensorDatum> OdometrySource::next() {
    if (impl_->cursor >= impl_->last) return std::nullopt;
    const std::size_t index = impl_->cursor++;

    auto scan = std::make_shared<fw::LidarScan>();
    scan->sensor = impl_->options.lidar_name;
    // times.txt 给的时刻当作本帧的参考时刻。覆盖区间按标称扫描周期往回推——
    // 这是一个声明的区间，不是测量出来的采集窗口。
    scan->reference_time = impl_->times[index];
    scan->coverage = fw::TimeRange{scan->reference_time - impl_->options.scan_period,
                                   scan->reference_time};
    scan->motion_compensation = impl_->options.motion_compensation;
    scan->points = readVelodyneBin(impl_->scans[index], impl_->options.derive_point_times,
                                   impl_->options.scan_period.count());
    return fw::SensorDatum{fw::Ptr<fw::LidarScan>(scan)};
}

std::string OdometrySource::name() const {
    return "kitti::OdometrySource(" + impl_->sequence.string() + ")";
}

fw::SourceDescriptor OdometrySource::descriptor() const {
    fw::SourceDescriptor descriptor;
    descriptor.declared = true;
    descriptor.name = name();
    descriptor.body_sensor = impl_->options.lidar_name;
    descriptor.motion_compensation = impl_->options.motion_compensation;
    descriptor.point_time_source = impl_->options.derive_point_times
                                       ? fw::PointTimeSource::kReconstructed
                                       : fw::PointTimeSource::kAbsent;
    // times.txt 只给一个时刻，它与扫描的哪一刻对应没有出处，不硬说成终点
    descriptor.reference_kind = fw::ReferenceKind::kDatasetProvided;
    descriptor.has_imu = false;
    descriptor.has_navigation = false;
    descriptor.has_calibration = true;
    descriptor.has_ground_truth = impl_->has_ground_truth;
    if (impl_->last > impl_->first) {
        descriptor.coverage = fw::TimeRange{impl_->times[impl_->first],
                                            impl_->times[impl_->last - 1]};
        // 每一帧的参考时刻都应当有位姿：目标窗口就是选中的首帧到末帧
        descriptor.evaluation_window = descriptor.coverage;
    }
    if (impl_->has_ground_truth) {
        descriptor.notes.push_back("真值只覆盖选中的扫描窗口，世界原点在第一帧选中扫描");
    }
    return descriptor;
}
std::size_t OdometrySource::frameCount() const { return impl_->last - impl_->first; }
const fw::CalibrationData& OdometrySource::calibration() const { return impl_->calibration; }
bool OdometrySource::hasGroundTruth() const { return impl_->has_ground_truth; }
const fw::Trajectory& OdometrySource::groundTruth() const { return impl_->ground_truth; }

fw::PacketizerPolicy OdometrySource::packetizerPolicy() const {
    fw::PacketizerPolicy policy;
    policy.trigger_on_lidar = true;
    policy.imu = fw::ImuRequirement::kNotRequired;  // odometry 序列没有 IMU
    return policy;
}

// ───────────────────────────────── raw ─────────────────────────────────

struct RawSource::Impl {
    RawOptions options;
    fs::path drive;
    std::vector<fs::path> scans;
    std::vector<fw::Timestamp> scan_reference;  // 扫描终点
    std::vector<fw::Timestamp> scan_start;
    std::vector<fs::path> oxts;
    std::vector<fw::Timestamp> oxts_times;
    fw::CalibrationData calibration;
    fw::Trajectory ground_truth;
    bool has_ground_truth = false;
    double imu_rate_hz = 0.0;        ///< 实测，不是标称
    fw::Duration typical_imu_gap{0}; ///< 实测的典型（中位数）间隔
    fw::Duration max_imu_gap{0};     ///< 实测的最大间隔：事实，不拿来当阈值
    fw::TimeRange scan_window{};     ///< 选中扫描的 [首帧起点, 末帧终点]
    fw::TimeRange evaluation_window{};  ///< 选中扫描的 [首帧参考时刻, 末帧参考时刻]
    std::vector<std::size_t> repaired_frames;  ///< 按标称周期修补过起点的扫描

    /// 合并后的事件序列：按时刻排好，next() 只管往下走。
    struct Event {
        fw::Timestamp time{0};
        bool is_scan = false;
        std::size_t index = 0;
    };
    std::vector<Event> events;
    std::size_t cursor = 0;
    bool gnss_pending = false;  ///< 同一条 oxts 先发 IMU 再发 GNSS
};

RawSource::RawSource(const std::string& drive_path, RawOptions options)
    : impl_(std::make_unique<Impl>()) {
    impl_->options = std::move(options);
    impl_->drive = fs::path(drive_path);

    const fs::path velodyne = impl_->drive / "velodyne_points";
    impl_->scans = sortedFiles(velodyne / "data", ".bin");
    impl_->scan_reference = readRawTimestamps(velodyne / "timestamps_end.txt");
    impl_->scan_start = readRawTimestamps(velodyne / "timestamps_start.txt");
    if (impl_->scan_reference.size() != impl_->scans.size()
        || impl_->scan_start.size() != impl_->scans.size()) {
        throw KittiError("velodyne 的时间戳行数与点云帧数不符: " + velodyne.string());
    }

    const fs::path oxts_dir = impl_->drive / "oxts";
    impl_->oxts = sortedFiles(oxts_dir / "data", ".txt");
    impl_->oxts_times = readRawTimestamps(oxts_dir / "timestamps.txt");
    if (impl_->oxts_times.size() != impl_->oxts.size()) {
        throw KittiError("oxts 的时间戳行数与数据文件数不符: " + oxts_dir.string());
    }

    // 实测 oxts 的典型间隔与最大间隔。`_sync` 目录通常是按同步帧取的约 10 Hz 导航状态，
    // 不是 100 Hz 惯性流，所以阈值按实测给，不写死一个标称值。
    impl_->imu_rate_hz = measuredRateHz(impl_->oxts_times);
    impl_->typical_imu_gap = typicalGap(impl_->oxts_times);
    impl_->max_imu_gap = maxGap(impl_->oxts_times);

    const auto [begin, end] =
        applyRange(impl_->scans.size(), impl_->options.skip_scans, impl_->options.max_scans);

    // 起点不早于终点的扫描（反向或零长度的覆盖区间）：默认拒绝读取。修补是容错政策，
    // 不是校验——修补过的帧的覆盖区间与逐点时间都是推出来的——所以要显式打开，
    // 并把政策与受影响的帧写进来源描述。只查选中的窗口：窗口外的帧不影响这次运行。
    for (std::size_t i = begin; i < end; ++i) {
        if (impl_->scan_start[i] < impl_->scan_reference[i]) continue;
        if (!impl_->options.repair_scan_timing) {
            throw KittiError("第 " + std::to_string(i) + " 帧扫描的起点不早于终点: "
                             + velodyne.string()
                             + "（确认要按 100 ms 标称周期修补时打开 repair_scan_timing）");
        }
        impl_->scan_start[i] = impl_->scan_reference[i] - std::chrono::milliseconds(100);
        impl_->repaired_frames.push_back(i);
    }
    const fw::Timestamp scan_begin =
        begin < impl_->scan_start.size() ? impl_->scan_start[begin] : fw::Timestamp{0};
    const fw::Timestamp scan_end = end > 0 && end - 1 < impl_->scan_reference.size()
                                       ? impl_->scan_reference[end - 1]
                                       : fw::Timestamp{0};

    // 扫描选择范围与惯性括点分开裁剪：第一帧起点之前留一条左括点（最后一条不晚于
    // 起点的样本），最后一帧终点之后留一条右括点（第一条不早于终点的样本）。
    // 端点插值要的是括点，不是"往外多留一段固定时长"——相位一变，固定时长就可能
    // 恰好把括点裁掉。
    const auto& oxts = impl_->oxts_times;
    std::size_t lo = 0;
    while (lo + 1 < oxts.size() && oxts[lo + 1] <= scan_begin) ++lo;
    std::size_t hi = 0;
    while (hi < oxts.size() && oxts[hi] < scan_end + impl_->options.post_support) ++hi;
    if (hi >= oxts.size() && !oxts.empty()) hi = oxts.size() - 1;
    for (std::size_t i = lo; !oxts.empty() && i <= hi; ++i) {
        impl_->events.push_back(Impl::Event{oxts[i], false, i});
    }
    impl_->scan_window = fw::TimeRange{scan_begin, scan_end};
    if (end > begin) {
        impl_->evaluation_window =
            fw::TimeRange{impl_->scan_reference[begin], impl_->scan_reference[end - 1]};
    }
    for (std::size_t i = begin; i < end; ++i) {
        impl_->events.push_back(Impl::Event{impl_->scan_reference[i], true, i});
    }
    std::stable_sort(impl_->events.begin(), impl_->events.end(),
                     [](const Impl::Event& a, const Impl::Event& b) { return a.time < b.time; });

    // ── 标定：body = IMU ──
    impl_->calibration.device = "kitti/raw/" + impl_->drive.filename().string();
    const fs::path date_dir = impl_->drive.parent_path();

    const fs::path imu_to_velo = date_dir / "calib_imu_to_velo.txt";
    fw::Extrinsic lidar;
    if (fs::exists(imu_to_velo)) {
        const SE3d T_velo_imu = poseFromRT(readLines(imu_to_velo), imu_to_velo);
        lidar.T_body_sensor = T_velo_imu.inverse();  // T_imu_velo
        lidar.source = fw::CalibrationSource::kDataset;
    }
    // 缺文件时这条外参保持"未知"（取不出来），读取本身不失败：
    // 外部标定文件可以补上它。补没补上由装配核对（SourceDescriptor::required_extrinsics）——
    // body 是 IMU，激光在车顶另一处，猜一个单位阵等于把杆臂抹掉。
    impl_->calibration.extrinsics[impl_->options.lidar_name] = lidar;

    fw::Extrinsic imu;  // body 自己，恒等且精确
    imu.source = fw::CalibrationSource::kDataset;
    impl_->calibration.extrinsics["imu"] = imu;

    const fs::path velo_to_cam = date_dir / "calib_velo_to_cam.txt";
    if (fs::exists(velo_to_cam) && lidar.source == fw::CalibrationSource::kDataset) {
        const SE3d T_cam_velo = poseFromRT(readLines(velo_to_cam), velo_to_cam);
        fw::Extrinsic camera;
        camera.T_body_sensor = lidar.T_body_sensor.compose(T_cam_velo.inverse());
        camera.source = fw::CalibrationSource::kDataset;
        // calib_velo_to_cam 给的是**未经 rectify** 的 cam0；rectified 图像还要乘 R_rect_00，
        // 所以名字写明是未经 rectify 的
        impl_->calibration.extrinsics["cam0_unrectified"] = camera;
    }

    fw::LidarIntrinsics lidar_intrinsics;
    lidar_intrinsics.rings = 64;
    lidar_intrinsics.rate_hz = 10.0;
    impl_->calibration.lidars[impl_->options.lidar_name] = lidar_intrinsics;
    impl_->calibration.imu.rate_hz = impl_->imu_rate_hz;  // 实测值，不是标称值

    // ── 真值：相对第一帧选中扫描的 body 轨迹 ──
    //
    // 与 odometry 读取器同一个政策：只覆盖选中的扫描窗口（带左右括点，终点插值要用），
    // 世界原点重置到第一帧选中扫描的终点时刻（在那里插值）。`T_origin⁻¹ · T_i` 把原点
    // 处的姿态也一并消掉了，剩下的是相对原点的位姿，不是 ENU。真值留在 OXTS 的原始
    // 时刻上；与扫描终点不同步，评测要用插值关联，不要当成同一时刻。
    if (!impl_->oxts.empty() && hi >= lo) {
        Mercator mercator;
        mercator.scale = std::cos(parseOxts(impl_->oxts[lo]).lat * kPi / 180.0);
        fw::Trajectory absolute;
        for (std::size_t i = lo; i <= hi; ++i) {
            const OxtsRecord record = parseOxts(impl_->oxts[i]);
            absolute.push(oxts[i], SE3d(mercator.project(record.lat, record.lon, record.alt),
                                        rpyToQuaternion(record.roll, record.pitch, record.yaw)));
        }
        const fw::Timestamp origin_time =
            begin < impl_->scan_reference.size() ? impl_->scan_reference[begin] : oxts[lo];
        const SE3d origin = absolute.interpolate(origin_time).value_or(absolute.at(0).pose);
        for (const auto& sample : absolute) {
            impl_->ground_truth.push(sample.time, origin.inverse().compose(sample.pose));
        }
        impl_->has_ground_truth = impl_->ground_truth.size() > 0;
        impl_->ground_truth.setFrame(fw::TrajectoryFrame::kWorld);
        impl_->ground_truth.setTracked("body");
        impl_->ground_truth.setName(oxts_dir.string() + "(相对第一帧选中扫描的 body)");
    }
}

RawSource::~RawSource() = default;
RawSource::RawSource(RawSource&&) noexcept = default;
RawSource& RawSource::operator=(RawSource&&) noexcept = default;

std::optional<fw::SensorDatum> RawSource::next() {
    if (impl_->cursor >= impl_->events.size()) return std::nullopt;
    const auto& event = impl_->events[impl_->cursor];

    if (event.is_scan) {
        ++impl_->cursor;
        auto scan = std::make_shared<fw::LidarScan>();
        scan->sensor = impl_->options.lidar_name;
        scan->reference_time = impl_->scan_reference[event.index];
        scan->coverage = fw::TimeRange{impl_->scan_start[event.index], scan->reference_time};
        const std::int64_t period =
            (scan->coverage.end - scan->coverage.begin).count() > 0
                ? (scan->coverage.end - scan->coverage.begin).count()
                : std::chrono::milliseconds(100).count() * 1000000LL;
        scan->points =
            readVelodyneBin(impl_->scans[event.index], impl_->options.derive_point_times, period);
        scan->motion_compensation = fw::MotionCompensation::kRaw;  // raw 保留扫描畸变
        return fw::SensorDatum{fw::Ptr<fw::LidarScan>(scan)};
    }

    const OxtsRecord record = parseOxts(impl_->oxts[event.index]);
    if (impl_->gnss_pending) {
        impl_->gnss_pending = false;
        ++impl_->cursor;
        auto fix = std::make_shared<fw::GnssFix>();
        fix->time = event.time;
        fix->ecef = geodeticToEcef(record.lat, record.lon, record.alt);
        const double sigma = std::max(record.position_accuracy, 0.01);
        fix->covariance = {sigma * sigma, 0.0, 0.0, 0.0, sigma * sigma, 0.0,
                           0.0, 0.0, sigma * sigma};
        // navstat 的取值表 KITTI 没有公布，所以不据它断言"这是差分解"。
        // posacc 复制成各向同性方差也只是近似：原值是水平位置精度，不是 ECEF 协方差。
        fix->kind = fw::GnssFixKind::kSingle;
        fix->satellites = static_cast<std::uint8_t>(std::min(record.satellites, 255));
        return fw::SensorDatum{fw::Ptr<fw::GnssFix>(fix)};
    }

    // 同一条 oxts 先发 IMU；要 GNSS 就把游标留在原地，下一次调用发 GNSS
    if (impl_->options.emit_gnss) {
        impl_->gnss_pending = true;
    } else {
        ++impl_->cursor;
    }
    auto sample = std::make_shared<fw::ImuSample>();
    sample->time = event.time;
    // ax/ay/az 与 wx/wy/wz 是**车体系**的量；af/al/au、wf/wl/wu 是厂商把它们
    // 经 roll/pitch 旋到水平系之后的结果。body = IMU 要的是前者，
    // 不能因为"前、左、上"三个字就把后者当成等价物。
    sample->accel = record.accel_body;
    sample->gyro = record.gyro_body;
    return fw::SensorDatum{fw::Ptr<fw::ImuSample>(sample)};
}

std::string RawSource::name() const {
    return "kitti::RawSource(" + impl_->drive.string() + ")";
}

fw::SourceDescriptor RawSource::descriptor() const {
    fw::SourceDescriptor descriptor;
    descriptor.declared = true;
    descriptor.name = name();
    descriptor.body_sensor = "imu";
    descriptor.motion_compensation = fw::MotionCompensation::kRaw;  // raw 保留扫描畸变
    descriptor.point_time_source = impl_->options.derive_point_times
                                       ? fw::PointTimeSource::kReconstructed
                                       : fw::PointTimeSource::kAbsent;
    descriptor.reference_kind = fw::ReferenceKind::kScanEnd;  // 读的就是 timestamps_end
    descriptor.has_imu = !impl_->oxts.empty();
    descriptor.has_navigation = !impl_->oxts.empty();
    descriptor.imu_rate_hz = impl_->imu_rate_hz;
    descriptor.max_imu_gap = impl_->max_imu_gap;
    descriptor.has_calibration = true;
    descriptor.has_ground_truth = impl_->has_ground_truth;
    descriptor.coverage = impl_->scan_window;
    descriptor.evaluation_window = impl_->evaluation_window;
    if (!impl_->repaired_frames.empty()) {
        descriptor.timing_repair = "scan_start = scan_end - 100ms";
        descriptor.repaired_frames = impl_->repaired_frames;
    }
    // body 是 IMU：激光相对 body 的外参必须已知，由数据集或外部标定文件提供
    descriptor.required_extrinsics = {impl_->options.lidar_name};
    descriptor.notes.push_back("真值取自 OXTS 融合导航解，与 OXTS 导出的 GNSS 同源");
    descriptor.notes.push_back(
        "OXTS 的 navstat 取值表未公布：GNSS 定位类型一律按 single 缺省解释，"
        "位置协方差由 posacc 近似为各向同性");
    descriptor.notes.push_back("真值在 OXTS 原始时刻，扫描参考在终点：评测应使用插值关联");
    if (!impl_->repaired_frames.empty()) {
        descriptor.notes.push_back(std::to_string(impl_->repaired_frames.size())
                                   + " 帧扫描起点不早于终点，按显式选项以 100 ms 标称周期修补");
    }
    return descriptor;
}

double RawSource::imuRateHz() const { return impl_->imu_rate_hz; }
fw::Duration RawSource::maxImuGap() const { return impl_->max_imu_gap; }
std::size_t RawSource::frameCount() const {
    std::size_t scans = 0;
    for (const auto& event : impl_->events) {
        if (event.is_scan) ++scans;
    }
    return scans;
}
std::size_t RawSource::imuSampleCount() const {
    std::size_t samples = 0;
    for (const auto& event : impl_->events) {
        if (!event.is_scan) ++samples;
    }
    return samples;
}
const fw::CalibrationData& RawSource::calibration() const { return impl_->calibration; }
bool RawSource::hasGroundTruth() const { return impl_->has_ground_truth; }
const fw::Trajectory& RawSource::groundTruth() const { return impl_->ground_truth; }

fw::PacketizerPolicy RawSource::packetizerPolicy() const {
    fw::PacketizerPolicy policy;
    policy.trigger_on_lidar = true;
    policy.imu = fw::ImuRequirement::kRequired;
    // 端点插值只要一条右括点：终点之后第一条样本一到就能发包，不需要额外等。
    // 时间偏置余量是算法的选择，推荐值不替它做（留零）。把实测间隔当余量的话，
    // 约 10 Hz 的 `_sync` 导航状态下每个触发帧都要等过下一帧。
    policy.post_support = fw::Duration{0};
    // 左括点由回收逻辑自己保住（总留一条不晚于回收点的样本），不需要额外的历史
    policy.pre_support = fw::Duration{0};
    // 连续性阈值按这条流**典型**的间隔给：比中位数宽一半。异常长的间隔如实标缺口——
    // 按实测最大间隔给的话，数据里出现过的每一个缺口都会被默认接受。算法自己的
    // 连续性上限另外声明（OdometryBase::maxImuGap），不由这里代劳。
    policy.max_imu_gap = impl_->typical_imu_gap + impl_->typical_imu_gap / 2;
    return policy;
}

}  // namespace simpleslam::plugin::kitti
