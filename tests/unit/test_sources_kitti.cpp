// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// KITTI 读取器：就地造一份最小的数据集，验证那几个容易栽跟头的地方——
/// 逐点时间由方位角反推、真值与包的参考时刻天然对齐、odometry 的真值要从
/// 相机系换到 body 系、raw 的 body 是 IMU 且外参来自 calib_imu_to_velo。

#include <SimpleSLAM/plugins/sources/kitti.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace simpleslam;
using Catch::Approx;
namespace fw = simpleslam::fw;
namespace kitti = simpleslam::plugin::kitti;
namespace fs = std::filesystem;

namespace {

/// 一次性目录，析构即删。测试不该在 /tmp 留垃圾。
class TempDir {
public:
    explicit TempDir(const char* tag) {
        path_ = fs::temp_directory_path()
                / ("simpleslam_" + std::string(tag) + std::to_string(::getpid()));
        fs::remove_all(path_);
        fs::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ignored;
        fs::remove_all(path_, ignored);
    }
    [[nodiscard]] const fs::path& path() const { return path_; }

private:
    fs::path path_;
};

void writeText(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream file(path);
    file << text;
}

/// raw 时间戳的文本格式：`2011-09-26 13:02:SS.nnnnnnnnn`。
std::string rawStamp(double seconds) {
    const int whole = static_cast<int>(seconds);
    const long long nanos = static_cast<long long>((seconds - whole) * 1e9 + 0.5);
    std::ostringstream out;
    out << "2011-09-26 13:02:" << std::setw(2) << std::setfill('0') << whole << '.'
        << std::setw(9) << std::setfill('0') << nanos;
    return out.str();
}

/// 改写一个逐行文本文件的第 index 行。
void replaceLine(const fs::path& path, std::size_t index, const std::string& text) {
    std::ifstream in(path);
    std::vector<std::string> lines;
    for (std::string line; std::getline(in, line);) lines.push_back(line);
    in.close();
    REQUIRE(index < lines.size());
    lines[index] = text;
    std::ostringstream out;
    for (const auto& line : lines) out << line << '\n';
    writeText(path, out.str());
}

/// 写一帧 velodyne .bin：四个点，方位角分别在 0°、90°、180°、270°。
void writeScan(const fs::path& path) {
    fs::create_directories(path.parent_path());
    const std::vector<std::array<float, 4>> points = {
        {1.0F, 0.0F, 0.0F, 0.5F},   // yaw = 0
        {0.0F, 1.0F, 0.0F, 0.6F},   // yaw = +90°
        {-1.0F, 0.0F, 0.0F, 0.7F},  // yaw = 180°
        {0.0F, -1.0F, 0.0F, 0.8F},  // yaw = -90°
    };
    std::ofstream file(path, std::ios::binary);
    for (const auto& point : points) {
        file.write(reinterpret_cast<const char*>(point.data()), sizeof(point));
    }
}

std::string rowMajor(const Eigen::Matrix3d& rotation, const Vec3d& translation) {
    std::ostringstream out;
    out.precision(12);
    for (int row = 0; row < 3; ++row) {
        out << rotation(row, 0) << ' ' << rotation(row, 1) << ' ' << rotation(row, 2) << ' '
            << translation(row) << (row == 2 ? "" : " ");
    }
    return out.str();
}

/// 造一个 odometry 序列：<root>/sequences/00 与 <root>/poses/00.txt。
fs::path makeOdometryDataset(const TempDir& dir, int frames, const Eigen::Matrix3d& tr_rotation) {
    const fs::path sequence = dir.path() / "sequences" / "00";
    std::ostringstream times;
    std::ostringstream poses;
    times.precision(9);
    poses.precision(12);
    for (int i = 0; i < frames; ++i) {
        writeScan(sequence / "velodyne" / (std::string(6 - std::to_string(i).size(), '0')
                                           + std::to_string(i) + ".bin"));
        times << std::fixed << (0.1 * i) << '\n';
        // 相机系真值：沿相机 x 轴每帧前进 1 米
        poses << rowMajor(Eigen::Matrix3d::Identity(), Vec3d(i, 0.0, 0.0)) << '\n';
    }
    writeText(sequence / "times.txt", times.str());
    writeText(dir.path() / "poses" / "00.txt", poses.str());
    writeText(sequence / "calib.txt",
              "P0: 1 0 0 0 0 1 0 0 0 0 1 0\n"
              "Tr: " + rowMajor(tr_rotation, Vec3d(0.0, 0.0, 0.0)) + "\n");
    return sequence;
}

/// 造一个 raw drive：<date>/<drive>_sync/ 加日期目录下的标定文件。
fs::path makeRawDataset(const TempDir& dir, int frames, int imu_per_frame) {
    const fs::path date = dir.path() / "2011_09_26";
    const fs::path drive = date / "2011_09_26_drive_0001_sync";

    auto stamp = [](double seconds) {
        const int whole = static_cast<int>(seconds);
        const long long nanos = static_cast<long long>((seconds - whole) * 1e9 + 0.5);
        std::ostringstream out;
        out << "2011-09-26 13:02:" << std::setw(2) << std::setfill('0') << whole << '.'
            << std::setw(9) << std::setfill('0') << nanos;
        return out.str();
    };

    std::ostringstream starts;
    std::ostringstream ends;
    std::ostringstream mids;
    for (int i = 0; i < frames; ++i) {
        writeScan(drive / "velodyne_points" / "data"
                  / (std::string(10 - std::to_string(i).size(), '0') + std::to_string(i)
                     + ".bin"));
        starts << stamp(10.0 + 0.1 * i) << '\n';
        mids << stamp(10.05 + 0.1 * i) << '\n';
        ends << stamp(10.1 + 0.1 * i) << '\n';
    }
    writeText(drive / "velodyne_points" / "timestamps_start.txt", starts.str());
    writeText(drive / "velodyne_points" / "timestamps.txt", mids.str());
    writeText(drive / "velodyne_points" / "timestamps_end.txt", ends.str());

    const int imu_count = frames * imu_per_frame + imu_per_frame;
    std::ostringstream imu_times;
    for (int i = 0; i < imu_count; ++i) {
        const double t = 10.0 + static_cast<double>(i) / (10.0 * imu_per_frame);
        imu_times << stamp(t) << '\n';
        std::ostringstream row;
        row.precision(12);
        // lat lon alt roll pitch yaw vn ve vf vl vu ax ay az af al au wx wy wz wf wl wu
        // pos_acc vel_acc navstat numsats posmode velmode orimode
        //
        // 车体系与水平系两组**故意取不同的值**：读错组的话测试会当场失败，
        // 而不是因为"前左上"三个字看起来合理就蒙混过去。
        row << std::fixed << (49.0 + 1e-5 * i) << ' ' << 8.4 << ' ' << 110.0 << " 0 0 0 "
            << "0 0 0 0 0  0.1 0.2 9.80  0.3 0.4 9.70  0.01 0.02 0.03  0.04 0.05 0.06"
               "  0.05 0.05 4 8 1 1 1\n";
        writeText(drive / "oxts" / "data"
                      / (std::string(10 - std::to_string(i).size(), '0') + std::to_string(i)
                         + ".txt"),
                  row.str());
    }
    writeText(drive / "oxts" / "timestamps.txt", imu_times.str());

    // T_velo_imu：激光在 IMU 前方 0.8 米
    writeText(date / "calib_imu_to_velo.txt",
              "calib_time: 25-May-2012\nR: 1 0 0 0 1 0 0 0 1\nT: -0.8 0 0\n");
    return drive;
}

}  // namespace

TEST_CASE("odometry：基准扫描已补偿过，默认不再反推逐点时间", "[sources][kitti]") {
    // 对补偿过的扫描再补一次，等于把同一段自车运动扣了两遍。
    // 默认不给逐点时间，也就没有人会顺手再补——补偿状态由描述声明，
    // 不从"有没有时间列"反推。
    TempDir dir("kitti_odo_compensated");
    const auto sequence = makeOdometryDataset(dir, 2, Eigen::Matrix3d::Identity());

    kitti::OdometrySource source(sequence.string());
    const auto descriptor = source.descriptor();
    CHECK(descriptor.motion_compensation == fw::MotionCompensation::kCompensated);
    CHECK(descriptor.point_time_source == fw::PointTimeSource::kAbsent);
    // times.txt 只给一个时刻，它对应扫描的哪一刻没有出处
    CHECK(descriptor.reference_kind == fw::ReferenceKind::kDatasetProvided);

    auto datum = source.next();
    REQUIRE(datum.has_value());
    const auto& scan = std::get<fw::Ptr<fw::LidarScan>>(*datum);
    CHECK(scan->points.time_offset_ns.empty());
}

TEST_CASE("odometry：显式打开时逐点时间由方位角反推，相对扫描终点为负",
          "[sources][kitti]") {
    TempDir dir("kitti_odo_times");
    const auto sequence = makeOdometryDataset(dir, 3, Eigen::Matrix3d::Identity());

    kitti::OdometryOptions options;
    options.derive_point_times = true;
    options.motion_compensation = fw::MotionCompensation::kRaw;
    kitti::OdometrySource source(sequence.string(), options);
    CHECK(source.frameCount() == 3);
    // 反推出来的是重建值，不是测量：相位、切帧位置、旋向都是假设
    CHECK(source.descriptor().point_time_source == fw::PointTimeSource::kReconstructed);

    auto datum = source.next();
    REQUIRE(datum.has_value());
    const auto& scan = std::get<fw::Ptr<fw::LidarScan>>(*datum);
    REQUIRE(scan->points.size() == 4);
    REQUIRE(scan->points.time_offset_ns.size() == 4);
    for (const auto offset : scan->points.time_offset_ns) {
        CHECK(offset <= 0);                    // 参考时刻取扫描终点
        CHECK(offset >= -100'000'000);         // 不超出一个扫描周期
    }
    // yaw=0 的点在半程，yaw=180° 的点在两端；四个点不应当挤在一起
    CHECK(scan->points.time_offset_ns[0] != scan->points.time_offset_ns[1]);
    CHECK(scan->points.intensity[3] == Approx(0.8F));
}

TEST_CASE("odometry：真值与包的参考时刻逐帧对齐", "[sources][kitti]") {
    TempDir dir("kitti_odo_align");
    const auto sequence = makeOdometryDataset(dir, 4, Eigen::Matrix3d::Identity());

    kitti::OdometrySource source(sequence.string());
    REQUIRE(source.hasGroundTruth());
    const auto& truth = source.groundTruth();
    REQUIRE(truth.size() == 4);

    for (std::size_t i = 0; i < 4; ++i) {
        auto datum = source.next();
        REQUIRE(datum.has_value());
        const auto& scan = std::get<fw::Ptr<fw::LidarScan>>(*datum);
        // 差一个扫描周期在 KITTI 的速度下就是米级误差，所以这条必须严格相等
        CHECK(scan->reference_time == truth.at(i).time);
        CHECK(scan->coverage.end == scan->reference_time);
        CHECK((scan->reference_time - scan->coverage.begin).count() == 100'000'000);
    }
    CHECK_FALSE(source.next().has_value());
}

TEST_CASE("odometry：真值默认换到 body 系", "[sources][kitti]") {
    TempDir dir("kitti_odo_frame");
    // Tr = T_cam0_velo 绕 z 转 +90°
    const Eigen::Matrix3d tr =
        Eigen::AngleAxisd(M_PI / 2.0, Vec3d::UnitZ()).toRotationMatrix();
    const auto sequence = makeOdometryDataset(dir, 3, tr);

    kitti::OdometrySource body(sequence.string());
    REQUIRE(body.hasGroundTruth());
    // 相机系里沿 +x 前进；body(velo) 系里应当是 Rz(-90°)·(i,0,0) = (0,-i,0)
    CHECK(body.groundTruth().at(2).pose.translation().x() == Approx(0.0).margin(1e-9));
    CHECK(body.groundTruth().at(2).pose.translation().y() == Approx(-2.0).margin(1e-9));

    kitti::OdometryOptions camera_options;
    camera_options.ground_truth_frame = kitti::GroundTruthFrame::kCamera;
    kitti::OdometrySource camera(sequence.string(), camera_options);
    CHECK(camera.groundTruth().at(2).pose.translation().x() == Approx(2.0));
}

TEST_CASE("odometry：没有 IMU，打包策略就不要求 IMU", "[sources][kitti]") {
    TempDir dir("kitti_odo_policy");
    const auto sequence = makeOdometryDataset(dir, 2, Eigen::Matrix3d::Identity());
    kitti::OdometrySource source(sequence.string());
    CHECK(source.packetizerPolicy().imu == fw::ImuRequirement::kNotRequired);
    // body 就是激光，所以它的外参是一个**已知的**恒等变换，不是"猜了个单位阵"
    CHECK(source.calibration().extrinsic("velodyne")->source == fw::CalibrationSource::kDataset);
    CHECK(source.calibration().T_body_sensor("velodyne").has_value());
    // calib.txt 里有 Tr，所以相机外参是数据集给的
    REQUIRE(source.calibration().extrinsic("cam0") != nullptr);
    CHECK(source.calibration().extrinsic("cam0")->source == fw::CalibrationSource::kDataset);
}

TEST_CASE("odometry：跳帧与限帧", "[sources][kitti]") {
    TempDir dir("kitti_odo_range");
    const auto sequence = makeOdometryDataset(dir, 10, Eigen::Matrix3d::Identity());
    kitti::OdometryOptions options;
    options.skip_scans = 3;
    options.max_scans = 4;
    kitti::OdometrySource source(sequence.string(), options);
    CHECK(source.frameCount() == 4);

    auto first = source.next();
    REQUIRE(first.has_value());
    CHECK(std::get<fw::Ptr<fw::LidarScan>>(*first)->reference_time
          == fw::Timestamp{300'000'000});
    std::size_t seen = 1;
    while (source.next()) ++seen;
    CHECK(seen == 4);
}

TEST_CASE("odometry：times.txt 行数不符是错误，不用合成时间掩盖", "[sources][kitti]") {
    TempDir dir("kitti_odo_bad");
    const auto sequence = makeOdometryDataset(dir, 3, Eigen::Matrix3d::Identity());
    writeText(sequence / "times.txt", "0.0\n0.1\n");  // 少一行
    CHECK_THROWS_AS(kitti::OdometrySource(sequence.string()), kitti::KittiError);
}

TEST_CASE("raw：IMU 与激光按时刻交织，body 是 IMU", "[sources][kitti]") {
    TempDir dir("kitti_raw");
    const auto drive = makeRawDataset(dir, 3, 10);

    kitti::RawSource source(drive.string());
    CHECK(source.frameCount() == 3);
    CHECK(source.imuSampleCount() > 0);
    CHECK(source.packetizerPolicy().imu == fw::ImuRequirement::kRequired);

    // body = IMU：激光外参是 T_imu_velo = (T_velo_imu)⁻¹，即 +0.8 米
    const auto T_body_lidar = source.calibration().T_body_sensor("velodyne");
    REQUIRE(T_body_lidar.has_value());
    CHECK(T_body_lidar->translation().x() == Approx(0.8));
    CHECK(source.calibration().extrinsic("velodyne")->source == fw::CalibrationSource::kDataset);
    CHECK(source.calibration().extrinsic("imu")->T_body_sensor.translation().norm()
          == Approx(0.0));

    std::size_t scans = 0;
    std::size_t imu = 0;
    fw::Timestamp previous{0};
    while (auto datum = source.next()) {
        fw::Timestamp now{0};
        if (const auto* scan = std::get_if<fw::Ptr<fw::LidarScan>>(&*datum)) {
            now = (*scan)->reference_time;
            ++scans;
        } else if (const auto* sample = std::get_if<fw::Ptr<fw::ImuSample>>(&*datum)) {
            now = (*sample)->time;
            // 取的是**车体系**那一组（ax/ay/az、wx/wy/wz）。
            // af/al/au 是厂商旋到水平系之后的量，不能因为"前左上"就当成等价物。
            CHECK((*sample)->accel[2] == Approx(9.80));  // az，不是 au(9.70)
            CHECK((*sample)->gyro[2] == Approx(0.03));   // wz，不是 wu(0.06)
            ++imu;
        }
        CHECK(now >= previous);  // 合并后必须按时刻单调
        previous = now;
    }
    CHECK(scans == 3);
    CHECK(imu > 0);
}

TEST_CASE("raw：真值原点在第一帧选中扫描，只覆盖选中窗口", "[sources][kitti]") {
    // 与 odometry 读取器同一个政策：原点在第一帧选中扫描的终点时刻（插值得到），
    // 真值只覆盖选中的扫描窗口加左右括点，不外扩一段固定时长
    TempDir dir("kitti_raw_gt");
    const auto drive = makeRawDataset(dir, 3, 5);
    kitti::RawOptions options;
    options.skip_scans = 1;
    kitti::RawSource source(drive.string(), options);
    REQUIRE(source.hasGroundTruth());
    const auto& truth = source.groundTruth();
    REQUIRE(truth.size() > 1);
    CHECK(truth.tracked() == "body");

    // 第一帧选中扫描的终点就是原点：在那里插值得到的是恒等位姿
    std::optional<fw::Timestamp> first_scan_end;
    std::optional<fw::Timestamp> first_scan_start;
    while (auto datum = source.next()) {
        if (const auto* scan = std::get_if<fw::Ptr<fw::LidarScan>>(&*datum)) {
            first_scan_end = (*scan)->reference_time;
            first_scan_start = (*scan)->coverage.begin;
            break;
        }
    }
    REQUIRE(first_scan_end.has_value());
    const auto origin = truth.interpolate(*first_scan_end);
    REQUIRE(origin.has_value());
    CHECK(origin->translation().norm() == Approx(0.0).margin(1e-6));
    // 窗口只多留一条左括点：第一条真值不晚于选中扫描的起点，第二条已经晚于它
    CHECK(truth.at(0).time <= *first_scan_start);
    CHECK(truth.at(1).time > *first_scan_start);
}

TEST_CASE("raw：末帧所需的尾部支撑不被裁掉", "[sources][kitti]") {
    // 目标扫描范围与所需惯性支撑范围要分开裁剪。只裁到最后一个扫描终点的话，
    // 末帧必然缺右侧支撑而被标缺口。
    TempDir dir("kitti_raw_tail");
    const auto drive = makeRawDataset(dir, 3, 10);
    kitti::RawSource source(drive.string());

    const auto policy = source.packetizerPolicy();
    fw::Timestamp last_scan{0};
    fw::Timestamp last_imu{0};
    while (auto datum = source.next()) {
        if (const auto* scan = std::get_if<fw::Ptr<fw::LidarScan>>(&*datum)) {
            last_scan = (*scan)->reference_time;
        } else if (const auto* sample = std::get_if<fw::Ptr<fw::ImuSample>>(&*datum)) {
            last_imu = (*sample)->time;
        }
    }
    CHECK(last_imu >= last_scan + policy.post_support);
}

TEST_CASE("raw：惯性采样率按实测报告", "[sources][kitti]") {
    // `_sync` 目录里通常是按同步帧取的约 10 Hz 导航状态，不是 100 Hz 惯性流。
    // 能力由实测值说明，不由类名或标称值说明。
    TempDir dir("kitti_raw_rate");
    const auto drive = makeRawDataset(dir, 4, 10);  // 10 Hz 扫描 × 10 条 = 100 Hz
    kitti::RawSource source(drive.string());

    CHECK(source.imuRateHz() == Approx(100.0).margin(5.0));
    CHECK(source.maxImuGap() <= std::chrono::milliseconds(11));
    const auto descriptor = source.descriptor();
    CHECK(descriptor.has_imu);
    CHECK(descriptor.imu_rate_hz == Approx(source.imuRateHz()));
    CHECK(descriptor.motion_compensation == fw::MotionCompensation::kRaw);
    CHECK(descriptor.reference_kind == fw::ReferenceKind::kScanEnd);
    // 推荐策略只等右括点，不把实测间隔当等待余量；连续性要求按实测间隔放宽
    const auto policy = source.packetizerPolicy();
    CHECK(policy.post_support == fw::Duration{0});
    CHECK(policy.max_imu_gap >= source.maxImuGap());
}

TEST_CASE("raw：扫描起点不早于终点时默认拒绝读取，显式修补才放行并记下受影响的帧",
          "[sources][kitti]") {
    TempDir dir("kitti_raw_reversed");
    const auto drive = makeRawDataset(dir, 3, 10);
    // 第 2 帧的起点推到终点之后（起点文件仍严格递增）：反向的覆盖区间
    replaceLine(drive / "velodyne_points" / "timestamps_start.txt", 2, rawStamp(10.35));
    CHECK_THROWS_AS(kitti::RawSource(drive.string()), kitti::KittiError);

    kitti::RawOptions options;
    options.repair_scan_timing = true;
    kitti::RawSource source(drive.string(), options);
    const auto descriptor = source.descriptor();
    CHECK_FALSE(descriptor.timing_repair.empty());
    CHECK(descriptor.repaired_frames == std::vector<std::size_t>{2});
}

TEST_CASE("raw：连续性阈值按典型间隔给，数据里的长缺口如实报告而不是被默认接受",
          "[sources][kitti]") {
    TempDir dir("kitti_raw_gap");
    const auto drive = makeRawDataset(dir, 4, 10);  // 100 Hz、共 50 条 oxts
    // 第 20 条之后整体推后 50 ms：制造一个 60 ms 的缺口
    std::ostringstream stamps;
    for (int i = 0; i < 50; ++i) {
        stamps << rawStamp(10.0 + i / 100.0 + (i >= 20 ? 0.05 : 0.0)) << '\n';
    }
    writeText(drive / "oxts" / "timestamps.txt", stamps.str());

    kitti::RawSource source(drive.string());
    const auto policy = source.packetizerPolicy();
    // 典型间隔 10 ms 放宽一半：长缺口会被如实标出，而不是因为"数据里出现过"就被接受
    CHECK(policy.max_imu_gap >= std::chrono::milliseconds(14));
    CHECK(policy.max_imu_gap <= std::chrono::milliseconds(16));
    CHECK(source.maxImuGap() >= std::chrono::milliseconds(59));  // 事实照报
    CHECK(source.maxImuGap() <= std::chrono::milliseconds(61));
    CHECK(source.descriptor().max_imu_gap == source.maxImuGap());
    CHECK(source.imuRateHz() == Approx(100.0).margin(1.0));  // 频率按典型间隔，不被缺口带偏
}

TEST_CASE("odometry：缺 Tr 就不给 body 系真值", "[sources][kitti]") {
    // 没有 Tr 做不了相机系 → 激光系的转换。照旧返回的话，
    // 拿到手的是相机位姿却挂着 body 的名字，误差里凭空多出一段杆臂。
    TempDir dir("kitti_odo_no_tr");
    const auto sequence = makeOdometryDataset(dir, 3, Eigen::Matrix3d::Identity());
    writeText(sequence / "calib.txt", "P0: 1 0 0 0 0 1 0 0 0 0 1 0\n");  // 没有 Tr

    CHECK_THROWS_AS(kitti::OdometrySource(sequence.string()), kitti::KittiError);

    kitti::OdometryOptions camera;
    camera.ground_truth_frame = kitti::GroundTruthFrame::kCamera;
    CHECK_NOTHROW(kitti::OdometrySource(sequence.string(), camera));
}

TEST_CASE("raw：缺 IMU→激光外参时外参取不出来，由装配核对", "[sources][kitti]") {
    // body 是 IMU，激光在车顶另一处。猜一个单位阵等于把杆臂抹掉——但读取本身不失败：
    // 外部标定文件可以补上它。补没补上由装配核对（SlamSystem::init）。
    TempDir dir("kitti_raw_no_calib");
    const auto drive = makeRawDataset(dir, 2, 5);
    std::filesystem::remove(drive.parent_path() / "calib_imu_to_velo.txt");

    kitti::RawSource strict(drive.string());
    CHECK_FALSE(strict.calibration().T_body_sensor("velodyne").has_value());
    const auto descriptor = strict.descriptor();
    REQUIRE(descriptor.required_extrinsics.size() == 1);
    CHECK(descriptor.required_extrinsics.front() == "velodyne");
    CHECK(fw::missingExtrinsics(strict.calibration(), descriptor.required_extrinsics).size() == 1);
}

TEST_CASE("odometry：坏的时间行不被静默吞掉", "[sources][kitti]") {
    TempDir dir("kitti_odo_bad_times");
    const auto sequence = makeOdometryDataset(dir, 3, Eigen::Matrix3d::Identity());
    writeText(sequence / "times.txt", "0.0\nabc\n0.2\n");
    CHECK_THROWS_AS(kitti::OdometrySource(sequence.string()), kitti::KittiError);

    writeText(sequence / "times.txt", "0.0\n0.0\n0.2\n");  // 时刻没有递增
    CHECK_THROWS_AS(kitti::OdometrySource(sequence.string()), kitti::KittiError);
}
