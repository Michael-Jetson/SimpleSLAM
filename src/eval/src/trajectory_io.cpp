// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/eval/trajectory_io.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <format>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string_view>

namespace simpleslam::eval {

namespace {

fw::Timestamp secondsToTimestamp(double seconds) {
    return fw::Timestamp{static_cast<std::int64_t>(std::llround(seconds * 1e9))};
}

double timestampToSeconds(fw::Timestamp time) {
    return static_cast<double>(time.count()) * 1e-9;
}

bool isBlankOrComment(const std::string& line) {
    for (const char c : line) {
        if (std::isspace(static_cast<unsigned char>(c)) != 0) continue;
        return c == '#';
    }
    return true;
}

std::vector<double> parseNumbers(const std::string& line) {
    std::istringstream stream(line);
    std::vector<double> numbers;
    double value = 0.0;
    while (stream >> value) numbers.push_back(value);
    return numbers;
}

/// 旋转的容差：文本文件里的小数位有限，四舍五入会让单位四元数、正交矩阵差一点点；
/// 超出这个量就不是舍入，是数据本身不对。
constexpr double kRotationTolerance = 1e-3;

bool allFinite(const std::vector<double>& numbers, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(numbers[i])) return false;
    }
    return true;
}

std::vector<double> readTimesFile(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw TrajectoryIoError("打不开 KITTI 时刻文件: " + path);
    std::vector<double> times;
    std::string line;
    while (std::getline(file, line)) {
        if (isBlankOrComment(line)) continue;
        const auto numbers = parseNumbers(line);
        if (numbers.empty()) continue;
        if (!std::isfinite(numbers.front())) {
            throw TrajectoryIoError("KITTI 时刻文件里有非有限值: " + path);
        }
        times.push_back(numbers.front());
    }
    return times;
}

constexpr std::string_view kSemanticsPrefix = "# simpleslam ";

std::string_view frameName(fw::TrajectoryFrame frame) {
    switch (frame) {
        case fw::TrajectoryFrame::kWorld: return "world";
        case fw::TrajectoryFrame::kOdom: return "odom";
        case fw::TrajectoryFrame::kUnknown: return "unknown";
    }
    return "unknown";
}

/// 读回写轨迹时记下的语义：`# simpleslam frame=odom tracked=body clock=0`。evo 等工具把它
/// 当注释跳过；认不出的键与值报错，免得一条写坏的语义被悄悄当成"未声明"。
void readSemantics(const std::string& line, Trajectory& trajectory, const std::string& where) {
    std::istringstream stream(line.substr(kSemanticsPrefix.size()));
    std::string token;
    while (stream >> token) {
        const auto equals = token.find('=');
        if (equals == std::string::npos) throw TrajectoryIoError(where + " 语义行写坏了: " + token);
        const std::string key = token.substr(0, equals);
        const std::string value = token.substr(equals + 1);
        if (key == "frame") {
            if (value == "world") {
                trajectory.setFrame(fw::TrajectoryFrame::kWorld);
            } else if (value == "odom") {
                trajectory.setFrame(fw::TrajectoryFrame::kOdom);
            } else if (value == "unknown") {
                trajectory.setFrame(fw::TrajectoryFrame::kUnknown);
            } else {
                throw TrajectoryIoError(where + " 认不出的参考系: " + value);
            }
        } else if (key == "tracked") {
            trajectory.setTracked(value);
        } else if (key == "clock") {
            std::uint32_t clock = 0;
            const char* last = value.data() + value.size();
            const auto [end, error] = std::from_chars(value.data(), last, clock);
            if (error != std::errc{} || end != last) {
                throw TrajectoryIoError(where + " 时钟域要是非负整数: " + value);
            }
            trajectory.setClock(fw::ClockDomainId{clock});
        } else {
            throw TrajectoryIoError(where + " 语义行里有未知键: " + key);
        }
    }
}

Trajectory readTum(std::istream& input, const std::string& path) {
    Trajectory trajectory;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.starts_with(kSemanticsPrefix)) {
            readSemantics(line, trajectory, path + ":" + std::to_string(line_number));
            continue;
        }
        if (isBlankOrComment(line)) continue;
        const auto n = parseNumbers(line);
        if (n.size() < 8) {
            throw TrajectoryIoError(path + ":" + std::to_string(line_number)
                                    + " TUM 每行需要 8 个数（时间 + 平移 + 四元数 xyzw）");
        }
        if (!allFinite(n, 8)) {
            throw TrajectoryIoError(path + ":" + std::to_string(line_number) + " 含非有限值");
        }
        const Eigen::Quaterniond q(n[7], n[4], n[5], n[6]);  // (w, x, y, z)
        // 只容忍舍入：归一化一个长度是 2 的四元数，等于替坏数据编一个"看起来对"的旋转
        if (std::abs(q.norm() - 1.0) > kRotationTolerance) {
            // 模长要按最短往返表示打印：to_string 只留 6 位小数，1.0000004 会打成 1.000000
            throw TrajectoryIoError(std::format("{}:{} 四元数不是单位四元数（模长 {}）", path,
                                                line_number, q.norm()));
        }
        trajectory.push(secondsToTimestamp(n[0]),
                        SE3d(Vec3d(n[1], n[2], n[3]), q.normalized()));
    }
    return trajectory;
}

Trajectory readKitti(std::istream& input, const std::string& path,
                     const TrajectoryIoOptions& options) {
    // KITTI 格式不带时刻：时刻只取数据集给的时刻文件，不拿行号或标称周期编
    if (options.kitti_times_path.empty()) {
        throw TrajectoryIoError(path + ": KITTI 格式不带时刻，要给时刻文件"
                                       "（如 KITTI 里程计序列的 times.txt）");
    }
    const std::vector<double> times = readTimesFile(options.kitti_times_path);

    Trajectory trajectory;
    std::string line;
    std::size_t index = 0;
    while (std::getline(input, line)) {
        if (isBlankOrComment(line)) continue;
        const auto n = parseNumbers(line);
        if (n.size() < 12) {
            throw TrajectoryIoError(path + ":" + std::to_string(index + 1)
                                    + " KITTI 每行需要 12 个数（行主序 3×4）");
        }
        if (!allFinite(n, 12)) {
            throw TrajectoryIoError(path + ":" + std::to_string(index + 1) + " 含非有限值");
        }
        Eigen::Matrix3d rotation;
        rotation << n[0], n[1], n[2], n[4], n[5], n[6], n[8], n[9], n[10];
        // 只容忍舍入：不正交、带缩放或是反射的矩阵不是旋转，转成四元数再归一化会把
        // 坏数据悄悄变成一个合法的姿态
        const double orthogonality =
            (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
        if (orthogonality > kRotationTolerance || rotation.determinant() <= 0.0) {
            throw TrajectoryIoError(path + ":" + std::to_string(index + 1)
                                    + " 旋转部分不是旋转矩阵（不正交或行列式不为正）");
        }
        const Vec3d translation(n[3], n[7], n[11]);

        if (index >= times.size()) {
            throw TrajectoryIoError("KITTI 时刻文件比位姿少: " + options.kitti_times_path);
        }
        const double seconds = times[index];
        trajectory.push(secondsToTimestamp(seconds),
                        SE3d(translation, Eigen::Quaterniond(rotation).normalized()));
        ++index;
    }
    return trajectory;
}

}  // namespace

std::string toString(TrajectoryFormat format) {
    return format == TrajectoryFormat::kTum ? "tum" : "kitti";
}

std::optional<TrajectoryFormat> parseTrajectoryFormat(std::string_view text) {
    if (text == "tum" || text == "TUM") return TrajectoryFormat::kTum;
    if (text == "kitti" || text == "KITTI") return TrajectoryFormat::kKitti;
    return std::nullopt;
}

TrajectoryFormat detectTrajectoryFormat(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw TrajectoryIoError("打不开轨迹文件: " + path);
    std::string line;
    while (std::getline(file, line)) {
        if (isBlankOrComment(line)) continue;
        const auto n = parseNumbers(line);
        if (n.size() >= 12) return TrajectoryFormat::kKitti;
        if (n.size() >= 8) return TrajectoryFormat::kTum;
        throw TrajectoryIoError("无法判定轨迹格式（每行既不是 8 列也不是 12 列）: " + path);
    }
    throw TrajectoryIoError("轨迹文件是空的: " + path);
}

Trajectory readTrajectory(const std::string& path, TrajectoryFormat format,
                          const TrajectoryIoOptions& options) {
    std::ifstream file(path);
    if (!file) throw TrajectoryIoError("打不开轨迹文件: " + path);
    Trajectory trajectory = format == TrajectoryFormat::kTum
                                ? readTum(file, path)
                                : readKitti(file, path, options);
    trajectory.setName(path);
    return trajectory;
}

void writeTrajectory(const std::string& path, const Trajectory& trajectory,
                     TrajectoryFormat format, const TrajectoryIoOptions& options) {
    std::ofstream file(path);
    if (!file) throw TrajectoryIoError("打不开输出文件: " + path);
    file << std::fixed << std::setprecision(options.precision);

    if (format == TrajectoryFormat::kTum) {
        // TUM 只有数，参考系、跟踪的传感器与时钟域写在注释里，读回来轨迹的语义不丢
        file << kSemanticsPrefix << "frame=" << frameName(trajectory.frame())
             << " tracked=" << trajectory.tracked() << " clock=" << trajectory.clock().value
             << '\n';
        file << "# timestamp tx ty tz qx qy qz qw\n";
        for (const auto& sample : trajectory) {
            const Eigen::Quaterniond q = sample.pose.quat().normalized();
            const Vec3d t = sample.pose.translation();
            file << timestampToSeconds(sample.time) << ' ' << t.x() << ' ' << t.y() << ' ' << t.z()
                 << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
        }
    } else {
        for (const auto& sample : trajectory) {
            const Eigen::Matrix3d r = sample.pose.rotation();
            const Vec3d t = sample.pose.translation();
            for (int row = 0; row < 3; ++row) {
                file << r(row, 0) << ' ' << r(row, 1) << ' ' << r(row, 2) << ' ' << t(row);
                file << (row == 2 ? '\n' : ' ');
            }
        }
    }
    if (!file) throw TrajectoryIoError("写轨迹文件失败: " + path);
}

}  // namespace simpleslam::eval
