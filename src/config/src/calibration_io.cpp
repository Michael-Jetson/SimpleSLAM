// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/config/calibration_io.hpp>
#include <SimpleSLAM/config/loader.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>

namespace simpleslam::config {

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string formatDouble(double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
    return buffer;
}

void requireKnownKeys(const YAML::Node& node, const std::set<std::string>& known,
                      const std::string& where, const std::string& origin) {
    if (!node || !node.IsMap()) return;
    for (const auto& item : node) {
        const std::string key = item.first.as<std::string>();
        if (known.count(key) == 0) {
            throw CalibrationError(origin + ": " + where + " 里有未知键 " + key);
        }
    }
}

std::vector<double> numbers(const YAML::Node& node, std::size_t expected,
                            const std::string& where, const std::string& origin) {
    if (!node.IsSequence() || node.size() != expected) {
        throw CalibrationError(origin + ": " + where + " 需要 " + std::to_string(expected)
                               + " 个数");
    }
    std::vector<double> out;
    out.reserve(expected);
    for (const auto& item : node) out.push_back(item.as<double>());
    return out;
}

/// 规范化导出里的字符串一律加引号并转义。
///
/// 不加的话，一个含 `#`、冒号或换行的设备名导出之后就变成了别的东西，往返不再等价：
/// 任务记录里存的 `calibration.yaml` 就不是这次生效的那份了。
std::string quoted(const std::string& value) {
    std::string out = "\"";
    for (const char c : value) {
        if (c == '"' || c == '\\') out.push_back('\\');
        if (c == '\n') {
            out += "\\n";
            continue;
        }
        out.push_back(c);
    }
    out.push_back('"');
    return out;
}

fw::PerturbationSide parseSide(const std::string& text, const std::string& origin) {
    if (text == "right_body") return fw::PerturbationSide::kRightBody;
    if (text == "left_world") return fw::PerturbationSide::kLeftWorld;
    throw CalibrationError(origin + ": 未知的扰动侧 " + text);
}

fw::TangentOrder parseOrder(const std::string& text, const std::string& origin) {
    if (text == "translation_rotation") return fw::TangentOrder::kTranslationRotation;
    if (text == "rotation_translation") return fw::TangentOrder::kRotationTranslation;
    throw CalibrationError(origin + ": 未知的切空间分块顺序 " + text);
}

fw::CalibrationSource parseSource(const std::string& text, const std::string& origin) {
    if (text == "dataset") return fw::CalibrationSource::kDataset;
    if (text == "file") return fw::CalibrationSource::kFile;
    if (text == "online") return fw::CalibrationSource::kOnline;
    if (text == "assumed") return fw::CalibrationSource::kAssumed;
    if (text == "unset") return fw::CalibrationSource::kUnset;
    throw CalibrationError(origin + ": 未知的标定来源 " + text);
}

Eigen::Quaterniond rotationOf(const YAML::Node& node, const std::string& where,
                              const std::string& origin) {
    const bool has_quaternion = static_cast<bool>(node["quaternion"]);
    const bool has_rpy = static_cast<bool>(node["rpy_deg"]);
    const bool has_matrix = static_cast<bool>(node["matrix"]);
    const int given = static_cast<int>(has_quaternion) + static_cast<int>(has_rpy)
                      + static_cast<int>(has_matrix);
    if (given == 0) return Eigen::Quaterniond::Identity();
    if (given > 1) {
        throw CalibrationError(origin + ": " + where
                               + " 的旋转只能给一种（quaternion / rpy_deg / matrix）");
    }
    if (has_quaternion) {
        const auto q = numbers(node["quaternion"], 4, where + ".quaternion", origin);
        const Eigen::Quaterniond raw(q[0], q[1], q[2], q[3]);  // (w, x, y, z)
        // 归一化会把任何非零四元数变成一个"合法"旋转，包括写错的那些。
        // 范数离 1 太远就是写错了，不是精度问题。
        if (!std::isfinite(raw.norm()) || raw.norm() < 1e-6) {
            throw CalibrationError(origin + ": " + where + ".quaternion 的范数为零或非有限");
        }
        if (std::abs(raw.norm() - 1.0) > 1e-3) {
            throw CalibrationError(origin + ": " + where + ".quaternion 的范数不是 1");
        }
        return raw.normalized();
    }
    if (has_rpy) {
        const auto rpy = numbers(node["rpy_deg"], 3, where + ".rpy_deg", origin);
        const double roll = rpy[0] * kPi / 180.0;
        const double pitch = rpy[1] * kPi / 180.0;
        const double yaw = rpy[2] * kPi / 180.0;
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Vec3d::UnitZ())
                                  * Eigen::AngleAxisd(pitch, Vec3d::UnitY())
                                  * Eigen::AngleAxisd(roll, Vec3d::UnitX()))
            .normalized();
    }
    const auto m = numbers(node["matrix"], 12, where + ".matrix", origin);
    Eigen::Matrix3d rotation;
    rotation << m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10];
    // 先确认它确实是个旋转：正交且行列式为 +1。不查的话，一个带尺度或含反射的
    // 矩阵会被四元数化成一个看起来正常、实际错了的旋转。
    if (!rotation.allFinite()) {
        throw CalibrationError(origin + ": " + where + ".matrix 含非有限值");
    }
    if (!(rotation * rotation.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-4)) {
        throw CalibrationError(origin + ": " + where + ".matrix 的旋转部分不正交");
    }
    if (rotation.determinant() < 0.0) {
        throw CalibrationError(origin + ": " + where + ".matrix 的旋转部分含反射");
    }
    return Eigen::Quaterniond(rotation).normalized();
}

}  // namespace

fw::CalibrationData loadCalibrationString(const std::string& text, const std::string& origin) {
    YAML::Node root;
    try {
        root = YAML::Load(text);
    } catch (const YAML::Exception& e) {
        throw CalibrationError(origin + ": YAML 解析失败: " + e.what());
    }
    if (!root || root.IsNull()) return fw::CalibrationData{};
    if (!root.IsMap()) throw CalibrationError(origin + ": 标定文件的顶层必须是映射");
    requireKnownKeys(root, {"device", "revision", "imu", "extrinsics", "lidars"}, "顶层", origin);

    fw::CalibrationData calibration;
    if (root["device"]) calibration.device = root["device"].as<std::string>();
    // 文件里的 revision 只读不用：版本由提交入口生成，两份不同的标定都可以
    // 声称自己是 1。留着这个键是为了让人读文件时知道它的来历。
    calibration.revision = 0;

    if (const YAML::Node imu = root["imu"]) {
        requireKnownKeys(imu,
                         {"gyro_noise_density", "accel_noise_density", "gyro_bias_random_walk",
                          "accel_bias_random_walk", "rate_hz", "gravity"},
                         "imu", origin);
        auto& target = calibration.imu;
        if (imu["gyro_noise_density"]) target.gyro_noise_density = imu["gyro_noise_density"].as<double>();
        if (imu["accel_noise_density"]) target.accel_noise_density = imu["accel_noise_density"].as<double>();
        if (imu["gyro_bias_random_walk"]) target.gyro_bias_random_walk = imu["gyro_bias_random_walk"].as<double>();
        if (imu["accel_bias_random_walk"]) target.accel_bias_random_walk = imu["accel_bias_random_walk"].as<double>();
        if (imu["rate_hz"]) target.rate_hz = imu["rate_hz"].as<double>();
        if (imu["gravity"]) target.gravity = imu["gravity"].as<double>();
    }

    for (const auto& item : root["extrinsics"]) {
        const std::string sensor = item.first.as<std::string>();
        const YAML::Node& node = item.second;
        requireKnownKeys(node,
                         {"translation", "quaternion", "rpy_deg", "matrix", "source",
                          "covariance_diag", "covariance", "convention", "fixed"},
                         "extrinsics." + sensor, origin);
        // 一条外参必须真的写出几何量。空映射（`lidar: {}`）或只写来源的条目，
        // 默认补出来的就是一个单位阵——那等于把"没写"认证成"标定过且是单位阵"。
        // 真要单位阵，就显式写出来。
        const bool has_translation = static_cast<bool>(node["translation"]);
        const bool has_rotation = node["quaternion"] || node["rpy_deg"];
        if (!node["matrix"] && !(has_translation && has_rotation)) {
            throw CalibrationError(origin + ": extrinsics." + sensor
                                   + " 必须显式给出平移与旋转（translation + quaternion/rpy_deg，"
                                     "或 matrix）；单位阵也要写出来");
        }
        fw::Extrinsic extrinsic;
        Vec3d translation = Vec3d::Zero();
        if (node["translation"]) {
            const auto t = numbers(node["translation"], 3, "extrinsics." + sensor + ".translation",
                                   origin);
            translation = Vec3d(t[0], t[1], t[2]);
        } else if (node["matrix"]) {
            const auto m = numbers(node["matrix"], 12, "extrinsics." + sensor + ".matrix", origin);
            translation = Vec3d(m[3], m[7], m[11]);
        }
        extrinsic.T_body_sensor =
            SE3d(translation, rotationOf(node, "extrinsics." + sensor, origin));
        extrinsic.source = node["source"]
                               ? parseSource(node["source"].as<std::string>(), origin)
                               : fw::CalibrationSource::kFile;
        if (node["covariance_diag"] && node["covariance"]) {
            throw CalibrationError(origin + ": extrinsics." + sensor
                                   + " 的协方差只能给一种写法（covariance 或 covariance_diag）");
        }
        if (node["covariance_diag"]) {
            const auto diag = numbers(node["covariance_diag"], 6,
                                      "extrinsics." + sensor + ".covariance_diag", origin);
            for (int i = 0; i < 6; ++i) {
                extrinsic.covariance(i, i) = diag[static_cast<std::size_t>(i)];
            }
            extrinsic.covariance_known = true;
        }
        if (node["fixed"]) extrinsic.fixed = node["fixed"].as<bool>();
        if (node["covariance"]) {
            const YAML::Node& rows = node["covariance"];
            if (!rows.IsSequence() || rows.size() != 6) {
                throw CalibrationError(origin + ": extrinsics." + sensor
                                       + ".covariance 需要 6 行");
            }
            for (int r = 0; r < 6; ++r) {
                const auto row = numbers(rows[static_cast<std::size_t>(r)], 6,
                                         "extrinsics." + sensor + ".covariance", origin);
                for (int c = 0; c < 6; ++c) {
                    extrinsic.covariance(r, c) = row[static_cast<std::size_t>(c)];
                }
            }
            extrinsic.covariance_known = true;
        }
        if (const YAML::Node convention = node["convention"]) {
            requireKnownKeys(convention, {"side", "order"},
                             "extrinsics." + sensor + ".convention", origin);
            if (convention["side"]) {
                extrinsic.convention.side =
                    parseSide(convention["side"].as<std::string>(), origin);
            }
            if (convention["order"]) {
                extrinsic.convention.order =
                    parseOrder(convention["order"].as<std::string>(), origin);
            }
        }
        calibration.extrinsics[sensor] = extrinsic;
    }

    for (const auto& item : root["lidars"]) {
        const std::string sensor = item.first.as<std::string>();
        const YAML::Node& node = item.second;
        requireKnownKeys(node, {"min_range_m", "max_range_m", "rings", "rate_hz"},
                         "lidars." + sensor, origin);
        fw::LidarIntrinsics intrinsics;
        if (node["min_range_m"]) intrinsics.min_range_m = node["min_range_m"].as<double>();
        if (node["max_range_m"]) intrinsics.max_range_m = node["max_range_m"].as<double>();
        if (node["rings"]) intrinsics.rings = node["rings"].as<std::uint32_t>();
        if (node["rate_hz"]) intrinsics.rate_hz = node["rate_hz"].as<double>();
        calibration.lidars[sensor] = intrinsics;
    }
    if (const auto problem = fw::validateCalibration(calibration); !problem.empty()) {
        throw CalibrationError(origin + ": " + problem);
    }
    return calibration;
}

fw::CalibrationData loadCalibrationFile(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw CalibrationError("打不开标定文件: " + path);
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return loadCalibrationString(buffer.str(), path);
}

std::string dumpCalibration(const fw::CalibrationData& calibration) {
    // 格式表示不了的内容一律拒绝导出，不静默丢：任务记录里存的就是导出，
    // 补评换算外参时读它，丢了的东西谁也看不见
    if (const auto problem = fw::validateCalibration(calibration); !problem.empty()) {
        throw CalibrationError("标定不能无损导出：" + problem);
    }
    // 规范化：键序固定、旋转统一成四元数、数值格式固定，两份标定有什么不同直接比导出。
    //
    // 比的是**序列化内容**，不是几何等价类：q 与 −q 表示同一个旋转，导出却不同。
    // 要按几何等价判断"是不是同一副外参"，比较位姿本身。
    std::ostringstream out;
    out << "device: " << quoted(calibration.device) << '\n';
    out << "\nimu:\n";
    out << "  gyro_noise_density: " << formatDouble(calibration.imu.gyro_noise_density) << '\n';
    out << "  accel_noise_density: " << formatDouble(calibration.imu.accel_noise_density) << '\n';
    out << "  gyro_bias_random_walk: " << formatDouble(calibration.imu.gyro_bias_random_walk) << '\n';
    out << "  accel_bias_random_walk: " << formatDouble(calibration.imu.accel_bias_random_walk) << '\n';
    out << "  rate_hz: " << formatDouble(calibration.imu.rate_hz) << '\n';
    out << "  gravity: " << formatDouble(calibration.imu.gravity) << '\n';

    out << "\nextrinsics:\n";
    for (const auto& [sensor, extrinsic] : calibration.extrinsics) {
        const Vec3d t = extrinsic.T_body_sensor.translation();
        const Eigen::Quaterniond q = extrinsic.T_body_sensor.quat().normalized();
        out << "  " << quoted(sensor) << ":\n";
        out << "    translation: [" << formatDouble(t.x()) << ", " << formatDouble(t.y()) << ", "
            << formatDouble(t.z()) << "]\n";
        out << "    quaternion: [" << formatDouble(q.w()) << ", " << formatDouble(q.x()) << ", "
            << formatDouble(q.y()) << ", " << formatDouble(q.z()) << "]\n";
        out << "    source: " << fw::toString(extrinsic.source) << '\n';
        out << "    fixed: " << (extrinsic.fixed ? "true" : "false") << '\n';
        // 扰动约定必须导出：6×6 没有它就没有确定含义，丢了它往返就不是无损的
        out << "    convention:\n";
        out << "      side: " << fw::toString(extrinsic.convention.side) << '\n';
        out << "      order: " << fw::toString(extrinsic.convention.order) << '\n';
        // 按"是否已知"决定导不导，而不是按数值是否近零：1e-14 是一个很小的已知
        // 不确定度，省掉它往返之后就变成了"没有协方差"
        if (extrinsic.covariance_known) {
            // 整个 6×6 都要导，不能只导对角：非对角项是相关性，丢了它往返就不是无损的
            out << "    covariance:\n";
            for (int r = 0; r < 6; ++r) {
                out << "      - [";
                for (int c = 0; c < 6; ++c) {
                    out << formatDouble(extrinsic.covariance(r, c)) << (c == 5 ? "" : ", ");
                }
                out << "]\n";
            }
        }
    }

    if (!calibration.lidars.empty()) {
        out << "\nlidars:\n";
        for (const auto& [sensor, intrinsics] : calibration.lidars) {
            out << "  " << quoted(sensor) << ":\n";
            out << "    min_range_m: " << formatDouble(intrinsics.min_range_m) << '\n';
            out << "    max_range_m: " << formatDouble(intrinsics.max_range_m) << '\n';
            out << "    rings: " << intrinsics.rings << '\n';
            out << "    rate_hz: " << formatDouble(intrinsics.rate_hz) << '\n';
        }
    }
    return out.str();
}

void writeCalibrationFile(const std::string& path, const fw::CalibrationData& calibration) {
    std::ofstream file(path);
    if (!file) throw CalibrationError("打不开标定输出文件: " + path);
    file << dumpCalibration(calibration);
    if (!file) throw CalibrationError("写标定文件失败: " + path);
}


}  // namespace simpleslam::config
