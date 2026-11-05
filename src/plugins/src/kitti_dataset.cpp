// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/plugins/sources/kitti.hpp>
#include <SimpleSLAM/plugins/sources/kitti_dataset.hpp>

#include <filesystem>
#include <format>
#include <memory>
#include <stdexcept>
#include <utility>

namespace simpleslam::plugin::kitti {

namespace {

void requireCommon(std::string_view type, const std::string& path, std::int64_t skip_scans,
                   std::int64_t max_scans) {
    if (path.empty()) throw std::invalid_argument(std::format("{}.path 不能为空", type));
    if (skip_scans < 0 || max_scans < 0) {
        throw std::invalid_argument(std::format("{} 的 skip_scans、max_scans 不能为负", type));
    }
}

std::string joined(const std::string& root, const std::string& sub, const char* middle) {
    if (sub.empty()) return root;
    auto path = std::filesystem::path(root);
    if (middle != nullptr) path /= middle;
    return (path / sub).string();
}

}  // namespace

OdometryReader::OdometryReader(const Config& config) : config_(config) {
    requireCommon(kTypeName, config_.path, config_.skip_scans, config_.max_scans);
    if (config_.groundtruth_frame != "body" && config_.groundtruth_frame != "camera") {
        throw std::invalid_argument(std::format(
            "kitti_odometry.groundtruth_frame 只能是 body / camera，得到 {}",
            config_.groundtruth_frame));
    }
}

std::string OdometryReader::name() const {
    return std::format("kitti_odometry({})", joined(config_.path, config_.sequence, "sequences"));
}

Dataset OdometryReader::doOpen(const DatasetRequest& request) const {
    OdometryOptions options;
    options.skip_scans = static_cast<std::size_t>(config_.skip_scans);
    options.max_scans = static_cast<std::size_t>(config_.max_scans);
    // 读取器换算 body 系真值用的是数据集自己的 Tr。外部标定会取代它，两者可能对不上：
    // 有外部标定时给原生的相机系真值，由评测按生效的外参换算
    const bool native = config_.groundtruth_frame == "camera" || request.external_calibration;
    options.ground_truth_frame = native ? GroundTruthFrame::kCamera : GroundTruthFrame::kBody;
    auto source = std::make_unique<OdometrySource>(
        joined(config_.path, config_.sequence, "sequences"), options);

    Dataset dataset;
    dataset.calibration = source->calibration();
    if (source->hasGroundTruth()) dataset.ground_truth = source->groundTruth();
    dataset.recommended_policy = source->packetizerPolicy();
    dataset.frames = source->frameCount();
    dataset.evaluate_at_ground_truth_sensor = config_.groundtruth_frame == "camera";
    dataset.source = std::move(source);
    return dataset;
}

RawReader::RawReader(const Config& config) : config_(config) {
    requireCommon(kTypeName, config_.path, config_.skip_scans, config_.max_scans);
}

std::string RawReader::name() const {
    return std::format("kitti_raw({})", joined(config_.path, config_.sequence, nullptr));
}

Dataset RawReader::doOpen(const DatasetRequest& request) const {
    RawOptions options;
    options.skip_scans = static_cast<std::size_t>(config_.skip_scans);
    options.max_scans = static_cast<std::size_t>(config_.max_scans);
    // 时间异常的帧默认拒绝读取；修补必须显式打开，修补了哪些帧进有效身份
    options.repair_scan_timing = config_.repair_scan_timing;
    // 惯性流的右括点要与实际生效的打包策略一致，否则末帧等不到
    options.post_support = request.post_support;
    auto source =
        std::make_unique<RawSource>(joined(config_.path, config_.sequence, nullptr), options);

    Dataset dataset;
    dataset.calibration = source->calibration();
    if (source->hasGroundTruth()) dataset.ground_truth = source->groundTruth();
    dataset.recommended_policy = source->packetizerPolicy();
    dataset.frames = source->frameCount();
    dataset.source = std::move(source);
    return dataset;
}

}  // namespace simpleslam::plugin::kitti
