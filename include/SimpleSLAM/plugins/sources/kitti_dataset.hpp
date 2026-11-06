// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file kitti_dataset.hpp
/// KITTI 的两种数据集读取器：odometry 基准序列与 raw 采集。
///
/// 它们把 `OdometrySource` / `RawSource` 连同自带的标定、真值与推荐的打包策略一起交出去。
/// 支持多序列：给了 `sequence` 时，`path` 是数据集根目录，读取器自己拼出那个序列的目录。

#include <SimpleSLAM/plugins/dataset.hpp>

#include <cstdint>
#include <string>
#include <string_view>

namespace simpleslam::plugin::kitti {

class OdometryReader final : public DatasetReader {
public:
    static constexpr std::string_view kTypeName = "kitti_odometry";

    struct Config {
        std::string path;
        std::string sequence;
        std::string groundtruth_frame = "body";
        std::int64_t skip_scans = 0;
        std::int64_t max_scans = 0;

        template <class V>
        void visit(V& v) {
            v("path", path, "序列目录；给了 sequence 时是含 sequences/ 的数据集根目录");
            v("sequence", sequence, "序列号（如 \"00\"）；空表示 path 就是序列目录");
            v("groundtruth_frame", groundtruth_frame,
              "真值取 body 系还是相机系：body / camera（poses 原生在相机系）");
            v("skip_scans", skip_scans, "跳过前 N 帧");
            v("max_scans", max_scans, "最多读 N 帧（0 = 全部）");
        }
    };

    /// 路径为空、坐标系拼错、帧数为负时抛 `std::invalid_argument`。
    explicit OdometryReader(const Config& config);

    [[nodiscard]] std::string name() const override;

private:
    [[nodiscard]] Dataset doOpen(const DatasetRequest& request) const override;

    Config config_;
};

class RawReader final : public DatasetReader {
public:
    static constexpr std::string_view kTypeName = "kitti_raw";

    struct Config {
        std::string path;
        std::string sequence;
        std::int64_t skip_scans = 0;
        std::int64_t max_scans = 0;
        bool repair_scan_timing = false;

        template <class V>
        void visit(V& v) {
            v("path", path, "采集目录（…_drive_XXXX_sync）；给了 sequence 时是它的上一级");
            v("sequence", sequence, "采集目录名；空表示 path 就是采集目录");
            v("skip_scans", skip_scans, "跳过前 N 帧");
            v("max_scans", max_scans, "最多读 N 帧（0 = 全部）");
            v("repair_scan_timing", repair_scan_timing,
              "按 100 ms 标称周期修补起点不早于终点的扫描（默认拒绝读取）");
        }
    };

    /// 路径为空、帧数为负时抛 `std::invalid_argument`。
    explicit RawReader(const Config& config);

    [[nodiscard]] std::string name() const override;

private:
    [[nodiscard]] Dataset doOpen(const DatasetRequest& request) const override;

    Config config_;
};

}  // namespace simpleslam::plugin::kitti
