// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file dataset_reader_contract.hpp
/// 数据集读取器（`plugin::DatasetReaderBase`）的契约测试模板。数据格式是读取器自己的，所以要给
/// 一份指向小样例数据的配置：
///
///     SIMPLESLAM_DATASET_READER_CONTRACT(MyReader, sampleConfig())
///
/// 配置要有字符串字段 `path`，模板把它改成不存在的路径来查报错。查的是读取器契约里写明的约定：
/// 能登记进目录、按类型名还原；一次给全，标定能通过校验，给了帧数就与实际的触发帧数相同；
/// 时刻有限、每条流里递增，读两次完全一样；路径不存在时抛异常，报错里写明路径；来源声明的
/// 时间范围盖住触发帧的参考时刻，有真值时与这个范围有交集。

#include <SimpleSLAM/config/plugin.hpp>
#include <SimpleSLAM/framework/calibration.hpp>
#include <SimpleSLAM/plugins/dataset.hpp>

#include <catch2/catch_test_macros.hpp>

#include <concepts>
#include <cstddef>
#include <exception>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

namespace simpleslam::testing {

/// 读取器的配置有字符串字段 `path`：模板要把它改成不存在的路径。
template <class R>
concept ReaderWithPath = requires(typename R::Config c) {
    { c.path } -> std::convertible_to<std::string>;
};

namespace detail {

/// 读出来的一条数据：种类、传感器、时刻与大小，够判断两次读得是否一样。
struct DatumRecord {
    int kind = 0;  ///< SensorDatum 的下标
    std::string sensor;
    fw::Timestamp time{0};
    std::size_t size = 0;  ///< 点数；IMU 与 GNSS 为 0
    float first_x = 0.0F;

    bool operator==(const DatumRecord&) const = default;
};

inline DatumRecord recordOf(const fw::SensorDatum& datum) {
    DatumRecord r;
    r.kind = static_cast<int>(datum.index());
    if (const auto* scan = std::get_if<fw::Ptr<fw::LidarScan>>(&datum)) {
        r.sensor = (*scan)->sensor;
        r.time = (*scan)->reference_time;
        r.size = (*scan)->points.size();
        if (r.size > 0) r.first_x = (*scan)->points.x.front();
    } else if (const auto* frame = std::get_if<fw::Ptr<fw::CameraFrame>>(&datum)) {
        r.time = (*frame)->capture_time;
    } else if (const auto* imu = std::get_if<fw::Ptr<fw::ImuSample>>(&datum)) {
        r.time = (*imu)->time;
    } else if (const auto* fix = std::get_if<fw::Ptr<fw::GnssFix>>(&datum)) {
        r.time = (*fix)->time;
    }
    return r;
}

inline std::vector<DatumRecord> drain(fw::ISensorSource& source) {
    std::vector<DatumRecord> out;
    while (auto datum = source.next()) out.push_back(recordOf(*datum));
    return out;
}

/// 触发帧：激光扫描与相机帧。
inline bool isTrigger(const DatumRecord& r) {
    return r.kind == static_cast<int>(fw::SensorDatum(fw::Ptr<fw::LidarScan>{}).index()) ||
           r.kind == static_cast<int>(fw::SensorDatum(fw::Ptr<fw::CameraFrame>{}).index());
}

}  // namespace detail

/// 数据集读取器契约。`config` 指向一小份样例数据，`request` 是任务给出的读取条件。
template <class R>
    requires config::PluginImplementation<R, plugin::DatasetReaderBase> && ReaderWithPath<R>
void checkDatasetReaderContract(const typename R::Config& config,
                                const plugin::DatasetRequest& request = {}) {
    const R reader(config);
    const plugin::DatasetReaderBase& r = reader;  // 经接口调用，和执行器里的用法一样
    const std::string name(R::kTypeName);

    SECTION("能登记进目录，按类型名还原、构造") {
        config::Catalogs catalogs;
        catalogs.of<plugin::DatasetReaderBase>().template add<R>();
        // 用这份配置写成的插件节点还原，和 YAML 里写了它一样
        const auto spec = config::Slot<plugin::DatasetReaderBase>::of<R>(config).resolved();
        const auto slot = catalogs.resolve<plugin::DatasetReaderBase>(spec, "data.source");
        CHECK(slot.type() == R::kTypeName);
        CHECK(slot.make() != nullptr);
    }

    SECTION("一次给全：有来源，标定能通过校验，给了帧数就与触发帧数相同") {
        plugin::Dataset dataset = r.open(request);
        REQUIRE(dataset.source != nullptr);
        INFO(fw::validateCalibration(dataset.calibration));
        CHECK(fw::validateCalibration(dataset.calibration).empty());
        const auto data = detail::drain(*dataset.source);
        CHECK_FALSE(data.empty());
        std::size_t triggers = 0;
        for (const auto& d : data) triggers += detail::isTrigger(d) ? 1 : 0;
        if (dataset.frames) CHECK(*dataset.frames == triggers);
    }

    SECTION("时刻有限、每条流里递增，读两次完全一样") {
        const auto first = detail::drain(*r.open(request).source);
        const auto second = detail::drain(*r.open(request).source);
        CHECK(first == second);
        std::map<std::pair<int, std::string>, fw::Timestamp> last;
        for (std::size_t i = 0; i < first.size(); ++i) {
            const auto key = std::make_pair(first[i].kind, first[i].sensor);
            if (const auto it = last.find(key); it != last.end()) {
                INFO("第 " << i << " 条数据");
                CHECK(first[i].time > it->second);
            }
            last[key] = first[i].time;
        }
    }

    SECTION("来源声明的时间范围盖住触发帧，有真值时与它有交集") {
        plugin::Dataset dataset = r.open(request);
        const fw::SourceDescriptor descriptor = dataset.source->descriptor();
        const auto data = detail::drain(*dataset.source);
        const auto& coverage = descriptor.coverage;
        if (coverage.end > coverage.begin) {
            for (const auto& d : data) {
                if (!detail::isTrigger(d)) continue;
                CHECK(d.time >= coverage.begin);
                CHECK(d.time <= coverage.end);
            }
            if (dataset.ground_truth && !dataset.ground_truth->empty()) {
                const auto& truth = *dataset.ground_truth;
                CHECK(truth.at(0).time <= coverage.end);
                CHECK(truth.at(truth.size() - 1).time >= coverage.begin);
            }
        }
    }

    SECTION("路径不存在：抛异常，报错里写明路径") {
        auto missing = config;
        missing.path = std::string(config.path) + "/不存在的目录";
        try {
            const R bad(missing);
            (void)bad.open(request);
            FAIL("路径不存在却没有报错");
        } catch (const std::exception& e) {
            CHECK(std::string(e.what()).find(missing.path) != std::string::npos);
        }
    }
}

}  // namespace simpleslam::testing

/// 给一个数据集读取器生成契约测试：`SIMPLESLAM_DATASET_READER_CONTRACT(MyReader, config)`，
/// `config` 指向一小份样例数据。
#define SIMPLESLAM_DATASET_READER_CONTRACT(Type, config)                           \
    TEST_CASE("数据集读取器契约：" #Type, "[contract][dataset]") {                \
        ::simpleslam::testing::checkDatasetReaderContract<Type>(config);           \
    }
