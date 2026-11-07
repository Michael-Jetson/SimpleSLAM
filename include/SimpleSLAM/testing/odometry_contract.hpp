// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file odometry_contract.hpp
/// 方案（里程计插件，`plugin::OdometryBase`）的契约测试模板。每个方案在自己的测试文件里一行
/// 实例化：
///
///     SIMPLESLAM_ODOMETRY_CONTRACT(MyLio)
///
/// 方案在合成来源（`SyntheticLidarImuSource`）上离线跑一个完整的系统。查的是框架对方案的
/// 约定，不查精度：能登记进目录、按类型名还原；每个送达的包都有终态，没有契约错误、没有算法
/// 失败、没有被隔离的异常；位姿有限，时刻是扫描的参考时刻；同样的输入跑两次轨迹逐位相同；
/// 退化帧（空扫描、极稀疏、共面）上同样不出错，可以降级或跟丢。精度用评测函数另看。

#include <SimpleSLAM/config/plugin.hpp>
#include <SimpleSLAM/modules/slam_system.hpp>
#include <SimpleSLAM/plugins/odometry.hpp>
#include <SimpleSLAM/testing/synthetic_source.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace simpleslam::testing {

namespace detail {

struct SyntheticRun {
    modules::RunOutcome outcome;
    fw::Trajectory odometry;
};

template <class S>
SyntheticRun runSynthetic(const typename S::Config& config, const SyntheticSequence& sequence) {
    modules::SlamSystemOptions options;
    options.offline = true;
    modules::SlamSystem system(options);
    system.setSource(std::make_unique<SyntheticLidarImuSource>(sequence));
    system.setOdometry<S>(config);
    system.init();
    system.runOffline();
    system.shutdown();
    return {system.runOutcome(), system.result().odometry};
}

inline bool sameTrajectory(const fw::Trajectory& a, const fw::Trajectory& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a.at(i).time != b.at(i).time) return false;
        if (a.at(i).pose.coeffs() != b.at(i).pose.coeffs()) return false;
    }
    return true;
}

/// 位姿有限，时刻都是某一帧扫描的参考时刻。
inline void checkPoses(const fw::Trajectory& trajectory, int scans) {
    std::vector<fw::Timestamp> times;
    for (int k = 0; k < scans; ++k) times.push_back(SyntheticLidarImuSource::scanTime(k));
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        INFO("第 " << i << " 个位姿");
        CHECK(trajectory.at(i).pose.coeffs().allFinite());
        CHECK(std::find(times.begin(), times.end(), trajectory.at(i).time) != times.end());
    }
}

}  // namespace detail

/// 方案契约。`config` 是被测方案的配置。
template <class S>
    requires config::PluginImplementation<S, plugin::OdometryBase>
void checkOdometryContract(const typename S::Config& config = {}) {
    SECTION("能登记进目录，按类型名还原、构造") {
        config::Catalogs catalogs;
        catalogs.of<plugin::OdometryBase>().template add<S>();
        const auto slot = catalogs.resolve<plugin::OdometryBase>(
            config::PluginSpec{.type = std::string(S::kTypeName), .params = {}, .children = {}},
            "system.modules.odometry.plugin");
        CHECK(slot.type() == S::kTypeName);
        CHECK(slot.make() != nullptr);
    }

    SECTION("合成序列：每个包都有终态，没有契约错误、算法失败与被隔离的异常") {
        const SyntheticSequence sequence;
        const auto run = detail::runSynthetic<S>(config, sequence);
        CHECK(run.outcome.packets_without_outcome == 0);
        CHECK(run.outcome.contract_errors == 0);
        CHECK(run.outcome.failed_frames == 0);
        CHECK(run.outcome.isolated_failures == 0);
        detail::checkPoses(run.odometry, sequence.scans);
    }

    SECTION("同样的输入跑两次，轨迹逐位相同") {
        const SyntheticSequence sequence;
        const auto a = detail::runSynthetic<S>(config, sequence);
        const auto b = detail::runSynthetic<S>(config, sequence);
        CHECK(detail::sameTrajectory(a.odometry, b.odometry));
    }

    SECTION("退化帧（空扫描、极稀疏、共面）：不出错，可以降级或跟丢") {
        SyntheticSequence sequence;
        sequence.degenerate = {{5, SyntheticScan::kEmpty},
                               {9, SyntheticScan::kSparse},
                               {13, SyntheticScan::kCoplanar}};
        const auto run = detail::runSynthetic<S>(config, sequence);
        INFO("退化输入报降级或跟丢，不抛异常，也不给非法的结果");
        CHECK(run.outcome.packets_without_outcome == 0);
        CHECK(run.outcome.contract_errors == 0);
        CHECK(run.outcome.failed_frames == 0);
        CHECK(run.outcome.isolated_failures == 0);
        detail::checkPoses(run.odometry, sequence.scans);
    }
}

}  // namespace simpleslam::testing

/// 给一个方案生成契约测试：`SIMPLESLAM_ODOMETRY_CONTRACT(MyLio)`，默认配置。
#define SIMPLESLAM_ODOMETRY_CONTRACT(Type)                                         \
    TEST_CASE("方案契约：" #Type, "[contract][odometry]") {                       \
        ::simpleslam::testing::checkOdometryContract<Type>();                      \
    }
