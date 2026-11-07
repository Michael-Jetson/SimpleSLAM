// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

// 机制验收：用户写的地图与方案，经契约测试模板一行验收；只改 YAML，方案就换用第二种地图；
// 选了不满足接口的实现，装配时就报错。这里的地图与方案都是测试替身，不是算法。

#include <SimpleSLAM/algo/builtins.hpp>
#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/task.hpp>
#include <SimpleSLAM/config/error.hpp>
#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/modules/slam_system.hpp>
#include <SimpleSLAM/testing/map_contract.hpp>
#include <SimpleSLAM/testing/odometry_contract.hpp>
#include <SimpleSLAM/testing/synthetic_source.hpp>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

using namespace simpleslam;

namespace {

/// 第二种地图：暴力最近邻的点到点地图。和体素点地图守同一份契约，另外数自己被线性化了几次，
/// 好看出方案用的是哪一种地图。
class BruteForceMap final : public algo::LinearizableMap {
public:
    static constexpr std::string_view kTypeName = "brute_force_map";
    struct Config {
        template <class V>
        void visit(V& /*v*/) {}
    };
    explicit BruteForceMap(const Config& /*config*/) {}

    static inline std::atomic<int> linearizations{0};

    [[nodiscard]] bool empty() const noexcept override { return points_.empty(); }
    [[nodiscard]] std::size_t size() const noexcept override { return points_.size(); }
    [[nodiscard]] std::string_view typeName() const noexcept override { return kTypeName; }
    void clear() override { points_.clear(); }

private:
    [[nodiscard]] algo::LinearizedSystem doLinearize(
        const fw::PointCloud& points, const SE3d& T_map_sensor,
        const algo::MatchOptions& options) const override {
        linearizations.fetch_add(1);
        algo::LinearizedSystem out;
        if (points_.empty()) return out;
        const double max_corr2 = options.max_correspondence_m * options.max_correspondence_m;
        Eigen::Matrix<double, 3, 6> J;
        J.leftCols<3>().setIdentity();
        for (std::size_t i = 0; i < points.size(); ++i) {
            const Vec3d p = T_map_sensor.act(Vec3d(points.x[i], points.y[i], points.z[i]));
            const Vec3d* best = nullptr;
            double best2 = std::numeric_limits<double>::infinity();
            for (const Vec3d& q : points_) {
                const double d2 = (p - q).squaredNorm();
                if (d2 < best2) {
                    best2 = d2;
                    best = &q;
                }
            }
            if (best == nullptr || best2 > max_corr2) continue;
            const Vec3d r = p - *best;
            const double w = options.kernel.weight(best2);
            // 左扰动、平移在前：∂(Exp(δ)·p)/∂δ = [I | −[p]×]
            J.rightCols<3>() << 0.0, p.z(), -p.y(),  //
                -p.z(), 0.0, p.x(),                  //
                p.y(), -p.x(), 0.0;
            out.H.noalias() += w * J.transpose() * J;
            out.g.noalias() += w * J.transpose() * r;
            out.cost += 0.5 * options.kernel.rho(best2);
            out.rows += 3;
        }
        return out;
    }
    void doInsert(const fw::PointCloud& points, const SE3d& T_map_sensor) override {
        for (std::size_t i = 0; i < points.size(); ++i) {
            points_.push_back(T_map_sensor.act(Vec3d(points.x[i], points.y[i], points.z[i])));
        }
    }
    void doMaintain(const Vec3d& /*center*/) override {}  // 什么都不删也不增加点数

    std::vector<Vec3d> points_;
};

/// 带地图槽的方案：第一帧建图，之后每帧按恒速的位姿线性化一次。只为看出方案用的是哪一种
/// 地图，不做配准；地图也不随帧数长大。
class MapScheme final : public plugin::OdometryBase {
public:
    static constexpr std::string_view kTypeName = "map_scheme";
    struct Config {
        config::Slot<algo::LinearizableMap> map =
            config::Slot<algo::LinearizableMap>::of<algo::VoxelPointMap>();
        template <class V>
        void visit(V& v) {
            v("map", map, "配准地图");
        }
    };
    explicit MapScheme(const Config& config) : map_(config.map.make()) {}

    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }

protected:
    plugin::TrackResult track(const fw::SensorPacket& packet) override {
        const SE3d pose(Vec3d(0.1 * static_cast<double>(frames_), 0.0, 0.0),
                        Eigen::Quaterniond::Identity());
        ++frames_;
        if (const fw::LidarScan* scan = packet.lidar(); scan != nullptr && !scan->points.empty()) {
            if (map_->empty()) {
                map_->insert(scan->points, pose);
            } else {
                (void)map_->linearize(scan->points, pose, algo::MatchOptions{});
            }
        }
        plugin::TrackResult result;
        result.status = fw::TrackingStatus::kTracking;
        result.T_odom_body = pose;
        result.registration_information = Mat6d::Identity() * 100.0;
        result.registration_information_source = fw::InformationSource::kPropagated;
        if (hasKeyframe()) {
            fw::RelativeConstraint constraint;
            constraint.from_time = lastKeyframeTime();
            constraint.to_time = packet.reference_time;
            constraint.measurement = lastKeyframePose().inverse().compose(pose);
            constraint.information = Mat6d::Identity() * 25.0;
            constraint.source = fw::InformationSource::kPropagated;
            result.constraint_to_last_keyframe = constraint;
        }
        result.suggest_keyframe = (frames_ % 2 == 0);
        return result;
    }
    void onReset() override {
        frames_ = 0;
        map_->clear();
    }

private:
    std::unique_ptr<algo::LinearizableMap> map_;
    std::uint64_t frames_ = 0;
};

/// 按一份任务 YAML 装配并跑完：里程计的插件节点由 `odometry` 给出，其余都相同。
/// 返回第二种地图被线性化的次数。
int linearizationsWith(const std::string& odometry) {
    config::Catalogs catalogs = app::builtinCatalogs();
    catalogs.of<algo::LinearizableMap>().add<BruteForceMap>();
    catalogs.of<plugin::OdometryBase>().add<MapScheme>();
    const std::string yaml = "data:\n  source: {type: kitti_odometry, path: /data/x}\n"
                             "system:\n  modules:\n    odometry:\n      plugin: "
                             + odometry + "\n";
    const app::ResolvedTask task =
        app::resolveTask(config::loadTaskString(yaml, "<任务>"), catalogs);
    BruteForceMap::linearizations = 0;
    modules::SlamSystem system;
    system.setSource(std::make_unique<testing::SyntheticLidarImuSource>());
    system.setOdometry(task.odometry.make());
    system.init();
    system.runOffline();
    system.shutdown();
    return BruteForceMap::linearizations.load();
}

}  // namespace

// 用户写的实现：契约测试各一行
SIMPLESLAM_MAP_CONTRACT(BruteForceMap)
SIMPLESLAM_ODOMETRY_CONTRACT(MapScheme)

TEST_CASE("机制验收：只改 YAML，方案就换用第二种地图", "[acceptance][mechanism]") {
    CHECK(linearizationsWith("{type: map_scheme}") == 0);  // 默认是体素点地图
    CHECK(linearizationsWith("{type: map_scheme, map: {type: brute_force_map}}") > 0);
}

TEST_CASE("机制验收：地图槽选了不满足接口的实现，装配时就报错", "[acceptance][mechanism]") {
    CHECK_THROWS_AS(linearizationsWith("{type: map_scheme, map: {type: voxel_downsample}}"),
                    config::ConfigError);
}
