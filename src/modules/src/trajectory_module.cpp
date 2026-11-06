// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/modules/trajectory_module.hpp>

namespace simpleslam::modules {

TrajectoryRecorder::TrajectoryRecorder() {
    odometry_.setFrame(fw::TrajectoryFrame::kOdom);
    odometry_.setTracked("body");
    odometry_.setName("odometry");
    online_.setFrame(fw::TrajectoryFrame::kWorld);
    online_.setTracked("body");
    online_.setName("online");
}

void TrajectoryRecorder::add(const fw::OdometryResult& result) {
    ++received_;
    if (result.status == fw::TrackingStatus::kInitializing) return;
    // 时间必须严格递增；来源重启或时钟回跳时宁可跳过并记下来，不悄悄接受
    if (!odometry_.empty() && result.time <= odometry_.samples().back().time) {
        ++skipped_;
        return;
    }
    odometry_.push(result.time, result.T_odom_body);
    online_.push(result.time, result.T_world_body());  // 事实：当时输出过的值
    if (result.applied_correction.solution_revision != 0) {
        ++corrected_;
        last_correction_ = result.applied_correction;
    }
}

fw::Trajectory keyframeTrajectory(const SlamResources& resources) {
    fw::Trajectory trajectory;
    trajectory.setFrame(fw::TrajectoryFrame::kWorld);
    trajectory.setTracked("body");
    trajectory.setName("keyframes");
    const auto graph = resources.graph.snapshot();
    graph->nodes.forEach(
        [&](const fw::PoseGraphNode& node) { trajectory.push(node.time, node.T_world_body); });
    return trajectory;
}

TrajectoryModule::TrajectoryModule(SlamResources& resources) : resources_(&resources) {}

void TrajectoryModule::onInit() {
    subscribe(topics::kOdometry, 0, [this](const fw::OdometryResult& result) {
        recorder_.add(result);
    });
}

}  // namespace simpleslam::modules
