#pragma once
#include <string>
#include <optional>
#include <vector>
#include <Eigen/Core>
#include <simpleslam/core/Types.h>
#include <simpleslam/data/KeyframeData.h>
#include <simpleslam/data/DegeneracyInfo.h>

namespace SimpleSLAM {
struct PipelineOutput {
    SE3 pose;
    Eigen::Matrix<double, 6, 6> pose_covariance = Eigen::Matrix<double, 6, 6>::Identity();
    std::optional<KeyframeData> keyframe;
    std::vector<Constraint> constraints;
    std::optional<MapDelta> map_delta;
};

struct PipelineResult {
    enum Status { OK, DEGRADED, LOST, FAILED };
    Status status = OK;
    PipelineOutput output;
    DegeneracyInfo degradation;
    std::string failure_reason;
};

// Pipeline state (managed by Scheduler)
enum class PipelineState {
    INITIALIZING,
    TRACKING,
    LOST
};
} // namespace SimpleSLAM
