#pragma once
#include <Eigen/Core>

namespace SimpleSLAM {
struct DegeneracyInfo {
    bool is_degenerate = false;
    Eigen::Vector3d degenerate_directions = Eigen::Vector3d::Zero();
    double confidence = 1.0; // 0=fully degenerate, 1=no degeneracy
};
} // namespace SimpleSLAM
