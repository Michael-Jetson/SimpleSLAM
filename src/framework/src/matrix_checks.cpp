// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <Eigen/Eigenvalues>

#include "matrix_checks.hpp"

namespace simpleslam::fw::detail {

Definiteness definiteness(const Mat6d& m) {
    const double scale = m.cwiseAbs().maxCoeff();
    if (scale == 0.0) return Definiteness::kSymmetricPsd;
    if ((m - m.transpose()).cwiseAbs().maxCoeff() > 1e-9 * scale) return Definiteness::kAsymmetric;
    const Eigen::SelfAdjointEigenSolver<Mat6d> solver(m);
    return solver.eigenvalues().minCoeff() >= -1e-9 * scale ? Definiteness::kSymmetricPsd
                                                           : Definiteness::kIndefinite;
}

}  // namespace simpleslam::fw::detail
