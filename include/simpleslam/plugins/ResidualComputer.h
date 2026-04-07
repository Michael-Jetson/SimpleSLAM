#pragma once

#include <utility>

#include <Eigen/Core>

#include <simpleslam/config/Params.h>
#include <simpleslam/core/Types.h>
#include <simpleslam/data/State.h>
#include <simpleslam/plugins/MapStructure.h>

namespace SimpleSLAM {

class ResidualComputer {
public:
    explicit ResidualComputer(Params params)
        : params_(std::move(params)) {}

    virtual ~ResidualComputer() = default;

    virtual void computeResiduals(const PointCloud& cloud,
                                  const MapStructure& map,
                                  const State& predicted_state,
                                  Eigen::VectorXd& residuals,
                                  Eigen::MatrixXd& jacobians) = 0;

    // Runtime reconfiguration (optional, e.g. dynamic weight adjustment)
    virtual void setParams(const Params& params) { params_ = params; }

protected:
    Params params_;
};

} // namespace SimpleSLAM
