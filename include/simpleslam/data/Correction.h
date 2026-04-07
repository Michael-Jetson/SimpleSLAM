#pragma once
#include <map>
#include <simpleslam/core/Types.h>

namespace SimpleSLAM {
struct Correction {
    std::map<int, SE3> corrected_poses; // kf_id -> new T_world_body
    enum Hint { FUTURE_ONLY, INCREMENTAL_PROPAGATION };
    Hint hint = FUTURE_ONLY;
    double confidence = 1.0;
};
} // namespace SimpleSLAM
