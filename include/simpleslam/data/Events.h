#pragma once
#include "simpleslam/core/Types.h"

namespace SimpleSLAM {
struct LoopDetectedEvent {
    int query_kf_id;
    int match_kf_id;
    SE3 relative_pose; // T_match_query
    double confidence;
};
} // namespace SimpleSLAM
