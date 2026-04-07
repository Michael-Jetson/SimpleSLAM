#pragma once

#include <simpleslam/core/Types.h>

namespace SimpleSLAM {

class KeyframeSelector {
public:
    virtual ~KeyframeSelector() = default;

    [[nodiscard]] virtual bool isKeyframe(const SE3& current_pose,
                                          const SE3& last_keyframe_pose,
                                          const TrackingQuality& quality) = 0;
};

} // namespace SimpleSLAM
