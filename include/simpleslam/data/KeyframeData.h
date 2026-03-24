#pragma once
#include <optional>
#include <vector>
#include <cstdint>
#include <simpleslam/core/Types.h>

namespace SimpleSLAM {
// Capability tags: bit-field for keyframe data availability
enum class CapabilityTags : uint32_t {
    NONE            = 0,
    HAS_POINT_CLOUD = 1 << 0,
    HAS_IMAGE       = 1 << 1,
    HAS_GLOBAL_DESC = 1 << 2,
    HAS_BOW_VECTOR  = 1 << 3,
    HAS_SC_DESC     = 1 << 4,
};
inline CapabilityTags operator|(CapabilityTags a, CapabilityTags b) {
    return static_cast<CapabilityTags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline CapabilityTags operator&(CapabilityTags a, CapabilityTags b) {
    return static_cast<CapabilityTags>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}
inline CapabilityTags& operator|=(CapabilityTags& a, CapabilityTags b) {
    return a = a | b;
}
inline bool hasCapability(CapabilityTags tags, CapabilityTags flag) {
    return (static_cast<uint32_t>(tags) & static_cast<uint32_t>(flag)) != 0;
}

struct KeyframeData {
    int kf_id = -1;
    double timestamp = 0.0;
    SE3 pose_world_body;
    std::optional<PointCloudPtr> point_cloud;
    std::optional<ImagePtr> image;
    std::optional<std::vector<float>> global_descriptor;
    std::optional<BowVector> bow_vector;
    std::optional<ScanContextDescriptor> sc_descriptor;
    CapabilityTags capabilities = CapabilityTags::NONE;
};
} // namespace SimpleSLAM
