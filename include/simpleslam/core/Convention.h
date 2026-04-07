#pragma once
// SimpleSLAM Framework Conventions
// - Coordinate system: ENU (East-North-Up), x=East, y=North, z=Up (ROS REP-103)
// - Quaternion format: Hamilton [w, x, y, z] (Eigen default)
// - Pose semantics: T_world_body transforms points from body frame to world frame
// - Point cloud precision: float (single precision)
// - State vector precision: double (for IEKF convergence)
// - Timestamp: seconds (double), UNIX time for sensor data timestamps.
//   Note: IClock::now() return value semantics are implementation-defined
//   (SystemClock returns process-relative elapsed time, DatasetClock returns
//   dataset timestamps). This convention covers sensor data only.

namespace SimpleSLAM {
namespace convention {
    constexpr const char* COORDINATE_SYSTEM = "ENU";
    constexpr const char* QUATERNION_FORMAT = "Hamilton_wxyz";
    constexpr const char* POSE_SEMANTICS = "T_world_body";
} // namespace convention
} // namespace SimpleSLAM
