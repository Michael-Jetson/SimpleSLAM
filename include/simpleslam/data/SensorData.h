#pragma once
#include <variant>
#include <optional>
#include <vector>
#include <simpleslam/core/Types.h>

namespace SimpleSLAM {
// Raw sensor data (single measurement)
struct SensorData {
    enum class Type { LiDAR, Camera, IMU };
    Type type;
    std::variant<LidarFrame, CameraFrame, ImuData> payload;
    double timestamp = 0.0;
};

// Assembled sensor bundle for pipeline processing
struct SensorBundle {
    double timestamp = 0.0;
    std::optional<PointCloudPtr> lidar;
    std::optional<ImagePtr> image;
    std::optional<std::vector<IMUMeasurement>> imu_batch;
};
} // namespace SimpleSLAM
