#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <memory>
#include <vector>
#include <optional>
#include <string>
#include <cstdint>

namespace SimpleSLAM {
using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;

struct Point {
    Eigen::Vector3f pos;  // float precision for point clouds
    float intensity = 0.0f;
};
using PointCloud = std::vector<Point>;
using PointCloudPtr = std::shared_ptr<const PointCloud>;      // immutable shared (cross-module safety)
using MutablePointCloudPtr = std::shared_ptr<PointCloud>;     // producer-side only

// Image placeholder
struct Image {
    int width = 0, height = 0;
    std::vector<uint8_t> data;
};
using ImagePtr = std::shared_ptr<const Image>;                // immutable shared
using MutableImagePtr = std::shared_ptr<Image>;               // producer-side only

// IMU measurement
struct IMUMeasurement {
    double timestamp;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
};

// Frame types  
using FrameId = int;
using LidarFrame = PointCloud; // placeholder
using CameraFrame = Image;    // placeholder
using ImuData = IMUMeasurement;

// Forward declarations
struct KeyframeData;
struct PipelineResult;
struct Correction;
struct SensorBundle;

// Nearest neighbor result
struct NearestResult {
    struct Neighbor {
        Point point;
        double distance_sq;
    };
    std::vector<Neighbor> neighbors;
};

// Tracking quality (placeholder for KeyframeSelector)
struct TrackingQuality {
    double translation_score = 1.0;
    double rotation_score = 1.0;
};

// Constraint (odometry edge etc.)
struct Constraint {
    FrameId from_id;
    FrameId to_id;
    SE3 relative_pose;
    Eigen::Matrix<double, 6, 6> information;
};

// MapDelta
struct MapDelta {
    std::vector<Point> new_points;
    std::vector<int> removed_indices;
};

// BowVector / ScanContextDescriptor placeholders
using BowVector = std::vector<std::pair<int, double>>;
using ScanContextDescriptor = std::vector<float>;
} // namespace SimpleSLAM
