#include <SimpleSLAM/core/math/point_ops.hpp>
#include <SimpleSLAM/core/math/kdtree.hpp>
#include <SimpleSLAM/core/math/scan_utils.hpp>
#include <SimpleSLAM/core/math/voxel_grid.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>
#include <vector>

namespace simpleslam {

LidarScan voxelDownsample(const LidarScan& input, float voxel_size) {
    assert(voxel_size > 0.0f);
    if (input.empty()) return input;

    VoxelMap voxels;
    for (size_t i = 0; i < input.size(); ++i) {
        const auto coord = toVoxelCoord(input.points[i], voxel_size);
        voxels.insert(coord, input.points[i]);
    }

    LidarScan result;
    result.timestamp = input.timestamp;
    result.layout.type = ScanLayout::Type::Unorganized;
    result.points.reserve(voxels.size());

    voxels.forEach([&](const VoxelCoord&, const Voxel& voxel) {
        result.points.push_back(voxel.centroid());
    });

    // 降采样只保留几何坐标——可选字段在体素合并后无法保持一一对应
    assert(result.isConsistent());
    return result;
}

LidarScan rangeFilter(const LidarScan& input, float min_range, float max_range) {
    assert(min_range >= 0.0f && max_range > min_range);
    const float sq_min = min_range * min_range;
    const float sq_max = max_range * max_range;

    std::vector<size_t> kept_indices;
    kept_indices.reserve(input.size());

    for (size_t i = 0; i < input.size(); ++i) {
        const float sq_dist = input.points[i].squaredNorm();
        if (sq_dist >= sq_min && sq_dist <= sq_max) {
            kept_indices.push_back(i);
        }
    }
    return extractByIndices(input, kept_indices);
}

LidarScan cropBox(const LidarScan& input,
                  const Eigen::Vector3f& min_pt,
                  const Eigen::Vector3f& max_pt) {
    std::vector<size_t> kept_indices;
    kept_indices.reserve(input.size());

    for (size_t i = 0; i < input.size(); ++i) {
        const auto& p = input.points[i];
        if (p.x() >= min_pt.x() && p.x() <= max_pt.x() &&
            p.y() >= min_pt.y() && p.y() <= max_pt.y() &&
            p.z() >= min_pt.z() && p.z() <= max_pt.z()) {
            kept_indices.push_back(i);
        }
    }
    return extractByIndices(input, kept_indices);
}

LidarScan transformScan(const LidarScan& input, const SE3d& transform) {
    LidarScan result = input;

    const Eigen::Matrix3f rotation =
        transform.rotation().cast<float>();

    for (auto& p : result.points) {
        const Eigen::Vector3d pd = transform.act(p.cast<double>());
        p = pd.cast<float>();
    }

    if (result.hasNormals()) {
        for (auto& n : *result.normals) {
            n = rotation * n;
        }
    }

    assert(result.isConsistent());
    return result;
}

LidarScan statisticalOutlierRemoval(const LidarScan& input,
                                    int k_neighbors,
                                    float stddev_multiplier) {
    assert(k_neighbors > 0 && stddev_multiplier > 0.0f);
    if (input.size() <= static_cast<size_t>(k_neighbors)) return input;

    KDTree3f tree(input.points);
    const size_t k = static_cast<size_t>(k_neighbors) + 1;  // +1 因为查询点本身也会被找到

    std::vector<float> mean_distances(input.size());
    std::vector<size_t> nn_indices;
    std::vector<float> nn_sq_dists;

    for (size_t i = 0; i < input.size(); ++i) {
        tree.knnSearch(input.points[i], k, nn_indices, nn_sq_dists);

        float dist_sum = 0.0f;
        size_t count = 0;
        for (size_t j = 0; j < nn_sq_dists.size(); ++j) {
            if (nn_indices[j] == i) continue;  // 跳过自身
            dist_sum += std::sqrt(nn_sq_dists[j]);
            ++count;
        }
        mean_distances[i] = (count > 0) ? dist_sum / static_cast<float>(count) : 0.0f;
    }

    const double global_mean = std::accumulate(
        mean_distances.begin(), mean_distances.end(), 0.0) /
        static_cast<double>(mean_distances.size());

    double variance_sum = 0.0;
    for (float d : mean_distances) {
        const double diff = static_cast<double>(d) - global_mean;
        variance_sum += diff * diff;
    }
    const double global_stddev = std::sqrt(
        variance_sum / static_cast<double>(mean_distances.size()));

    const float threshold =
        static_cast<float>(global_mean + stddev_multiplier * global_stddev);

    std::vector<size_t> kept_indices;
    kept_indices.reserve(input.size());
    for (size_t i = 0; i < input.size(); ++i) {
        if (mean_distances[i] <= threshold) {
            kept_indices.push_back(i);
        }
    }
    return extractByIndices(input, kept_indices);
}

}  // namespace simpleslam
