#include <SimpleSLAM/core/math/normal_estimation.hpp>
#include <SimpleSLAM/core/math/kdtree.hpp>

#include <Eigen/Eigenvalues>
#include <cassert>
#include <vector>

namespace simpleslam {

void estimateNormals(LidarScan& scan, int k_neighbors) {
    assert(k_neighbors >= 3);
    if (scan.empty()) return;

    KDTree3f tree(scan.points);
    const size_t k = static_cast<size_t>(k_neighbors) + 1;  // +1 包含查询点自身

    auto& normals = scan.normals.emplace();
    normals.resize(scan.size());

    std::vector<size_t> nn_indices;
    std::vector<float> nn_sq_dists;

    for (size_t i = 0; i < scan.size(); ++i) {
        tree.knnSearch(scan.points[i], k, nn_indices, nn_sq_dists);

        // 计算邻域质心
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        for (size_t idx : nn_indices) {
            mean += scan.points[idx];
        }
        mean /= static_cast<float>(nn_indices.size());

        // 计算 3x3 协方差矩阵
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (size_t idx : nn_indices) {
            const Eigen::Vector3f diff = scan.points[idx] - mean;
            cov += diff * diff.transpose();
        }

        // 最小特征值对应的特征向量即为法向量
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        Eigen::Vector3f normal = solver.eigenvectors().col(0);

        // 朝向传感器原点（scan 在传感器坐标系，原点即传感器位置）
        if (normal.dot(-scan.points[i]) < 0.0f) {
            normal = -normal;
        }

        normals[i] = normal;
    }

    assert(scan.isConsistent());
}

}  // namespace simpleslam
