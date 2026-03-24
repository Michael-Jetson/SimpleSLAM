#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <stdexcept>

#include <simpleslam/config/Node.h>

namespace SimpleSLAM {

/// Convenience conversions from Node (Array) to Eigen/Sophus types.
/// Kept separate from Node.h to avoid coupling Node to Eigen/Sophus.

inline Eigen::Vector3d nodeToVector3(const Node& node) {
    const auto& arr = node.asArray();
    if (arr.size() != 3) {
        throw std::runtime_error("Vector3 requires exactly 3 elements, got " +
                                 std::to_string(arr.size()));
    }
    return {arr[0].asDouble(), arr[1].asDouble(), arr[2].asDouble()};
}

inline Eigen::Matrix3d nodeToMatrix3(const Node& node) {
    const auto& arr = node.asArray();
    if (arr.size() != 9) {
        throw std::runtime_error("Matrix3 requires exactly 9 elements (row-major), got " +
                                 std::to_string(arr.size()));
    }
    Eigen::Matrix3d mat;
    for (int i = 0; i < 9; ++i) {
        mat(i / 3, i % 3) = arr[i].asDouble();
    }
    return mat;
}

inline Eigen::Matrix4d nodeToMatrix4(const Node& node) {
    const auto& arr = node.asArray();
    if (arr.size() != 16) {
        throw std::runtime_error("Matrix4 requires exactly 16 elements (row-major), got " +
                                 std::to_string(arr.size()));
    }
    Eigen::Matrix4d mat;
    for (int i = 0; i < 16; ++i) {
        mat(i / 4, i % 4) = arr[i].asDouble();
    }
    return mat;
}

inline Sophus::SE3d nodeToSE3(const Node& node) {
    Eigen::Matrix4d mat = nodeToMatrix4(node);
    return Sophus::SE3d(mat);
}

/// Convenience: extract Eigen::VectorXd of arbitrary size from a Node array
inline Eigen::VectorXd nodeToVectorXd(const Node& node) {
    const auto& arr = node.asArray();
    Eigen::VectorXd vec(static_cast<Eigen::Index>(arr.size()));
    for (size_t i = 0; i < arr.size(); ++i) {
        vec(static_cast<Eigen::Index>(i)) = arr[i].asDouble();
    }
    return vec;
}

/// Convenience: extract diagonal matrix from a Node array (e.g., process_noise_diag)
template<int N>
inline Eigen::Matrix<double, N, N> nodeToDiagonalMatrix(const Node& node) {
    const auto& arr = node.asArray();
    if (static_cast<int>(arr.size()) != N) {
        throw std::runtime_error("Diagonal matrix requires exactly " + std::to_string(N) +
                                 " elements, got " + std::to_string(arr.size()));
    }
    Eigen::Matrix<double, N, N> mat = Eigen::Matrix<double, N, N>::Zero();
    for (int i = 0; i < N; ++i) {
        mat(i, i) = arr[i].asDouble();
    }
    return mat;
}

} // namespace SimpleSLAM
