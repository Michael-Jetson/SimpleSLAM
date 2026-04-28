#pragma once

/// @file kdtree.hpp
/// nanoflann 零拷贝适配器——持有外部 vector<PointT> 的引用，不做数据拷贝
///
/// 公共接口只使用 std 类型（vector<size_t>、vector<float>），
/// nanoflann 内部类型限于 private（P1 接口最小化）。

#include <nanoflann.hpp>

#include <Eigen/Core>
#include <cassert>
#include <cstddef>
#include <memory>
#include <vector>

namespace simpleslam {

namespace detail {

/// nanoflann DatasetAdaptor——零拷贝绑定到 vector<PointT>
template <typename PointT>
struct PointCloudAdaptor {
    const std::vector<PointT>& points;

    explicit PointCloudAdaptor(const std::vector<PointT>& pts) : points(pts) {}

    [[nodiscard]] size_t kdtree_get_point_count() const { return points.size(); }

    [[nodiscard]] float kdtree_get_pt(size_t idx, size_t dim) const {
        return points[idx][static_cast<Eigen::Index>(dim)];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

}  // namespace detail

/// 基于 nanoflann 的 KD-Tree
/// @tparam PointT 点类型，须支持 operator[](Eigen::Index) -> float
template <typename PointT = Eigen::Vector3f>
class KDTree {
public:
    /// 构造并建树。points 的生命期必须长于本对象。
    explicit KDTree(const std::vector<PointT>& points)
        : adaptor_(points) {
        rebuildIndex();
    }

    /// 外部数据变更后重建索引
    void rebuild() { rebuildIndex(); }

    /// K 最近邻搜索
    /// @param query        查询点
    /// @param k            邻居数量
    /// @param[out] indices 结果索引（resize 到实际找到的数量）
    /// @param[out] squared_distances 对应的平方距离
    void knnSearch(const PointT& query, size_t k,
                   std::vector<size_t>& indices,
                   std::vector<float>& squared_distances) const {
        assert(index_ && "KDTree index not built");
        const size_t actual_k = std::min(k, adaptor_.kdtree_get_point_count());
        indices.resize(actual_k);
        squared_distances.resize(actual_k);
        if (actual_k == 0) return;

        nanoflann::KNNResultSet<float> result_set(actual_k);
        result_set.init(indices.data(), squared_distances.data());
        index_->findNeighbors(result_set, query.data());

        const size_t found = result_set.size();
        indices.resize(found);
        squared_distances.resize(found);
    }

    /// 半径搜索
    /// @param query        查询点
    /// @param radius       搜索半径
    /// @param[out] indices 结果索引
    /// @param[out] squared_distances 对应的平方距离
    void radiusSearch(const PointT& query, float radius,
                      std::vector<size_t>& indices,
                      std::vector<float>& squared_distances) const {
        assert(index_ && "KDTree index not built");
        const float sq_radius = radius * radius;

        std::vector<nanoflann::ResultItem<size_t, float>> matches;
        nanoflann::SearchParameters params;
        params.sorted = true;
        index_->radiusSearch(query.data(), sq_radius, matches, params);

        indices.resize(matches.size());
        squared_distances.resize(matches.size());
        for (size_t i = 0; i < matches.size(); ++i) {
            indices[i] = matches[i].first;
            squared_distances[i] = matches[i].second;
        }
    }

    [[nodiscard]] size_t size() const { return adaptor_.kdtree_get_point_count(); }
    [[nodiscard]] bool empty() const { return size() == 0; }

private:
    using IndexType = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, detail::PointCloudAdaptor<PointT>>,
        detail::PointCloudAdaptor<PointT>,
        3, size_t>;

    detail::PointCloudAdaptor<PointT> adaptor_;
    std::unique_ptr<IndexType> index_;

    void rebuildIndex() {
        if (adaptor_.kdtree_get_point_count() == 0) {
            index_.reset();
            return;
        }
        index_ = std::make_unique<IndexType>(3, adaptor_);
        index_->buildIndex();
    }
};

/// 3D float 点云的常用别名
using KDTree3f = KDTree<Eigen::Vector3f>;

}  // namespace simpleslam
