// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file map.hpp
/// 配准地图的两个接口：查询（按批线性化）与写入（插入、维护）。
///
/// 地图的检索、插入、管理都在实现内部：结构和检索拆不开，所以没有"拿别的地图的
/// 操作去动这张图"这回事。接口按批调用，每次迭代一次虚调用，逐点循环在实现内部。
///
/// 所有实现都要满足的约定（契约测试守着）：
/// - **扰动**：左扰动、平移在前，更新是 `Exp(δ)·T`，与 manif 切向量的分量顺序一致。
/// - **H 的语义**：高斯牛顿型 Σ w JᵀJ，半正定；g = Σ w Jᵀ r；cost = ½ Σ ρ(e²)。
///   w 是鲁棒核 ρ 对残差平方的导数，所以 g 正是 cost 对 δ 的梯度。
/// - **6×6 的前提**：残差只经传感器位姿依赖状态。
/// - **退化输入有定义**：空图、没有点、没有对应都返回 0 行，不返回单位阵。
/// - **确定**：同样的地图与输入，结果逐位相同，与线程数无关。
/// - **维护**：什么时候做由算法决定，怎么做由地图决定；维护不增加点数。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/sensor_data.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace simpleslam::algo {

using Vec6d = Eigen::Matrix<double, 6, 1>;

/// 鲁棒核。`weight` 是 ρ 对残差平方 e² 的导数，`rho` 是代价。
struct RobustKernel {
    enum class Kind : std::uint8_t {
        kNone,          ///< ρ = e²
        kHuber,         ///< |e| ≤ s 时 ρ = e²，之外线性增长
        kGemanMcClure,  ///< ρ = s·e²/(s + e²)，s 取 scale²；KISS-ICP 用它
    };
    Kind kind = Kind::kNone;
    double scale = 1.0;  ///< 米

    [[nodiscard]] double weight(double e2) const noexcept;
    [[nodiscard]] double rho(double e2) const noexcept;
};

/// 随每次线性化传入的参数：每帧都可能变，比如自适应的对应阈值。
struct MatchOptions {
    double max_correspondence_m = 1.0;  ///< 超过它的对应不算
    RobustKernel kernel{};
};

/// 一次线性化的结果：以传感器位姿为状态的 6×6 正规方程。
struct LinearizedSystem {
    Mat6d H = Mat6d::Zero();
    Vec6d g = Vec6d::Zero();
    double cost = 0.0;
    std::size_t rows = 0;  ///< 残差行数；0 表示没有约束
};

/// 只读查询。定位模式只把这个接口交给算法，"不能写先验地图"由构造保证。
class MapQuery {
public:
    static constexpr std::string_view kInterfaceName = "MapQuery";

    MapQuery() = default;
    MapQuery(const MapQuery&) = delete;
    MapQuery& operator=(const MapQuery&) = delete;
    virtual ~MapQuery() = default;

    /// 把传感器系的点按 `T_map_sensor` 变到地图系，找对应并累加。
    /// 输入列不一致、选项不合法时抛 `std::invalid_argument`；
    /// 实现给出非有限的结果时抛 `std::logic_error`。
    [[nodiscard]] LinearizedSystem linearize(const fw::PointCloud& points,
                                             const SE3d& T_map_sensor,
                                             const MatchOptions& options) const;

    [[nodiscard]] virtual bool empty() const noexcept = 0;
    /// 地图里的点数。
    [[nodiscard]] virtual std::size_t size() const noexcept = 0;
    [[nodiscard]] virtual std::string_view typeName() const noexcept = 0;

private:
    [[nodiscard]] virtual LinearizedSystem doLinearize(const fw::PointCloud& points,
                                                       const SE3d& T_map_sensor,
                                                       const MatchOptions& options) const = 0;
};

/// 里程计自己的地图：查询加写入。地图槽要求的是这个接口。
class LinearizableMap : public MapQuery {
public:
    static constexpr std::string_view kInterfaceName = "LinearizableMap";

    /// 插入一帧：传感器系的点，加上这一帧在地图系里的位姿。列不一致时抛 `std::invalid_argument`。
    void insert(const fw::PointCloud& points, const SE3d& T_map_sensor);

    /// 维护一次。`center` 一般是当前传感器在地图系里的位置；
    /// 中心不是有限数时抛 `std::invalid_argument`。
    void maintain(const Vec3d& center);

    virtual void clear() = 0;

private:
    virtual void doInsert(const fw::PointCloud& points, const SE3d& T_map_sensor) = 0;
    virtual void doMaintain(const Vec3d& center) = 0;
};

}  // namespace simpleslam::algo
