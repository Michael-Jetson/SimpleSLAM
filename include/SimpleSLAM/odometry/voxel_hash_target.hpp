#pragma once

/// @file voxel_hash_target.hpp
/// 体素哈希配准地图——满足 RegistrationTarget concept
///
/// 对标 KISS-ICP VoxelHashMap：体素哈希存储地图点，
/// match() 做 27 邻域最近邻 + 填充点到点残差和左扰动雅可比，
/// update() 将新帧点云插入地图（防重叠 + 容量限制）。

#include <SimpleSLAM/core/concepts/registration_target.hpp>
#include <SimpleSLAM/core/math/voxel_grid.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/core/types/sensor_data.hpp>

#include <tsl/robin_map.h>

#include <cmath>
#include <limits>
#include <vector>

namespace simpleslam {

/// 体素哈希配准地图配置
struct VoxelHashTargetConfig {
    float voxel_size = 1.0f;              ///< 体素边长（米），KISS-ICP 默认 0.01×max_range
    int max_points_per_voxel = 20;        ///< 每个体素最多存储的点数，控制查询上界
    double max_range = 100.0;             ///< 点的最大传感器距离，超出不插入地图
    double max_correspondence_dist = 3.0; ///< match() 中有效对应的最大距离
};

/// 体素哈希配准地图
///
/// 核心数据结构：robin_map<体素坐标, 点列表>
///   - 插入 O(1)：算体素坐标 → 哈希定位桶 → 追加
///   - 查询 O(1)：27 邻域 × 每桶最多 20 点 = 最多 540 次比较
///   - 与地图总点数无关，适合在线增量 SLAM
class VoxelHashTarget final {
public:
    explicit VoxelHashTarget(const VoxelHashTargetConfig& config = {})
        : config_(config)
        , max_range_sq_(config.max_range * config.max_range)
        , max_corr_sq_(config.max_correspondence_dist
                       * config.max_correspondence_dist)
        // 同一体素内点的最小间距²，保证体素内点近似均匀分布
        , min_dist_sq_(static_cast<double>(config.voxel_size * config.voxel_size)
                       / config.max_points_per_voxel) {}

    // ── RegistrationTarget concept 要求的四个方法 ──

    /// 在地图中为当前帧的每个点查找最近邻，计算残差和雅可比
    ///
    /// 残差：r = p_world - q_nearest（3D 向量，指向地图最近点的偏差）
    /// 雅可比：J = [I₃ | -[p_world]×]（左扰动，3×6 矩阵）
    ///   前 3 列 I₃ 对应平移，后 3 列 -[p]× 对应旋转
    void match(const LidarScan& scan, const SE3d& pose, MatchResult& result) {
        if (map_.empty()) return;

        // 从位姿提取旋转和平移，避免每个点重复解包
        auto R = pose.rotation();
        auto t = pose.translation();

        for (const auto& point : scan.points) {
            // 将传感器坐标系下的点变换到世界坐标系
            Eigen::Vector3d p_world = R * point.cast<double>() + t;

            // 27 邻域最近邻搜索，返回指向地图内部的指针（nullptr=未找到）
            const auto* closest = findClosestNeighbor(p_world);
            if (!closest) continue;

            // 残差向量：变换后的源点到最近地图点的偏差
            Eigen::Vector3d r = p_world - *closest;

            // 距离过滤：超出最大对应距离的匹配是外点，跳过
            if (r.squaredNorm() > max_corr_sq_) continue;

            double px = p_world.x(), py = p_world.y(), pz = p_world.z();

            // 写入 3 个残差分量
            result.residuals.push_back(r.x());
            result.residuals.push_back(r.y());
            result.residuals.push_back(r.z());

            // 写入 3×6 雅可比（row-major 展平）
            // J = [I₃ | -[p_world]×]，其中 [p]× 是反对称矩阵（叉积矩阵）
            //
            //     | 1  0  0 |  0   pz  -py |
            // J = | 0  1  0 | -pz   0   px |
            //     | 0  0  1 |  py  -px   0 |
            //       平移部分    旋转部分

            // 第 1 行
            result.jacobians.push_back(1.0);
            result.jacobians.push_back(0.0);
            result.jacobians.push_back(0.0);
            result.jacobians.push_back(0.0);
            result.jacobians.push_back(pz);
            result.jacobians.push_back(-py);

            // 第 2 行
            result.jacobians.push_back(0.0);
            result.jacobians.push_back(1.0);
            result.jacobians.push_back(0.0);
            result.jacobians.push_back(-pz);
            result.jacobians.push_back(0.0);
            result.jacobians.push_back(px);

            // 第 3 行
            result.jacobians.push_back(0.0);
            result.jacobians.push_back(0.0);
            result.jacobians.push_back(1.0);
            result.jacobians.push_back(py);
            result.jacobians.push_back(-px);
            result.jacobians.push_back(0.0);

            // 每个匹配贡献 3 行（3D 残差）
            result.num_valid += 3;
        }
    }

    /// 将当前帧的点变换到世界坐标系后插入地图
    ///
    /// 防重叠策略：
    ///   - 桶满（>= max_points_per_voxel）→ 跳过
    ///   - 与桶内已有点距离 < min_dist → 跳过（信息增益不足）
    void update(const LidarScan& scan, const SE3d& pose) {
        auto R = pose.rotation();
        auto t = pose.translation();

        for (const auto& point : scan.points) {
            // 传感器坐标系下的距离过滤（比在世界系下算更快——省一次变换）
            auto p_local = point.cast<double>();
            if (p_local.squaredNorm() > max_range_sq_) continue;

            // 变换到世界坐标系
            Eigen::Vector3d p_world = R * p_local + t;

            // 计算体素坐标（向负无穷取整）
            auto voxel = toVoxelCoord(p_world.cast<float>(), config_.voxel_size);

            // 获取该体素的桶（不存在时自动创建空桶）
            auto& bucket = map_[voxel];

            // 桶满则跳过
            if (static_cast<int>(bucket.size()) >= config_.max_points_per_voxel)
                continue;

            // 防重叠：新点与桶内已有点太近则跳过
            bool too_close = false;
            for (const auto& existing : bucket) {
                if ((p_world - existing).squaredNorm() < min_dist_sq_) {
                    too_close = true;
                    break;
                }
            }
            if (!too_close) bucket.push_back(p_world);
        }
    }

    /// 地图是否为空
    [[nodiscard]] bool empty() const { return map_.empty(); }

    /// 地图中的总点数（遍历所有体素累加）
    [[nodiscard]] size_t size() const {
        size_t total = 0;
        for (const auto& [_, bucket] : map_) total += bucket.size();
        return total;
    }

    // ── 额外方法 ──

    /// 滑窗地图：删除距 origin 超过 max_dist 的体素
    void removeIfFar(const Eigen::Vector3d& origin, double max_dist) {
        double max_dist_sq = max_dist * max_dist;
        for (auto it = map_.begin(); it != map_.end();) {
            // 用桶内第一个点的距离近似判断整个体素是否过远
            if (!it->second.empty() &&
                (it->second.front() - origin).squaredNorm() > max_dist_sq) {
                it = map_.erase(it);  // erase 返回下一个有效迭代器
            } else {
                ++it;
            }
        }
    }

    /// 清空整个地图
    void clear() { map_.clear(); }

    /// 非空体素数量
    [[nodiscard]] size_t voxelCount() const { return map_.size(); }

private:
    /// 27 邻域最近邻搜索
    ///
    /// 遍历查询点所在体素及其 26 个邻居，在所有桶内的点中找距离最小的。
    /// 返回指向 map_ 内部的 const 指针（零拷贝），nullptr 表示未找到。
    [[nodiscard]] const Eigen::Vector3d* findClosestNeighbor(
            const Eigen::Vector3d& query) const {
        auto voxel = toVoxelCoord(query.cast<float>(), config_.voxel_size);
        double best_sq = std::numeric_limits<double>::max();
        const Eigen::Vector3d* best = nullptr;

        // 遍历 3×3×3 = 27 个邻居体素（含自身）
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    VoxelCoord neighbor{voxel.x + dx, voxel.y + dy, voxel.z + dz};
                    // 用 find 而非 operator[]——避免查询时创建空桶
                    auto it = map_.find(neighbor);
                    if (it == map_.end()) continue;
                    // 在该体素的所有点中找最近的
                    for (const auto& p : it->second) {
                        double d_sq = (query - p).squaredNorm();
                        if (d_sq < best_sq) {
                            best_sq = d_sq;
                            best = &p;  // 指向 map_ 内部，生命周期安全
                        }
                    }
                }
            }
        }
        return best;
    }

    VoxelHashTargetConfig config_;
    double max_range_sq_;   ///< max_range²，预计算避免热路径重复乘法
    double max_corr_sq_;    ///< max_correspondence_dist²
    double min_dist_sq_;    ///< 同一体素内点的最小间距²
    /// 体素坐标 → 该体素内的地图点列表（核心数据结构）
    tsl::robin_map<VoxelCoord, std::vector<Eigen::Vector3d>, VoxelHash> map_;
};

static_assert(RegistrationTarget<VoxelHashTarget>,
              "VoxelHashTarget must satisfy RegistrationTarget concept");

}  // namespace simpleslam
