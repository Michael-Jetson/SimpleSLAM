#pragma once

/// @file keyframe.hpp
/// 关键帧数据结构——一个关键帧所有关联信息的被动容器
///
/// KeyframeData 是纯数据记录，不含业务逻辑。
/// 通过 shared_ptr<const LidarScan/ImageFrame> 实现零拷贝共享（P2 值语义）。
/// extras 提供可扩展属性，与 LidarScan::extras 模式一致。

#include <SimpleSLAM/core/types/common.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/core/types/sensor_data.hpp>

#include <any>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

namespace simpleslam {

/// 关键帧核心数据——插入 KeyframeStore 后核心字段不可变
struct KeyframeData {
    uint64_t id{0};
    Timestamp timestamp{0.0};
    SE3d pose{};                                    ///< T_world_body（关键帧时刻的最优估计）

    /// 传感器数据（shared_ptr<const> 零拷贝不可变共享，P2）
    std::shared_ptr<const LidarScan> scan;
    std::shared_ptr<const ImageFrame> image;         ///< 可选，VIO/LIVO 时使用

    /// 可扩展属性（回环描述子、语义标签、AI 描述等）
    /// 后端服务在注册时声明并写入，写入后不可变
    std::unordered_map<std::string, std::any> extras;

    /// 安全获取扩展属性（类型不匹配或不存在时返回 nullptr）
    template <typename T>
    [[nodiscard]] const T* tryGetExtra(const std::string& key) const {
        auto it = extras.find(key);
        if (it == extras.end()) return nullptr;
        return std::any_cast<T>(&it->second);
    }

    [[nodiscard]] bool hasExtra(const std::string& key) const {
        return extras.count(key) > 0;
    }
};

}  // namespace simpleslam
