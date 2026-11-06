// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file keyframe.hpp
/// 关键帧与关键帧库：真源的一半（另一半是位姿图）。
///
/// 关键帧写入后不可变，内含整包原始数据，因此各类表征都可以从它重建。
/// 注意"原始点与去畸变点不是一回事"：这里保存的是原始包（含逐点时间与 IMU 区间），
/// 去畸变结果是有版本的派生数据，不进真源。
///
/// 关键帧之间的相对约束由算法给出并原样保存（含来源与扰动约定）。
/// 框架不会拿"本帧配准的信息矩阵"冒充"上一关键帧到本关键帧的约束信息矩阵"——
/// 那是两个随机量（events.hpp 的 RelativeConstraint 说明了为什么）。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/framework/append_log.hpp>
#include <SimpleSLAM/framework/calibration.hpp>
#include <SimpleSLAM/framework/events.hpp>
#include <SimpleSLAM/framework/ids.hpp>
#include <SimpleSLAM/framework/resources.hpp>
#include <SimpleSLAM/framework/sensor_packet.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace simpleslam::fw {

/// 写入后不可变。
struct Keyframe {
    KeyframeId id{};
    std::uint64_t sequence = 0;
    /// 生成本帧的里程计段；复位前后的 T_odom_body 不属于同一个坐标系。
    std::uint64_t odometry_segment = 0;
    Timestamp time{0};
    /// 里程计系下的位姿。世界系位姿在位姿图里，两者分开。
    SE3d T_odom_body = SE3d::Identity();
    /// 整包原始数据：后端与各类地图模块都从这里取，不各存一份。
    std::shared_ptr<const SensorPacket> packet;
    /// 生成这一帧时所用的标定版本，重建时据此判断兼容性。
    std::uint64_t calibration_revision = 0;
    /// 生成这一帧时所用的标定**内容**。版本号只能标识内容、不能代替内容：标定换代之后，
    /// 按当时条件重建这一帧，靠的是这份快照。与帧上下文里的是同一份不可变对象，
    /// 多个关键帧共享，不复制。
    std::shared_ptr<const CalibrationData> calibration;

    /// 上一关键帧。首帧为空。
    KeyframeId previous{};
    /// 上一关键帧 → 本关键帧的相对约束。算法给出才有，框架不伪造。
    /// 为空表示"算法没有给出跨关键帧的传播量"，此时图边用启发式权重并如实标注。
    std::optional<RelativeConstraint> constraint_from_previous;

    /// 本帧配准的信息矩阵（本帧相对局部地图）。只供诊断与关键帧策略使用，
    /// 不作为图边权重——它与上面那条跨关键帧的约束不是同一个随机量。
    Mat6d registration_information = Mat6d::Zero();
    InformationSource registration_information_source = InformationSource::kUnset;
};

using KeyframePtr = std::shared_ptr<const Keyframe>;

/// 关键帧库的内容。按 id 升序追加，遍历顺序确定。
///
/// 存储用 AppendLog：新增一帧不必整份复制索引（append_log.hpp 说明了代价）。
struct KeyframeStoreData {
    AppendLog<KeyframePtr> frames;
    std::uint64_t next_sequence = 1;
    KeyframeId last_id{};

    /// 追加一帧。必须按 id 升序，否则二分查找不成立。
    void append(KeyframePtr frame) {
        if (!frame) throw std::logic_error("关键帧库不接受空帧");
        if (frames.size() != 0 && !(frames.back()->id < frame->id)) {
            throw std::logic_error("关键帧必须按 id 升序入库");
        }
        last_id = frame->id;
        frames.push_back(std::move(frame));
    }

    [[nodiscard]] KeyframePtr find(KeyframeId id) const {
        std::size_t lo = 0;
        std::size_t hi = frames.size();
        while (lo < hi) {
            const std::size_t mid = lo + (hi - lo) / 2;
            const auto& candidate = frames.at(mid);
            if (candidate->id == id) return candidate;
            if (candidate->id < id) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        return nullptr;
    }

    [[nodiscard]] std::size_t size() const noexcept { return frames.size(); }

    /// 从某个序号之后的全部关键帧，按 id 升序。消费者据此补读断号。
    /// 序号随 id 单调递增，所以起点可以二分定位，不必全表扫描。
    [[nodiscard]] std::vector<KeyframePtr> since(std::uint64_t sequence_exclusive) const {
        std::size_t lo = 0;
        std::size_t hi = frames.size();
        while (lo < hi) {
            const std::size_t mid = lo + (hi - lo) / 2;
            if (frames.at(mid)->sequence <= sequence_exclusive) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        return frames.tail(lo);
    }
};

using KeyframeStore = Resource<KeyframeStoreData>;

}  // namespace simpleslam::fw
