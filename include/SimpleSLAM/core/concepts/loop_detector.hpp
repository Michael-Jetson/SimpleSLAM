#pragma once

/// @file loop_detector.hpp
/// LoopDetector C++20 concept —— 回环检测的统一抽象
///
/// 具体实现（ScanContext、STD、DBoW2 等）满足此 concept 即可被框架使用。

#include <SimpleSLAM/core/types/geometry.hpp>
#include <SimpleSLAM/core/types/keyframe.hpp>

#include <concepts>
#include <cstdint>
#include <optional>

namespace simpleslam {

/// 回环候选结果
struct LoopCandidate {
    uint64_t match_keyframe_id;     ///< 匹配到的历史关键帧 ID
    SE3d T_match_query;             ///< 从 query 到 match 的相对变换
    double score;                    ///< 得分（越高越好，算法定义尺度）
};

/// 回环检测器 concept
template <typename T>
concept LoopDetector = requires(
    T& detector,
    const KeyframeData& keyframe) {
    { detector.addKeyframe(keyframe) } -> std::same_as<void>;
    { detector.detect(keyframe) } -> std::convertible_to<std::optional<LoopCandidate>>;
};

}  // namespace simpleslam
