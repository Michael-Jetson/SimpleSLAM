// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file place_recognizer.hpp
/// 位置识别算法接口：回环检测与重定位。
///
/// 这是**插件**：纯算法，不碰总线、不碰资源，
/// 关键帧由所属模块按序号补读后传入（`modules/place_recognizer_module.hpp`）。
///
/// 回环检测与重定位是同一件事（检索 + 几何验证），只是查询来源与结果去向不同，
/// 所以共用一套描述子库，不拆成两个插件各存一份。

#include <SimpleSLAM/framework/events.hpp>
#include <SimpleSLAM/framework/keyframe.hpp>
#include <SimpleSLAM/framework/sensor_packet.hpp>

#include <string>
#include <string_view>
#include <vector>

namespace simpleslam::plugin {

namespace fw = simpleslam::fw;

class PlaceRecognizerBase {
public:
    static constexpr std::string_view kInterfaceName = "PlaceRecognizerBase";

    virtual ~PlaceRecognizerBase() = default;
    PlaceRecognizerBase(const PlaceRecognizerBase&) = delete;
    PlaceRecognizerBase& operator=(const PlaceRecognizerBase&) = delete;

    /// 新关键帧入库。框架保证与查询串行，实现内部不需要加锁。
    void insert(const fw::Keyframe& keyframe) { onInsert(keyframe); }

    /// 回环查询。返回的约束必须已通过几何验证；没有就返回空。
    [[nodiscard]] std::vector<fw::LoopConstraint> detectLoops(const fw::Keyframe& keyframe) {
        return onDetectLoops(keyframe);
    }

    /// 重定位查询。结果必须带回请求代号，跟踪恢复后旧结果据此被拒。
    [[nodiscard]] fw::RelocationResult relocalize(const fw::SensorPacket& packet,
                                                  const fw::RelocationRequest& request) {
        fw::RelocationResult result = onRelocalize(packet, request);
        result.generation = request.generation;
        return result;
    }

    [[nodiscard]] virtual std::string name() const { return "PlaceRecognizerBase"; }

protected:
    PlaceRecognizerBase() = default;

    virtual void onInsert(const fw::Keyframe& keyframe) = 0;
    virtual std::vector<fw::LoopConstraint> onDetectLoops(const fw::Keyframe& keyframe) = 0;
    virtual fw::RelocationResult onRelocalize(const fw::SensorPacket& packet,
                                              const fw::RelocationRequest& request) {
        (void)packet;
        (void)request;
        return fw::RelocationResult{};
    }
};

/// 不找回环。建图管线可以先不配位置识别：关键帧照常进位姿图，只是没有回环边。
class NoPlaceRecognizer final : public PlaceRecognizerBase {
public:
    static constexpr std::string_view kTypeName = "none";

    struct Config {
        template <class V>
        void visit(V& /*v*/) {}
    };

    explicit NoPlaceRecognizer(const Config& /*config*/ = {}) {}

    [[nodiscard]] std::string name() const override { return std::string(kTypeName); }

private:
    void onInsert(const fw::Keyframe& /*keyframe*/) override {}
    std::vector<fw::LoopConstraint> onDetectLoops(const fw::Keyframe& /*keyframe*/) override {
        return {};
    }
};

}  // namespace simpleslam::plugin
