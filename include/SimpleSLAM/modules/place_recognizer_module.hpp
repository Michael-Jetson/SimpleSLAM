// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file place_recognizer_module.hpp
/// 位置识别模块：回环检测与重定位合一，共用一套描述子库。
///
/// 关键帧通知只提示"消费到哪里了"：模块按序号从关键帧库补读。先入库再通知，所以读到的关键帧
/// 一定已经在库里，发出的回环约束的端点图优化模块也一定能补读到。

#include <SimpleSLAM/comm/module.hpp>
#include <SimpleSLAM/plugins/place_recognizer.hpp>
#include <SimpleSLAM/modules/topics.hpp>

#include <cstdint>
#include <memory>
#include <utility>

namespace simpleslam::modules {

class PlaceRecognizerModule final : public comm::Module {
public:
    PlaceRecognizerModule(std::unique_ptr<plugin::PlaceRecognizerBase> algorithm,
                          SlamResources& resources)
        : algorithm_(std::move(algorithm)), resources_(&resources) {}

    [[nodiscard]] plugin::PlaceRecognizerBase& algorithm() noexcept { return *algorithm_; }
    [[nodiscard]] std::uint64_t indexedFrames() const noexcept { return indexed_; }
    [[nodiscard]] std::uint64_t loopsFound() const noexcept { return loops_; }

protected:
    void onInit() override;
    /// 主动核对一次补读游标，不等下一条通知。
    void onFinish() override;

private:
    void onKeyframe(const fw::KeyframeEvent& event);
    /// 按游标从关键帧库补读：建索引、找回环、发约束。
    void catchUpKeyframes();
    void onRelocationRequest(const fw::RelocationRequest& request);

    std::unique_ptr<plugin::PlaceRecognizerBase> algorithm_;
    SlamResources* resources_;
    comm::Publisher<fw::LoopConstraint> loop_pub_;
    comm::Publisher<fw::RelocationResult> reloc_pub_;
    std::uint64_t consumed_sequence_ = 0;
    std::uint64_t indexed_ = 0;
    std::uint64_t loops_ = 0;
};

}  // namespace simpleslam::modules
