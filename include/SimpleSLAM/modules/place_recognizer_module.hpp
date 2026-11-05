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
#include <SimpleSLAM/framework/diagnostics.hpp>
#include <SimpleSLAM/modules/topics.hpp>

#include <cstdint>
#include <memory>
#include <utility>

namespace simpleslam::modules {

class PlaceRecognizerModule final : public comm::Module {
public:
    PlaceRecognizerModule(std::unique_ptr<plugin::PlaceRecognizerBase> algorithm,
                          SlamResources& resources, fw::DiagnosticSink* diagnostics = nullptr)
        : algorithm_(std::move(algorithm)), resources_(&resources), diagnostics_(diagnostics) {}

    [[nodiscard]] plugin::PlaceRecognizerBase& algorithm() noexcept { return *algorithm_; }
    [[nodiscard]] std::uint64_t indexedFrames() const noexcept { return indexed_; }
    [[nodiscard]] std::uint64_t loopsFound() const noexcept { return loops_; }

protected:
    void onInit() override {
        loop_pub_ = advertise<fw::LoopConstraint>(topics::kLoopConstraint);
        reloc_pub_ = advertise<fw::RelocationResult>(topics::kRelocationResult);
        subscribe(topics::kKeyframe, 0, &PlaceRecognizerModule::onKeyframe);
        subscribe(topics::kRelocationRequest, 0, &PlaceRecognizerModule::onRelocationRequest);
    }

    /// 主动核对一次补读游标，不等下一条通知。
    void onFinish() override { catchUpKeyframes(); }

private:
    void onKeyframe(const fw::KeyframeEvent& event) {
        (void)event;  // 通知只提示位置，真正的数据按序号补读
        catchUpKeyframes();
    }

    /// 按游标从关键帧库补读：建索引、找回环、发约束。
    void catchUpKeyframes() {
        const auto store = resources_->keyframes.snapshot();
        for (const auto& frame : store->since(consumed_sequence_)) {
            algorithm_->insert(*frame);
            ++indexed_;
            consumed_sequence_ = frame->sequence;

            for (auto& constraint : algorithm_->detectLoops(*frame)) {
                constraint.id = fw::ConstraintId{++loops_};
                loop_pub_.publish(constraint);
            }
        }
    }

    void onRelocationRequest(const fw::RelocationRequest& request) {
        const auto store = resources_->keyframes.snapshot();
        const auto frame = store->find(request.query);
        if (!frame || !frame->packet) {
            fw::RelocationResult result;
            result.generation = request.generation;
            result.session = resources_->session;
            result.success = false;
            reloc_pub_.publish(result);
            return;
        }
        auto result = algorithm_->relocalize(*frame->packet, request);
        result.session = resources_->session;
        reloc_pub_.publish(result);
        report(result.success ? "place_recognizer.reloc_ok" : "place_recognizer.reloc_failed");
    }

    void report(const char* key) {
        if (diagnostics_ == nullptr) return;
        fw::DiagnosticRecord record;
        record.header.module = name();
        record.header.session = resources_->session;
        record.header.frame_sequence = consumed_sequence_;
        record.level = fw::DiagnosticClass::kControl;
        record.payload = fw::EventRecord{key, ""};
        diagnostics_->submit(record);
    }

    std::unique_ptr<plugin::PlaceRecognizerBase> algorithm_;
    SlamResources* resources_;
    fw::DiagnosticSink* diagnostics_;
    comm::Publisher<fw::LoopConstraint> loop_pub_;
    comm::Publisher<fw::RelocationResult> reloc_pub_;
    std::uint64_t consumed_sequence_ = 0;
    std::uint64_t indexed_ = 0;
    std::uint64_t loops_ = 0;
};

}  // namespace simpleslam::modules
