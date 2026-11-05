// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/modules/place_recognizer_module.hpp>

namespace simpleslam::modules {

void PlaceRecognizerModule::onInit() {
    loop_pub_ = advertise(topics::kLoopConstraint);
    reloc_pub_ = advertise(topics::kRelocationResult);
    subscribe(topics::kKeyframe, 0, &PlaceRecognizerModule::onKeyframe);
    subscribe(topics::kRelocationRequest, 0, &PlaceRecognizerModule::onRelocationRequest);
}

void PlaceRecognizerModule::onFinish() { catchUpKeyframes(); }

void PlaceRecognizerModule::onKeyframe(const fw::KeyframeEvent& event) {
    (void)event;  // 通知只提示位置，真正的数据按序号补读
    catchUpKeyframes();
}

void PlaceRecognizerModule::catchUpKeyframes() {
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

void PlaceRecognizerModule::onRelocationRequest(const fw::RelocationRequest& request) {
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

void PlaceRecognizerModule::report(const char* key) {
    if (diagnostics_ == nullptr) return;
    fw::DiagnosticRecord record;
    record.header.module = name();
    record.header.session = resources_->session;
    record.header.frame_sequence = consumed_sequence_;
    record.level = fw::DiagnosticClass::kControl;
    record.payload = fw::EventRecord{key, ""};
    diagnostics_->submit(record);
}

}  // namespace simpleslam::modules
