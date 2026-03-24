#pragma once

#include <simpleslam/data/PipelineResult.h>
#include <simpleslam/data/SensorData.h>

namespace SimpleSLAM {

class FrameBoundaryHook {
public:
    virtual ~FrameBoundaryHook() = default;

    virtual void onFrameStart(const SensorBundle& bundle) {}
    virtual void onFrameEnd(const PipelineResult& result) {}
};

} // namespace SimpleSLAM
