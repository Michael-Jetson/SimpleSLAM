#pragma once

#include <simpleslam/data/SensorData.h>
#include <simpleslam/modules/ModuleBase.h>

namespace SimpleSLAM {

class SensorIO : public ModuleBase {
public:
    using ModuleBase::ModuleBase;

    virtual SensorData receive() = 0;
    [[nodiscard]] virtual bool hasMore() const = 0;

    /// Request the sensor source to stop blocking in receive().
    /// Called by Scheduler::stop() to enable graceful shutdown in online mode.
    /// Default: no-op (offline implementations don't block).
    virtual void requestStop() {}
};

} // namespace SimpleSLAM
