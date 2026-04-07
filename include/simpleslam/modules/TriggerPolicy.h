#pragma once

#include <simpleslam/data/SensorData.h>

namespace SimpleSLAM {

class TriggerPolicy {
public:
    virtual ~TriggerPolicy() = default;

    virtual bool shouldTrigger(const SensorData& data) = 0;
};

} // namespace SimpleSLAM
