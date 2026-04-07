#pragma once

#include <simpleslam/core/IService.h>

namespace SimpleSLAM {

class IFrameEventDrain : public IService {
public:
    ~IFrameEventDrain() override = default;
    virtual void drainOneRound() = 0;
};

} // namespace SimpleSLAM
