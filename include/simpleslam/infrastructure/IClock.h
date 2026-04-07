#pragma once

#include <simpleslam/core/IService.h>

namespace SimpleSLAM {

class IClock : public IService {
public:
    ~IClock() override = default;

    virtual double now() const = 0;
    virtual void sleep(double duration) = 0;
    virtual void sleepUntil(double timestamp) = 0;
};

} // namespace SimpleSLAM
