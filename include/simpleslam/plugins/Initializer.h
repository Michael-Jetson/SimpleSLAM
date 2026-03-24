#pragma once

#include <simpleslam/data/SensorData.h>
#include <simpleslam/data/State.h>

namespace SimpleSLAM {

enum class InitResult {
    Continue,
    Ready,
    Failed
};

class Initializer {
public:
    virtual ~Initializer() = default;

    virtual InitResult step(const SensorBundle& bundle) = 0;
    [[nodiscard]] virtual bool isReady() const = 0;
    [[nodiscard]] virtual double progress() const = 0;
    [[nodiscard]] virtual State getInitialState() const = 0;
};

} // namespace SimpleSLAM
