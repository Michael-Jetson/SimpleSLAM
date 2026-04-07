#pragma once

#include <optional>

#include <simpleslam/data/SensorData.h>

namespace SimpleSLAM {

class BundleAssembler {
public:
    virtual ~BundleAssembler() = default;

    virtual void addData(SensorData data) = 0;
    [[nodiscard]] virtual std::optional<SensorBundle> assemble() = 0;
};

} // namespace SimpleSLAM
