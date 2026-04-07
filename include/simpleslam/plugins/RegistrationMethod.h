#pragma once

#include <simpleslam/core/Types.h>

namespace SimpleSLAM {

class RegistrationMethod {
public:
    virtual ~RegistrationMethod() = default;

    [[nodiscard]] virtual SE3 align(const PointCloud& source,
                                    const PointCloud& target) = 0;
};

} // namespace SimpleSLAM
