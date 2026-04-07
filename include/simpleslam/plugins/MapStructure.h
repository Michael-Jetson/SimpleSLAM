#pragma once

#include <simpleslam/core/Types.h>

namespace SimpleSLAM {

class MapStructure {
public:
    virtual ~MapStructure() = default;

    virtual void insert(const PointCloud& points) = 0;
    [[nodiscard]] virtual NearestResult queryNearest(const Point& point, int k) const = 0;
};

} // namespace SimpleSLAM
