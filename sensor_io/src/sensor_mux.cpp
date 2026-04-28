#include <SimpleSLAM/sensor_io/sensor_mux.hpp>

#include <limits>

namespace simpleslam {

void SensorMux::addSource(std::unique_ptr<ISensorSource> source) {
    sources_.push_back(std::move(source));
}

bool SensorMux::hasNext() const {
    for (const auto& src : sources_) {
        if (src->hasNext()) return true;
    }
    return false;
}

std::optional<LidarScan> SensorMux::nextScan() {
    ISensorSource* earliest = nullptr;
    Timestamp earliest_ts = std::numeric_limits<double>::max();

    for (auto& src : sources_) {
        if (src->hasNext() && src->currentTimestamp() < earliest_ts) {
            earliest_ts = src->currentTimestamp();
            earliest = src.get();
        }
    }

    if (!earliest) return std::nullopt;
    return earliest->nextScan();
}

}  // namespace simpleslam
