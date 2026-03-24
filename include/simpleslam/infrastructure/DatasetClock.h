#pragma once

#include <stdexcept>
#include <string>

#include <simpleslam/infrastructure/IClock.h>

namespace SimpleSLAM {

class DatasetClock : public IClock {
public:
    explicit DatasetClock(double speed = 1.0)
        : playback_speed_(speed) {
        if (speed <= 0.0) {
            throw std::invalid_argument("DatasetClock playback_speed must be positive (got "
                                        + std::to_string(speed) + ")");
        }
    }

    double now() const override {
        return current_time_;
    }

    void sleep(double duration) override {
        current_time_ += duration * playback_speed_;
    }

    void sleepUntil(double timestamp) override {
        if (timestamp > current_time_) {
            current_time_ = timestamp;
        }
    }

    void advance(double timestamp) {
        current_time_ = timestamp;
    }

private:
    double current_time_ = 0.0;
    double playback_speed_ = 1.0;
};

} // namespace SimpleSLAM
