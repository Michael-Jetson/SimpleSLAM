#pragma once

#include <chrono>
#include <thread>

#include <simpleslam/infrastructure/IClock.h>

namespace SimpleSLAM {

class SystemClock : public IClock {
public:
    SystemClock()
        : start_time_(std::chrono::steady_clock::now()) {}

    double now() const override {
        const auto elapsed = std::chrono::steady_clock::now() - start_time_;
        return std::chrono::duration<double>(elapsed).count();
    }

    void sleep(double duration) override {
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
    }

    void sleepUntil(double timestamp) override {
        const auto target = start_time_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                              std::chrono::duration<double>(timestamp));
        std::this_thread::sleep_until(target);
    }

private:
    std::chrono::steady_clock::time_point start_time_;
};

} // namespace SimpleSLAM
