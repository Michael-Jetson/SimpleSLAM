#pragma once

#include <atomic>

#include <simpleslam/core/IService.h>
#include <simpleslam/data/PipelineResult.h>

namespace SimpleSLAM {

// RuntimeState is currently a scheduler-owned observable mirror of runtime phase
// and running state. Scheduler remains the sole writer in Phase 1.
class RuntimeState : public IService {
public:
    RuntimeState() = default;
    ~RuntimeState() override = default;

    [[nodiscard]] PipelineState phase() const noexcept {
        return phase_.load(std::memory_order_relaxed);
    }

    void setPhase(PipelineState phase) noexcept {
        phase_.store(phase, std::memory_order_relaxed);
    }

    [[nodiscard]] bool isRunning() const noexcept {
        return running_.load(std::memory_order_relaxed);
    }

    void setRunning(bool running) noexcept {
        running_.store(running, std::memory_order_relaxed);
    }

private:
    std::atomic<PipelineState> phase_{PipelineState::INITIALIZING};
    std::atomic<bool> running_{false};
};

} // namespace SimpleSLAM
