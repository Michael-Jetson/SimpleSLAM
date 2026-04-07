#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include <simpleslam/core/IService.h>
#include <simpleslam/data/PipelineResult.h>
#include <simpleslam/infrastructure/LogService.h>

namespace SimpleSLAM {

class IFrameEventDrain;
class RuntimeState;
class SensorIO;
class PipelineBase;
class BundleAssembler;
class TriggerPolicy;
class Initializer;
class FrameBoundaryHook;

class Scheduler : public IService {
public:
    Scheduler() = default;
    ~Scheduler() override;  // defined in Scheduler.cpp (unique_ptr<incomplete> needs complete type)

    // Setters declared here, defined in Scheduler.cpp
    // (unique_ptr<incomplete type> move-assign needs complete type)
    void setSensorIO(std::shared_ptr<SensorIO> sensor_io);
    void setTriggerPolicy(std::unique_ptr<TriggerPolicy> trigger_policy);
    void setBundleAssembler(std::unique_ptr<BundleAssembler> bundle_assembler);
    void setInitializer(std::unique_ptr<Initializer> initializer);
    void setPipeline(std::shared_ptr<PipelineBase> pipeline);
    void setEventDrain(std::shared_ptr<IFrameEventDrain> event_drain);
    void setTopicBus(std::shared_ptr<IFrameEventDrain> topic_bus);  // compatibility alias
    void setRuntimeState(std::shared_ptr<RuntimeState> runtime_state);
    void addHook(std::shared_ptr<FrameBoundaryHook> hook);
    void setOfflineMode(bool offline_mode) noexcept { offline_mode_ = offline_mode; }
    void setLog(Log log) { log_ = std::move(log); }

    [[nodiscard]] PipelineState state() const noexcept { return state_; }
    [[nodiscard]] bool isRunning() const noexcept {
        return running_.load(std::memory_order_relaxed);
    }

    void stop();
    void run();

private:
    void validateConfiguration() const;

    std::unique_ptr<TriggerPolicy> trigger_policy_;
    std::unique_ptr<BundleAssembler> bundle_assembler_;
    std::vector<std::shared_ptr<FrameBoundaryHook>> hooks_;

    std::shared_ptr<SensorIO> sensor_io_;
    std::unique_ptr<Initializer> initializer_;
    std::shared_ptr<PipelineBase> pipeline_;
    std::shared_ptr<IFrameEventDrain> event_drain_;
    std::shared_ptr<RuntimeState> runtime_state_;
    PipelineState state_ = PipelineState::INITIALIZING;
    std::atomic<bool> running_{false};
    bool offline_mode_ = true;
    Log log_;
};

} // namespace SimpleSLAM
