#pragma once

#include <functional>
#include <memory>

#include <simpleslam/core/ConcurrentQueue.h>
#include <simpleslam/data/PipelineResult.h>
#include <simpleslam/data/SensorData.h>
#include <simpleslam/data/State.h>
#include <simpleslam/modules/ModuleBase.h>

namespace SimpleSLAM {

class PipelineBase : public ModuleBase {
public:
    using PendingCallbackQueue = MPSCQueue<std::function<void()>>;

    using ModuleBase::ModuleBase;

    virtual PipelineResult process(const SensorBundle& data) = 0;
    virtual void setInitialState(const State& initial_state) = 0;

    void drainPendingCallbacks() {
        while (auto callback = pending_callbacks_->try_pop()) {
            try {
                (*callback)();
            } catch (const std::exception& e) {
                // Isolate callback exceptions — don't abort remaining callbacks
                if (log_) {
                    log_.error("exception in pending callback: {}", e.what());
                } else {
                    std::fprintf(stderr, "[PipelineBase] exception in pending callback: %s\n",
                                 e.what());
                }
            } catch (...) {
                if (log_) {
                    log_.error("unknown exception in pending callback");
                } else {
                    std::fprintf(stderr, "[PipelineBase] unknown exception in pending callback\n");
                }
            }
        }
    }

    [[nodiscard]] std::shared_ptr<PendingCallbackQueue> getPendingCallbacksQueue() {
        return pending_callbacks_;
    }

private:
    std::shared_ptr<PendingCallbackQueue> pending_callbacks_ =
        std::make_shared<PendingCallbackQueue>();
};

} // namespace SimpleSLAM
