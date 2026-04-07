#include <simpleslam/runtime/Scheduler.h>

#include <stdexcept>
#include <utility>

#include <simpleslam/runtime/IFrameEventDrain.h>
#include <simpleslam/runtime/RuntimeState.h>
#include <simpleslam/modules/BundleAssembler.h>
#include <simpleslam/modules/FrameBoundaryHook.h>
#include <simpleslam/modules/PipelineBase.h>
#include <simpleslam/modules/SensorIO.h>
#include <simpleslam/modules/TriggerPolicy.h>
#include <simpleslam/plugins/Initializer.h>

namespace SimpleSLAM {

Scheduler::~Scheduler() = default;

void Scheduler::setSensorIO(std::shared_ptr<SensorIO> sensor_io) {
    sensor_io_ = std::move(sensor_io);
}
void Scheduler::setTriggerPolicy(std::unique_ptr<TriggerPolicy> trigger_policy) {
    trigger_policy_ = std::move(trigger_policy);
}
void Scheduler::setBundleAssembler(std::unique_ptr<BundleAssembler> bundle_assembler) {
    bundle_assembler_ = std::move(bundle_assembler);
}
void Scheduler::setInitializer(std::unique_ptr<Initializer> initializer) {
    initializer_ = std::move(initializer);
}
void Scheduler::setPipeline(std::shared_ptr<PipelineBase> pipeline) {
    pipeline_ = std::move(pipeline);
}
void Scheduler::setEventDrain(std::shared_ptr<IFrameEventDrain> event_drain) {
    event_drain_ = std::move(event_drain);
}
void Scheduler::setTopicBus(std::shared_ptr<IFrameEventDrain> topic_bus) {
    event_drain_ = std::move(topic_bus);
}
void Scheduler::setRuntimeState(std::shared_ptr<RuntimeState> runtime_state) {
    runtime_state_ = std::move(runtime_state);
    if (runtime_state_) {
        runtime_state_->setPhase(state_);
        runtime_state_->setRunning(running_.load(std::memory_order_relaxed));
    }
}
void Scheduler::addHook(std::shared_ptr<FrameBoundaryHook> hook) {
    hooks_.push_back(std::move(hook));
}

void Scheduler::stop() {
    running_.store(false, std::memory_order_relaxed);
    if (runtime_state_) {
        runtime_state_->setRunning(false);
    }
    if (sensor_io_) {
        sensor_io_->requestStop();  // unblock receive() in online mode
    }
}

void Scheduler::validateConfiguration() const {
    if (!sensor_io_) {
        throw std::logic_error("Scheduler requires SensorIO before run()");
    }
    if (!trigger_policy_) {
        throw std::logic_error("Scheduler requires TriggerPolicy before run()");
    }
    if (!bundle_assembler_) {
        throw std::logic_error("Scheduler requires BundleAssembler before run()");
    }
    if (!pipeline_) {
        throw std::logic_error("Scheduler requires PipelineBase before run()");
    }
    if (state_ == PipelineState::INITIALIZING && !initializer_) {
        throw std::logic_error("Scheduler requires Initializer while in INITIALIZING state");
    }
}

void Scheduler::run() {
    validateConfiguration();
    running_.store(true, std::memory_order_relaxed);
    if (runtime_state_) {
        runtime_state_->setRunning(true);
        runtime_state_->setPhase(state_);
    }

    while (running_.load(std::memory_order_relaxed)) {
        if (offline_mode_ && !sensor_io_->hasMore()) {
            break;
        }

        SensorData data = sensor_io_->receive();

        const bool should_trigger = trigger_policy_->shouldTrigger(data);
        bundle_assembler_->addData(std::move(data));
        if (!should_trigger) {
            continue;
        }

        auto bundle = bundle_assembler_->assemble();
        if (!bundle) {
            continue;
        }

        for (const auto& hook : hooks_) {
            if (hook) {
                hook->onFrameStart(*bundle);
            }
        }

        PipelineResult result{};
        bool exit_after_frame = false;

        if (state_ == PipelineState::INITIALIZING) {
            const InitResult init_result = initializer_->step(*bundle);

            if (init_result == InitResult::Ready) {
                pipeline_->setInitialState(initializer_->getInitialState());
                state_ = PipelineState::TRACKING;
                if (runtime_state_) {
                    runtime_state_->setPhase(state_);
                }
                log_.info("Initialization complete (progress={:.1f}%), entering TRACKING",
                          initializer_->progress() * 100.0);
                initializer_.reset();  // 不再需要，释放内存
            } else if (init_result == InitResult::Failed) {
                log_.error("Initialization failed, stopping");
                result.status = PipelineResult::FAILED;
                result.failure_reason = "Initialization failed";
                exit_after_frame = true;
            } else {
                // InitResult::Continue — still initializing, mark result accordingly
                result.status = PipelineResult::DEGRADED;
                result.failure_reason = "Initializing";
            }

            if (event_drain_) {
                event_drain_->drainOneRound();
            }

            for (const auto& hook : hooks_) {
                if (hook) {
                    hook->onFrameEnd(result);
                }
            }

            if (exit_after_frame) {
                break;
            }
            continue;
        }

        if (state_ == PipelineState::LOST) {
            // Phase 3+: attempt re-initialization or relocalization
            // For now, still drain pending callbacks (backend may inject recovery commands)
            // and mark result as LOST so hooks see the correct state
            pipeline_->drainPendingCallbacks();
            result.status = PipelineResult::LOST;
            result.failure_reason = "System in LOST state, frame skipped";
            log_.warn("System in LOST state, skipping frame processing");
        } else {
            // TRACKING state: process frame
            pipeline_->drainPendingCallbacks();
            try {
                result = pipeline_->process(*bundle);
            } catch (const std::exception& e) {
                result.status = PipelineResult::FAILED;
                result.failure_reason = e.what();
                log_.error("Exception in pipeline->process(): {}", e.what());
            } catch (...) {
                result.status = PipelineResult::FAILED;
                result.failure_reason = "Unknown exception in pipeline->process()";
                log_.error("Unknown exception in pipeline->process()");
            }
            pipeline_->drainPendingCallbacks();
        }

        // Handle LOST/FAILED → transition to LOST state
        if (result.status == PipelineResult::LOST ||
            result.status == PipelineResult::FAILED) {
            state_ = PipelineState::LOST;
            if (runtime_state_) {
                runtime_state_->setPhase(state_);
            }
            log_.warn("Pipeline reported {}, entering LOST state",
                      result.status == PipelineResult::LOST ? "LOST" : "FAILED");
            // Phase 3+: trigger re-initialization or relocalization
        }

        if (event_drain_) {
            event_drain_->drainOneRound();
        }

        for (const auto& hook : hooks_) {
            if (hook) {
                hook->onFrameEnd(result);
            }
        }
    }

    // Flush remaining pending events on exit (BFS tail events)
    if (event_drain_) {
        event_drain_->drainOneRound();
    }

    running_.store(false, std::memory_order_relaxed);
    if (runtime_state_) {
        runtime_state_->setRunning(false);
        runtime_state_->setPhase(state_);
    }
}

} // namespace SimpleSLAM
