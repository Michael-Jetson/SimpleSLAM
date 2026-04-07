#include <simpleslam/runtime/BuildSystem.h>

#include <memory>
#include <utility>

#include <simpleslam/core/ServiceNames.h>
#include <simpleslam/infrastructure/DatasetClock.h>
#include <simpleslam/infrastructure/SystemClock.h>
#include <simpleslam/infrastructure/TopicBus.h>
#include <simpleslam/runtime/RuntimeState.h>

namespace SimpleSLAM {

BuiltSystem buildSystem(const SystemConfig& config) {
    BuiltSystem system;

    system.context = std::make_shared<SystemContext>();
    system.log_service = std::make_shared<LogService>(config.logging);
    system.resource_hub = std::make_shared<ResourceHub>();
    auto topic_bus = std::make_shared<TopicBus>();
    system.event_drain = topic_bus;
    system.runtime_state = std::make_shared<RuntimeState>();
    system.scheduler = std::make_shared<Scheduler>();

    if (config.runtime.use_dataset_clock) {
        system.clock = std::make_shared<DatasetClock>(config.runtime.dataset_clock_speed);
    } else {
        system.clock = std::make_shared<SystemClock>();
    }

    system.scheduler->setLog(system.log_service->getLogger("scheduler"));
    system.scheduler->setEventDrain(system.event_drain);
    system.scheduler->setOfflineMode(config.runtime.offline_mode);
    system.scheduler->setRuntimeState(system.runtime_state);

    system.context->registerService(service_names::LOG, system.log_service);
    system.context->registerService(service_names::RESOURCES, system.resource_hub);
    system.context->registerService(service_names::TOPIC_BUS, topic_bus);
    system.context->registerService(service_names::CLOCK, system.clock);
    system.context->registerService(service_names::RUNTIME_STATE, system.runtime_state);
    system.context->registerService(service_names::SCHEDULER, system.scheduler);

    return system;
}

void BuiltSystem::requestStopAndFlush() noexcept {
    if (scheduler) {
        scheduler->stop();
    }
    if (runtime_state) {
        runtime_state->setRunning(false);
    }
    // Do not force an extra event drain here unless the caller has already
    // ensured the scheduler loop has exited; frame-boundary drain ordering
    // is owned by Scheduler.
    if (log_service) {
        log_service->flush();
    }
}

} // namespace SimpleSLAM
