#pragma once

#include <memory>

#include <simpleslam/config/SystemConfig.h>
#include <simpleslam/infrastructure/IClock.h>
#include <simpleslam/infrastructure/LogService.h>
#include <simpleslam/runtime/IFrameEventDrain.h>
#include <simpleslam/runtime/ResourceHub.h>
#include <simpleslam/runtime/RuntimeState.h>
#include <simpleslam/runtime/Scheduler.h>
#include <simpleslam/runtime/SystemContext.h>

namespace SimpleSLAM {

struct BuiltSystem {
    std::shared_ptr<SystemContext> context;
    std::shared_ptr<LogService> log_service;
    std::shared_ptr<ResourceHub> resource_hub;
    std::shared_ptr<IFrameEventDrain> event_drain;
    std::shared_ptr<IClock> clock;
    std::shared_ptr<RuntimeState> runtime_state;
    std::shared_ptr<Scheduler> scheduler;

    // Temporary lifecycle helper for infrastructure-only composition.
    // This only requests stop and flushes logging. Caller must ensure the
    // scheduler loop has actually exited before destroying the built system.
    void requestStopAndFlush() noexcept;
};

[[nodiscard]] BuiltSystem buildSystem(const SystemConfig& config);

} // namespace SimpleSLAM
