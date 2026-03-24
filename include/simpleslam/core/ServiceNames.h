#pragma once

namespace SimpleSLAM {

/// Centralized service name constants for SystemContext registration.
/// Eliminates string-literal typo risks across the codebase.
namespace service_names {
    constexpr const char* LOG = "log";
    constexpr const char* CLOCK = "clock";
    constexpr const char* TOPIC_BUS = "topic_bus";
    constexpr const char* RESOURCES = "resources";
    constexpr const char* PIPELINE = "pipeline";
    constexpr const char* SENSOR_IO = "sensor_io";
    constexpr const char* SCHEDULER = "scheduler";
} // namespace service_names

} // namespace SimpleSLAM
