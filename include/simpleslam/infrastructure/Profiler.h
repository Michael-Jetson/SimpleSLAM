#pragma once

#include <chrono>
#include <cstdio>
#include <string>

#include <simpleslam/infrastructure/LogService.h>

namespace SimpleSLAM {

/// RAII scope timer — measures elapsed time from construction to destruction.
///
/// Usage:
///   {
///       ScopeTimer timer("Pipeline::process", log_);
///       // ... do work ...
///   }  // prints "[Pipeline::process] 12.3 ms" on destruction
///
/// Or with the macro:
///   void Pipeline::process() {
///       SIMPLESLAM_PROFILE_SCOPE("Pipeline::process", log_);
///       // ...
///   }
class ScopeTimer {
public:
    ScopeTimer(std::string name, Log log, double warn_threshold_ms = 100.0)
        : name_(std::move(name))
        , log_(std::move(log))
        , warn_threshold_ms_(warn_threshold_ms)
        , start_(std::chrono::steady_clock::now()) {}

    ~ScopeTimer() {
        auto end = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start_).count();
        if (ms >= warn_threshold_ms_) {
            log_.warn("[{}] {:.1f} ms (exceeds {:.0f} ms threshold)",
                      name_, ms, warn_threshold_ms_);
        } else {
            log_.debug("[{}] {:.1f} ms", name_, ms);
        }
    }

    // Non-copyable, non-movable
    ScopeTimer(const ScopeTimer&) = delete;
    ScopeTimer& operator=(const ScopeTimer&) = delete;
    ScopeTimer(ScopeTimer&&) = delete;
    ScopeTimer& operator=(ScopeTimer&&) = delete;

    /// Manual lap: log intermediate time without stopping the timer
    void lap(const std::string& label) {
        auto now = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(now - start_).count();
        log_.debug("[{}::{}] {:.1f} ms", name_, label, ms);
    }

private:
    std::string name_;
    Log log_;
    double warn_threshold_ms_;
    std::chrono::steady_clock::time_point start_;
};

/// Lightweight version without Log dependency (prints to stdout)
class ScopeTimerLite {
public:
    explicit ScopeTimerLite(std::string name)
        : name_(std::move(name))
        , start_(std::chrono::steady_clock::now()) {}

    ~ScopeTimerLite() {
        auto end = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start_).count();
        // Using fprintf to avoid any dependency
        std::fprintf(stderr, "[PROFILE] %s: %.1f ms\n", name_.c_str(), ms);
    }

    ScopeTimerLite(const ScopeTimerLite&) = delete;
    ScopeTimerLite& operator=(const ScopeTimerLite&) = delete;

private:
    std::string name_;
    std::chrono::steady_clock::time_point start_;
};

// Macro helpers for unique variable names
#define SIMPLESLAM_CONCAT_INNER(a, b) a##b
#define SIMPLESLAM_CONCAT(a, b) SIMPLESLAM_CONCAT_INNER(a, b)

/// Profile a scope with Log integration
/// Usage: SIMPLESLAM_PROFILE_SCOPE("name", log_);
#define SIMPLESLAM_PROFILE_SCOPE(name, log) \
    ::SimpleSLAM::ScopeTimer SIMPLESLAM_CONCAT(simpleslam_timer_, __LINE__)(name, log)

/// Profile a scope with custom warning threshold (milliseconds)
/// Usage: SIMPLESLAM_PROFILE_SCOPE_WARN("name", log_, 50.0);
#define SIMPLESLAM_PROFILE_SCOPE_WARN(name, log, threshold_ms) \
    ::SimpleSLAM::ScopeTimer SIMPLESLAM_CONCAT(simpleslam_timer_, __LINE__)(name, log, threshold_ms)

/// Lightweight profiling (no Log dependency, prints to stderr)
/// Usage: SIMPLESLAM_PROFILE_LITE("name");
#define SIMPLESLAM_PROFILE_LITE(name) \
    ::SimpleSLAM::ScopeTimerLite SIMPLESLAM_CONCAT(simpleslam_timer_lite_, __LINE__)(name)

} // namespace SimpleSLAM
