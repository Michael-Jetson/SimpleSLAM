#pragma once

/// @file clock.hpp
/// 时钟抽象——支持系统时钟和数据集模拟时钟
///
/// SystemClock：基于 steady_clock 的单调时钟（返回进程启动后的相对时间）
/// DatasetClock：由外部（Runner）手动推进的模拟时钟，用于离线模式

#include <SimpleSLAM/core/types/common.hpp>

#include <atomic>
#include <chrono>

namespace simpleslam {

/// 时钟接口——只提供 now()，不负责 sleep（那是调度器的事）
class IClock {
public:
    virtual ~IClock() = default;
    virtual Timestamp now() const = 0;
};

/// 系统单调时钟（返回进程启动后的相对秒数）
/// 选用 steady_clock 而非 system_clock，因为：
///   - steady_clock 单调递增，不受 NTP 跳变影响
///   - SLAM 运行时需要的是稳定的时间差，不是绝对 UTC 时间
///   - 传感器时间戳（UTC 纪元）由数据源提供，不由系统时钟决定
class SystemClock final : public IClock {
public:
    SystemClock() : start_(std::chrono::steady_clock::now()) {}

    Timestamp now() const override {
        auto elapsed = std::chrono::steady_clock::now() - start_;
        return std::chrono::duration<double>(elapsed).count();
    }

private:
    std::chrono::steady_clock::time_point start_;
};

/// 数据集时钟——离线模式下由 Runner 按数据时间戳推进
///
/// 线程安全：使用 atomic<double> 保证 advance() 和 now() 可以在不同线程调用。
/// 但不保证多线程同时 advance() 的时序一致性（调用方需自行保证顺序）。
class DatasetClock final : public IClock {
public:
    Timestamp now() const override {
        return current_time_.load(std::memory_order_acquire);
    }

    /// 推进到指定时间戳（通常是当前帧的传感器时间戳）
    void advance(Timestamp t) {
        current_time_.store(t, std::memory_order_release);
    }

private:
    std::atomic<double> current_time_{0.0};
};

}  // namespace simpleslam
