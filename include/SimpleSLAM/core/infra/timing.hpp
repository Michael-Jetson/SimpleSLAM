#pragma once

/// @file timing.hpp
/// 计时系统——RAII 作用域计时器 + 全局统计管理
///
/// 使用方式：
///   1. 直接构造 ScopeTimer：
///      ScopeTimer timer("iekf_update");
///
///   2. 使用宏（自动拼接行号避免命名冲突）：
///      SIMPLESLAM_SCOPED_TIMER("iekf_update");
///
///   3. 关闭时 dump 统计数据：
///      TimingManager::instance().dumpToLog();

#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace simpleslam {

/// 样本滑动窗口上限——防止长时间运行时内存无限增长
/// 10000 帧 x 8 字节/样本 = 80KB/计时项，足够计算准确的百分位数
inline constexpr size_t kMaxSamples = 10000;

/// 单个计时项的统计数据
struct TimingStats {
    uint64_t count = 0;
    double sum_ms = 0.0;
    double min_ms = 1e18;
    double max_ms = 0.0;
    std::deque<double> samples;  // 滑动窗口，头部删除 O(1)，最多 kMaxSamples 个

    double mean() const;

    /// 单独获取某个百分位（内部调用 computePercentiles，仅排序一次）
    double p50() const;
    double p95() const;
    double p99() const;

    /// 一次排序计算全部百分位数——批量输出场景优先用此方法
    void computePercentiles(double& out_p50, double& out_p95, double& out_p99) const;
};

/// 全局计时统计管理器（单例）
class TimingManager final {
public:
    static TimingManager& instance();

    /// 记录一次耗时（单位毫秒）
    void record(const std::string& name, double duration_ms);

    /// 查询指定计时项的统计
    std::optional<TimingStats> stats(const std::string& name) const;

    /// 导出统计数据
    void dumpJson(const std::string& path) const;
    void dumpCsv(const std::string& path) const;
    void dumpToLog() const;

    /// 清空所有统计
    void reset();

private:
    TimingManager() = default;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, TimingStats> stats_;
};

/// RAII 作用域计时器——析构时自动记录到 TimingManager
class ScopeTimer final {
public:
    explicit ScopeTimer(std::string name);
    ~ScopeTimer();

    // 不可拷贝、不可移动（RAII 语义，生命周期绑定作用域）
    ScopeTimer(const ScopeTimer&) = delete;
    ScopeTimer& operator=(const ScopeTimer&) = delete;
    ScopeTimer(ScopeTimer&&) = delete;
    ScopeTimer& operator=(ScopeTimer&&) = delete;

private:
    std::string name_;
    std::chrono::steady_clock::time_point start_;
};

// 两级宏展开确保 __LINE__ 被展开为数字而非字面量 "__LINE__"
#define SIMPLESLAM_TIMER_CONCAT_INNER(a, b) a##b
#define SIMPLESLAM_TIMER_CONCAT(a, b) SIMPLESLAM_TIMER_CONCAT_INNER(a, b)
#define SIMPLESLAM_SCOPED_TIMER(name) \
    ::simpleslam::ScopeTimer SIMPLESLAM_TIMER_CONCAT(_simpleslam_timer_, __LINE__)(name)

}  // namespace simpleslam
