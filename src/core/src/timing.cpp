/// @file timing.cpp
/// 计时系统实现

#include <SimpleSLAM/core/infra/timing.hpp>
#include <SimpleSLAM/core/infra/logger.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

#include <fmt/format.h>

namespace simpleslam {

// ── TimingStats ──

double TimingStats::mean() const {
    return count > 0 ? sum_ms / static_cast<double>(count) : 0.0;
}

/// 对排好序的数据计算线性插值百分位数
static double percentileFromSorted(const std::vector<double>& sorted, double p) {
    if (sorted.empty()) return 0.0;
    double rank = p / 100.0 * static_cast<double>(sorted.size() - 1);
    auto lo = static_cast<size_t>(std::floor(rank));
    auto hi = static_cast<size_t>(std::ceil(rank));
    if (lo == hi) return sorted[lo];
    double frac = rank - static_cast<double>(lo);
    return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

void TimingStats::computePercentiles(double& out_p50, double& out_p95, double& out_p99) const {
    // deque → vector 拷贝一次，排序一次，计算三个百分位
    std::vector<double> sorted(samples.begin(), samples.end());
    std::sort(sorted.begin(), sorted.end());
    out_p50 = percentileFromSorted(sorted, 50.0);
    out_p95 = percentileFromSorted(sorted, 95.0);
    out_p99 = percentileFromSorted(sorted, 99.0);
}

// 单独调用委托给 computePercentiles，只排序一次
double TimingStats::p50() const { double v, _1, _2; computePercentiles(v, _1, _2); return v; }
double TimingStats::p95() const { double _1, v, _2; computePercentiles(_1, v, _2); return v; }
double TimingStats::p99() const { double _1, _2, v; computePercentiles(_1, _2, v); return v; }

// ── TimingManager ──

TimingManager& TimingManager::instance() {
    static TimingManager mgr;
    return mgr;
}

void TimingManager::record(const std::string& name, double duration_ms) {
    std::lock_guard lock(mutex_);
    auto& s = stats_[name];
    s.count++;
    s.sum_ms += duration_ms;
    s.min_ms = std::min(s.min_ms, duration_ms);
    s.max_ms = std::max(s.max_ms, duration_ms);

    // 滑动窗口：deque 头部删除 O(1)，防止长时间运行内存无限增长
    if (s.samples.size() >= kMaxSamples) {
        s.samples.pop_front();
    }
    s.samples.push_back(duration_ms);
}

std::optional<TimingStats> TimingManager::stats(const std::string& name) const {
    std::lock_guard lock(mutex_);
    if (auto it = stats_.find(name); it != stats_.end()) {
        return it->second;
    }
    return std::nullopt;
}

void TimingManager::dumpJson(const std::string& path) const {
    std::lock_guard lock(mutex_);
    std::ofstream out(path);
    if (!out.is_open()) {
        // 用 stderr 而非 Logger，因为此处持有 mutex_，调用 Logger 可能间接触发 record() 死锁
        fprintf(stderr, "[SimpleSLAM][timing] Cannot open file: %s\n", path.c_str());
        return;
    }

    out << "{\n";
    bool first = true;
    for (const auto& [name, s] : stats_) {
        double p50_val, p95_val, p99_val;
        s.computePercentiles(p50_val, p95_val, p99_val);

        if (!first) out << ",\n";
        first = false;
        out << fmt::format(
            "  \"{}\": {{\n"
            "    \"count\": {},\n"
            "    \"mean_ms\": {:.3f},\n"
            "    \"min_ms\": {:.3f},\n"
            "    \"max_ms\": {:.3f},\n"
            "    \"p50_ms\": {:.3f},\n"
            "    \"p95_ms\": {:.3f},\n"
            "    \"p99_ms\": {:.3f}\n"
            "  }}",
            name, s.count, s.mean(), s.min_ms, s.max_ms, p50_val, p95_val, p99_val);
    }
    out << "\n}\n";
}

void TimingManager::dumpCsv(const std::string& path) const {
    std::lock_guard lock(mutex_);
    std::ofstream out(path);
    if (!out.is_open()) {
        // 用 stderr 而非 Logger，因为此处持有 mutex_，调用 Logger 可能间接触发 record() 死锁
        fprintf(stderr, "[SimpleSLAM][timing] Cannot open file: %s\n", path.c_str());
        return;
    }

    out << "name,count,mean_ms,min_ms,max_ms,p50_ms,p95_ms,p99_ms\n";
    for (const auto& [name, s] : stats_) {
        double p50_val, p95_val, p99_val;
        s.computePercentiles(p50_val, p95_val, p99_val);

        out << fmt::format("{},{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n",
                           name, s.count, s.mean(), s.min_ms, s.max_ms,
                           p50_val, p95_val, p99_val);
    }
}

void TimingManager::dumpToLog() const {
    std::lock_guard lock(mutex_);
    auto log = Logger::get("timing");

    log->info("=== 计时统计 ===");
    for (const auto& [name, s] : stats_) {
        double p50_val, p95_val, p99_val;
        s.computePercentiles(p50_val, p95_val, p99_val);

        log->info("  {:30s}  count={:6d}  mean={:8.2f}ms  "
                  "min={:8.2f}ms  max={:8.2f}ms  "
                  "p50={:8.2f}ms  p95={:8.2f}ms  p99={:8.2f}ms",
                  name, s.count, s.mean(), s.min_ms, s.max_ms,
                  p50_val, p95_val, p99_val);
    }
}

void TimingManager::reset() {
    std::lock_guard lock(mutex_);
    stats_.clear();
}

// ── ScopeTimer ──

ScopeTimer::ScopeTimer(std::string name)
    : name_(std::move(name)), start_(std::chrono::steady_clock::now()) {}

ScopeTimer::~ScopeTimer() {
    auto elapsed = std::chrono::steady_clock::now() - start_;
    double ms = std::chrono::duration<double, std::milli>(elapsed).count();
    TimingManager::instance().record(name_, ms);
}

}  // namespace simpleslam
