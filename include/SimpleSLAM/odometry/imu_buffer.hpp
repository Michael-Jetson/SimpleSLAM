#pragma once

/// @file imu_buffer.hpp
/// 线程安全 IMU 数据缓冲区——高频追加、按时间窗查询、老数据自动裁剪
///
/// 里程计前端使用：传感器线程写入 IMU 样本，里程计线程按时间窗查询。
/// 查询支持边界线性插值，确保预积分获得精确的起止时间样本。

#include <SimpleSLAM/core/types/sensor_data.hpp>

#include <algorithm>
#include <cassert>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

namespace simpleslam {

class ImuBuffer final {
public:
    explicit ImuBuffer(size_t max_size = 10000) : max_size_(max_size) {
        assert(max_size > 0);
    }

    void addSample(const ImuSample& sample) {
        std::lock_guard lock(mutex_);
        assert(buffer_.empty() || sample.timestamp >= buffer_.back().timestamp);
        buffer_.push_back(sample);
        trimToCapacity();
    }

    void addBatch(const ImuBatch& batch) {
        std::lock_guard lock(mutex_);
        for (const auto& s : batch) {
            assert(buffer_.empty() || s.timestamp >= buffer_.back().timestamp);
            buffer_.push_back(s);
        }
        trimToCapacity();
    }

    [[nodiscard]] std::vector<ImuSample> query(Timestamp t_start, Timestamp t_end) const {
        assert(t_start <= t_end);
        std::lock_guard lock(mutex_);

        if (buffer_.empty()) return {};

        auto it_begin = std::lower_bound(
            buffer_.begin(), buffer_.end(), t_start,
            [](const ImuSample& s, Timestamp t) { return s.timestamp < t; });
        auto it_end = std::upper_bound(
            buffer_.begin(), buffer_.end(), t_end,
            [](Timestamp t, const ImuSample& s) { return t < s.timestamp; });

        if (it_begin == it_end && it_begin == buffer_.end()) return {};

        std::vector<ImuSample> result;

        if (it_begin != buffer_.begin() && (it_begin == buffer_.end() || it_begin->timestamp > t_start)) {
            auto prev = std::prev(it_begin);
            if (it_begin != buffer_.end()) {
                result.push_back(interpolateBetween(*prev, *it_begin, t_start));
            }
        }

        for (auto it = it_begin; it != it_end; ++it) {
            result.push_back(*it);
        }

        if (it_end != buffer_.end() && it_end != buffer_.begin()) {
            auto prev = std::prev(it_end);
            if (prev->timestamp < t_end) {
                result.push_back(interpolateBetween(*prev, *it_end, t_end));
            }
        }

        return result;
    }

    [[nodiscard]] std::optional<ImuSample> interpolateAt(Timestamp t) const {
        std::lock_guard lock(mutex_);

        if (buffer_.size() < 2) return std::nullopt;
        if (t < buffer_.front().timestamp || t > buffer_.back().timestamp) return std::nullopt;

        auto it = std::lower_bound(
            buffer_.begin(), buffer_.end(), t,
            [](const ImuSample& s, Timestamp ts) { return s.timestamp < ts; });

        if (it != buffer_.end() && it->timestamp == t) return *it;
        if (it == buffer_.begin() || it == buffer_.end()) return std::nullopt;

        return interpolateBetween(*std::prev(it), *it, t);
    }

    void trimBefore(Timestamp t) {
        std::lock_guard lock(mutex_);
        while (!buffer_.empty() && buffer_.front().timestamp < t) {
            buffer_.pop_front();
        }
    }

    [[nodiscard]] size_t size() const {
        std::lock_guard lock(mutex_);
        return buffer_.size();
    }

    [[nodiscard]] bool empty() const {
        std::lock_guard lock(mutex_);
        return buffer_.empty();
    }

    [[nodiscard]] std::optional<Timestamp> oldestTimestamp() const {
        std::lock_guard lock(mutex_);
        if (buffer_.empty()) return std::nullopt;
        return buffer_.front().timestamp;
    }

    [[nodiscard]] std::optional<Timestamp> newestTimestamp() const {
        std::lock_guard lock(mutex_);
        if (buffer_.empty()) return std::nullopt;
        return buffer_.back().timestamp;
    }

private:
    void trimToCapacity() {
        while (buffer_.size() > max_size_) {
            buffer_.pop_front();
        }
    }

    static ImuSample interpolateBetween(const ImuSample& a, const ImuSample& b,
                                         Timestamp t) {
        double dt = b.timestamp - a.timestamp;
        double alpha = (dt > 0.0) ? (t - a.timestamp) / dt : 0.0;
        ImuSample result;
        result.timestamp = t;
        result.acc = (1.0 - alpha) * a.acc + alpha * b.acc;
        result.gyro = (1.0 - alpha) * a.gyro + alpha * b.gyro;
        return result;
    }

    mutable std::mutex mutex_;
    std::deque<ImuSample> buffer_;
    size_t max_size_;
};

}  // namespace simpleslam
