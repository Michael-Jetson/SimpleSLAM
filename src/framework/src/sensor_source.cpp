// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/framework/sensor_source.hpp>

#include <algorithm>
#include <utility>

namespace simpleslam::fw {

// 附件队列的三个模板只在本文件里对图像与 GNSS 实例化。

template <class T>
void Packetizer::pushAttachment(std::deque<Ptr<T>>& queue, const Ptr<T>& item) {
    queue.push_back(item);
    observe(timeOf(*item));
    if (queue.size() > policy_.max_pending_attachments) {
        queue.pop_front();
        ++attachment_drops_;
    }
}

template <class T>
std::vector<Ptr<T>> Packetizer::takeNear(std::deque<Ptr<T>>& queue, Timestamp reference_time) {
    std::vector<Ptr<T>> taken;
    while (!queue.empty()) {
        const Timestamp t = timeOf(*queue.front());
        if (t > reference_time + policy_.attachment_tolerance) break;
        if (t + policy_.attachment_tolerance >= reference_time) taken.push_back(queue.front());
        queue.pop_front();
    }
    return taken;
}

template <class T>
void Packetizer::pruneStale(std::deque<Ptr<T>>& queue, Timestamp reference_time) {
    while (!queue.empty()
           && timeOf(*queue.front()) + policy_.attachment_tolerance < reference_time) {
        queue.pop_front();
    }
}

std::optional<SensorPacket> Packetizer::feed(const SensorDatum& datum) {
    accept(datum);
    return poll();
}

std::optional<SensorPacket> Packetizer::poll() {
    while (frontReady()) {
        if (!overloaded()) return buildFront();
        dropFront();
    }
    return std::nullopt;
}

std::optional<SensorPacket> Packetizer::flush() {
    if (pending_triggers_.empty()) return std::nullopt;
    return buildFront();
}

void Packetizer::accept(const SensorDatum& datum) {
    if (const auto* imu = std::get_if<Ptr<ImuSample>>(&datum)) {
        if (*imu && acceptDomain((*imu)->domain)) pushImu(*imu);
        return;
    }
    if (const auto* image = std::get_if<Ptr<CameraFrame>>(&datum)) {
        if (!*image || !acceptDomain((*image)->domain)) return;
        if (policy_.trigger_on_lidar) {
            pushAttachment(pending_images_, *image);
        } else {
            setTrigger(Trigger{*image}, (*image)->capture_time);
        }
        return;
    }
    if (const auto* gnss = std::get_if<Ptr<GnssFix>>(&datum)) {
        if (*gnss && acceptDomain((*gnss)->domain)) pushAttachment(pending_gnss_, *gnss);
        return;
    }
    if (const auto* scan = std::get_if<Ptr<LidarScan>>(&datum)) {
        if (!*scan || !acceptDomain((*scan)->domain)) return;
        if (policy_.trigger_on_lidar) setTrigger(Trigger{*scan}, (*scan)->reference_time);
    }
}

bool Packetizer::acceptDomain(ClockDomainId domain) {
    if (domain == policy_.clock_domain) return true;
    ++foreign_clock_drops_;
    pending_gap_ = GapReason::kClockDomain;
    return false;
}

void Packetizer::pushImu(const Ptr<ImuSample>& sample) {
    if (!imu_.empty() && sample->time <= imu_.back()->time) {
        // 时间倒流或重复：丢掉这条并显式记缺口。取区间的算法依赖样本有序，
        // 乱序样本进去之后左右括点都可能找错。
        ++imu_out_of_order_;
        pending_gap_ = GapReason::kClockReset;
        return;
    }
    imu_.push_back(sample);
    observe(sample->time);
    if (imu_.size() > policy_.max_imu_backlog) {
        imu_.pop_front();
        ++imu_backlog_drops_;
        pending_gap_ = GapReason::kImuGap;  // 有界内存下"永不丢"只在声明容量内成立
    }
}

void Packetizer::observe(Timestamp time) noexcept {
    if (!first_observed_) first_observed_ = time;
    observed_until_ = std::max(observed_until_, time);
}

void Packetizer::setTrigger(Trigger trigger, Timestamp reference_time) {
    const Timestamp last = pending_triggers_.empty() ? generated_until_
                                                     : pending_triggers_.back().reference_time;
    if (sequence_ > 0 || !pending_triggers_.empty()) {
        if (reference_time <= last) {
            ++rejected_triggers_;
            pending_gap_ = GapReason::kClockReset;  // 触发帧时刻不前进：拒收
            return;
        }
    }
    pending_triggers_.push_back(PendingTrigger{std::move(trigger), reference_time});
    observe(reference_time);
    if (pending_triggers_.size() > policy_.max_pending_triggers) {
        dropFront();  // 等括点的触发帧太多
    }
}

bool Packetizer::frontReady() const {
    if (pending_triggers_.empty()) return false;
    if (policy_.imu != ImuRequirement::kRequired) return true;
    const Timestamp watermark = pending_triggers_.front().reference_time + policy_.post_support;
    return !imu_.empty() && imu_.back()->time >= watermark;
}

bool Packetizer::overloaded() const noexcept {
    return policy_.max_in_flight > 0 && inFlight() >= policy_.max_in_flight;
}

void Packetizer::dropFront() {
    pending_triggers_.pop_front();
    ++dropped_triggers_;
    ++dropped_since_packet_;
}

SensorPacket Packetizer::buildFront() {
    auto front = std::move(pending_triggers_.front());
    pending_triggers_.pop_front();
    return buildPacket(std::move(front.trigger), front.reference_time);
}

SensorPacket Packetizer::buildPacket(Trigger trigger, Timestamp reference_time) {
    SensorPacket packet;
    packet.sequence = ++sequence_;
    packet.trigger = std::move(trigger);
    packet.reference_time = reference_time;
    packet.time_offset_support = policy_.post_support;

    // 需求由触发帧与生成水位决定，不由现有数据反推。状态传播从上一包的参考时刻起算；
    // 首包没有这个时刻，从本帧采集起点起算——拿"现有的第一条 IMU"当起点，等于用数据
    // 的缺失来定义需求，缺口就永远不会被发现。
    // 去畸变还要覆盖整帧采集区间，所以支撑的需求是两者的并集。
    const Timestamp capture_begin =
        std::min(captureBegin(packet.trigger, reference_time), reference_time);
    const Timestamp propagation_begin =
        has_generated_ ? std::min(generated_until_, reference_time) : capture_begin;
    const Timestamp end = reference_time + policy_.post_support;
    packet.propagation = TimeRange{propagation_begin, end};
    packet.imu_required = TimeRange{std::min(propagation_begin, capture_begin), end};

    GapReason gap = pending_gap_;
    std::vector<ImuSample> taken = takeImuSpan(packet.imu_required.begin, end);

    Timestamp declared_begin = packet.imu_required.begin;
    Timestamp declared_end = end;
    if (taken.empty()) {
        declared_begin = reference_time;
        declared_end = reference_time;
        if (policy_.imu == ImuRequirement::kRequired && gap == GapReason::kNone) {
            gap = GapReason::kImuGap;
        }
    } else {
        if (taken.front().time > declared_begin) {
            declared_begin = taken.front().time;
            gap = GapReason::kImuGap;  // 左边界插值不了：如实收窄并标缺口
        }
        if (taken.back().time < declared_end) {
            declared_end = taken.back().time;
            gap = GapReason::kImuGap;
        }
        if (declared_end < declared_begin) declared_end = declared_begin;
    }
    // 两端有样本不等于中间连续：量出相邻样本的最大间隔，超过要求就标缺口
    for (std::size_t i = 1; i < taken.size(); ++i) {
        const Duration step = taken[i].time - taken[i - 1].time;
        if (step > packet.imu_max_gap) packet.imu_max_gap = step;
    }
    if (policy_.max_imu_gap.count() > 0 && packet.imu_max_gap > policy_.max_imu_gap) {
        gap = GapReason::kImuGap;
    }
    packet.imu_coverage = TimeRange{declared_begin, declared_end};
    packet.imu = Batch<ImuSample>(std::move(taken));

    packet.attachments.images = takeNear(pending_images_, reference_time);
    packet.attachments.gnss = takeNear(pending_gnss_, reference_time);
    // 早于"本包时刻 - 容差"的附件再也附不上了：清掉，不让它们无界堆积
    pruneStale(pending_images_, reference_time);
    pruneStale(pending_gnss_, reference_time);
    packet.gap = gap;
    packet.dropped_triggers = dropped_since_packet_;
    dropped_since_packet_ = 0;

    pending_gap_ = GapReason::kNone;
    generated_until_ = reference_time;
    has_generated_ = true;
    trimImu();
    return packet;
}

std::vector<ImuSample> Packetizer::takeImuSpan(Timestamp begin, Timestamp end) const {
    std::vector<ImuSample> taken;
    if (imu_.empty()) return taken;

    std::size_t lo = 0;
    for (std::size_t i = 0; i < imu_.size(); ++i) {
        if (imu_[i]->time <= begin) {
            lo = i;  // 最后一个不晚于起点的样本：起点插值的左端
        } else {
            break;
        }
    }
    std::size_t hi = imu_.size() - 1;
    for (std::size_t i = lo; i < imu_.size(); ++i) {
        if (imu_[i]->time >= end) {
            hi = i;  // 第一个不早于终点的样本：终点插值的右端
            break;
        }
    }
    if (hi < lo) hi = lo;
    taken.reserve(hi - lo + 1);
    for (std::size_t i = lo; i <= hi; ++i) taken.push_back(*imu_[i]);
    return taken;
}

void Packetizer::trimImu() {
    const Timestamp keep_from = generated_until_ - policy_.pre_support;
    while (imu_.size() >= 2 && imu_[1]->time <= keep_from) {
        imu_.pop_front();
    }
}

Timestamp Packetizer::captureBegin(const Trigger& trigger, Timestamp reference_time) {
    if (const auto* scan = std::get_if<Ptr<LidarScan>>(&trigger); scan != nullptr && *scan) {
        if (!(*scan)->coverage.empty()) return (*scan)->coverage.begin;
    }
    return reference_time;
}

}  // namespace simpleslam::fw
