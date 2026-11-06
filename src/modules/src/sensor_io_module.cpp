// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/modules/sensor_io_module.hpp>

#include <utility>

namespace simpleslam::modules {

SensorIoModule::PumpResult SensorIoModule::pump(std::size_t max_packets, std::size_t max_samples) {
    PumpResult result;
    if (inputsStopped() || stopped()) {
        result.stopped = true;
        return result;
    }
    const std::size_t sample_budget =
        max_samples > 0 ? max_samples : kSamplesPerPacket * (max_packets + 1);
    // 预算在生成之前检查：一条 IMU 可能同时让好几帧凑齐，一步只该发预算允许的那几个
    const auto emitReady = [&](bool at_eof) {
        if (result.published >= max_packets) return false;
        auto packet = at_eof ? packetizer_.flush() : packetizer_.poll();
        if (!packet) return false;
        packet_pub_.publish(std::move(*packet));
        ++published_;
        ++result.published;
        return true;
    };
    for (;;) {
        if (inputsStopped()) {
            result.stopped = true;
            return result;
        }
        if (source_exhausted_) {
            while (emitReady(true)) {
            }
            if (packetizer_.waitingForWatermark()) {
                result.yielded = true;  // 预算用完：剩下的下一步接着排空
                return result;
            }
            markFinished();
            result.finished = true;
            return result;
        }
        while (emitReady(false)) {
        }
        if (result.published >= max_packets) return result;
        if (result.samples >= sample_budget) {
            result.yielded = true;
            return result;
        }
        if (!pending_) {
            auto datum = source_->next();
            ++result.samples;
            if (!datum) {
                // 来源读完：不再调用 next()，剩下的触发帧按"已有多少 IMU 就声明多少"
                // 逐帧发出，缺支撑如实标缺口
                source_exhausted_ = true;
                outcome_ = source_->outcome();
                continue;
            }
            pending_ = std::move(datum);
        }
        if (pacing_ == Pacing::kDataTime && !due(*pending_)) {
            result.yielded = true;  // 还没到时候：留着，定时器下一次再看
            return result;
        }
        packetizer_.ingest(*pending_);
        pending_.reset();
    }
}

bool SensorIoModule::due(const fw::SensorDatum& datum) {
    const auto time = fw::dataTime(datum);
    if (!time) return true;
    const comm::Time now = clock().now();
    if (!origin_) origin_ = ReplayOrigin{.wall = now, .data = *time};
    // 时刻倒退的数据（来源乱序）立刻放行：顺序由来源负责，节奏只管不提前
    return now - origin_->wall >= *time - origin_->data;
}

void SensorIoModule::markFinished() {
    if (finished_) return;
    finished_ = true;
    if (on_finished_) on_finished_();
}

void SensorIoModule::startPumping(comm::Duration period, std::size_t packets_per_tick,
                                  Pacing pacing, std::function<void()> on_finished) {
    if (stopped()) return;
    pacing_ = pacing;
    on_finished_ = std::move(on_finished);
    pump_timer_ = createTimer(period, [this, packets_per_tick] { pump(packets_per_tick); });
}

void SensorIoModule::onInit() {
    packet_pub_ = advertise(topics::kSensorPacket);
    // 消费进度是累计的，只有最新值有用
    subscribe(topics::kPacketConsumed, 1, &SensorIoModule::onConsumed);
    (void)resources_;
}

void SensorIoModule::onConsumed(const fw::PacketConsumed& event) {
    packetizer_.notifyConsumedThrough(event.through);
    consumed_ = event.through;
}

}  // namespace simpleslam::modules
