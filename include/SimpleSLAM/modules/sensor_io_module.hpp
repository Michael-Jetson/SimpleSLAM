// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file sensor_io_module.hpp
/// 传感器接入模块：来源 → 规范化 → 打包 → 发布。
///
/// 离线由 Runner 调 pump() 步进，在线由定时器驱动同一个 pump()，
/// 模块代码只有一套。EOF 是显式状态，不靠超时判断。
///
/// 发布结果与消费确认分别推进打包器的接纳水位与消费水位：
/// 被拒的包不推进接纳水位（那段 IMU 会并进下一包），
/// 没被确认消费的原始数据不回收（sensor_source.hpp 的三条水位线）。
///
/// 一条接线契约：**原始包的可靠订阅者只能有一个**——主消费者（里程计）。
/// 发布结果是所有订阅端的汇总，只有在"可靠档只有一个"时，"发布被拒"才等价于
/// "主消费者拒收"。否则一个观察者满载就会让上游把整包当成没送到，
/// 再把同一段积分区间并进下一包，而主消费者其实已经收到了。
/// 观察者请订阅 /slam/odometry 或诊断流，或用 kDropOldest / kLatest（它们不拒收）。
///
/// ## 三条纪律
///
/// 1. **发布成功的判据是"主消费者确实接纳"**，不是"没人满载"。话题在停止阶段
///    被关掉时后者仍然为真（关闭且零订阅者时所有计数都是 0），接纳水位照常推进，
///    而下游一个包也没收到。判据用 `deliveredTo(1)`。
/// 2. **pump 的预算按尝试计，不按成功计**。只在发布成功时加计数的话，下游满载时
///    循环会继续从来源读，单工作线程配置下消费者永远拿不到执行机会，pump 永不返回。
///    遇到拒收就带着可诊断的结果返回，让出执行权。预算约束**所有**出包路径：
///    一条 IMU 可能同时让好几帧凑齐括点，EOF 时也可能还有好几帧在等——预算用完就
///    停在"生成下一包之前"，已就绪的帧留在打包器里，下一步先发它们再读来源。
///    不能先生成下一包再判断预算：那会留下一个没有接纳或拒收结论的悬空包。
/// 3. **来源的终态要传上来**。回放一个被截断的录制文件时，`next()` 返回空与正常
///    读完长得一模一样；不完整必须可见，否则严格评测会拿一段残缺轨迹下结论。

#include <SimpleSLAM/comm/module.hpp>
#include <SimpleSLAM/framework/sensor_source.hpp>
#include <SimpleSLAM/modules/topics.hpp>

#include <memory>
#include <string>
#include <utility>

namespace simpleslam::modules {

class SensorIoModule final : public comm::Module {
public:
    /// 一个包大致要读多少条来源数据（10 Hz 扫描 + 100 Hz IMU ≈ 11）。
    /// 只用来给 pump 的读取预算一个默认量级，不是契约。
    static constexpr std::size_t kSamplesPerPacket = 64;

    SensorIoModule(std::unique_ptr<fw::ISensorSource> source, fw::PacketizerPolicy policy,
                   SlamResources& resources)
        : source_(std::move(source)), packetizer_(policy), resources_(&resources) {}

    /// 一次 pump 的结果。返回包数不够用：调用方要能区分"读完了"与"被下游挡住了"。
    struct PumpResult {
        std::size_t published = 0;   ///< 主消费者确实接纳的包数
        std::size_t rejected = 0;    ///< 被下游拒收的包数（本次让出执行权的原因）
        std::size_t samples = 0;     ///< 从来源读取的条数
        bool finished = false;       ///< 来源已结束，且剩下的触发帧都已发出
        bool yielded = false;        ///< 因为拒收或预算耗尽而提前返回
        bool stopped = false;        ///< 运行时已停止输入，不再读取来源或尝试发布

        /// 兼容旧调用点："这次发了几个包"。
        [[nodiscard]] operator std::size_t() const noexcept { return published; }  // NOLINT
    };

    /// 发出至多 max_packets 个包（按尝试计，被拒的也算），读满 max_samples 条、或数据
    /// 结束时返回。
    ///
    /// **预算按尝试计**：发布被拒时立刻返回并置 yielded。只在成功时加计数的话，
    /// "下游满载 → 继续读来源 → 还是满载"会一直转下去，单工作线程配置下消费者
    /// 根本没有机会被调度。读取条数也有上限，来源是连续流（IMU 高频、始终不凑成包）
    /// 时同样要让出执行权。
    ///
    /// 每一步先发打包器里已经就绪的帧，再读来源；来源读完之后逐帧排空还在等括点的
    /// 触发帧，排空完才报 finished。模块已关闭时什么也不做：关闭之后它是只读的最终状态。
    PumpResult pump(std::size_t max_packets = 1, std::size_t max_samples = 0) {
        PumpResult result;
        if (core().inputsStopped() || stopped()) {
            result.stopped = true;
            return result;
        }
        const std::size_t sample_budget = max_samples > 0 ? max_samples : kSamplesPerPacket
                                                                              * (max_packets + 1);
        const auto attempts = [&result] { return result.published + result.rejected; };
        // 发一个已就绪的包。预算在生成之前检查：先生成再判断会留下没有结论的悬空包
        const auto emitReady = [&](bool at_eof) {
            if (attempts() >= max_packets) return false;
            auto packet = at_eof ? packetizer_.flush() : packetizer_.poll();
            if (!packet) return false;
            publishPacket(std::move(*packet), result);
            return true;
        };
        for (;;) {
            if (core().inputsStopped()) {
                result.stopped = true;
                return result;
            }
            if (source_exhausted_) {
                while (result.rejected == 0 && emitReady(true)) {
                }
                if (result.rejected > 0 || packetizer_.waitingForWatermark()) {
                    result.yielded = true;  // 拒收或预算用完：剩下的下一步接着排空
                    if (result.rejected > 0) tail_rejected_ = true;
                    return result;
                }
                finished_ = true;
                result.finished = true;
                return result;
            }
            while (result.rejected == 0 && emitReady(false)) {
            }
            if (result.rejected > 0 || attempts() >= max_packets) {
                result.yielded = result.rejected > 0;  // 拒收即让出：继续读只会把积压堆得更高
                return result;
            }
            if (result.samples >= sample_budget) {
                result.yielded = true;
                return result;
            }
            auto datum = source_->next();
            ++result.samples;
            if (!datum) {
                // 来源读完：不再调用 next()，剩下的触发帧按"已有多少 IMU 就声明多少"
                // 逐帧发出，缺支撑如实标缺口
                source_exhausted_ = true;
                outcome_ = source_->outcome();
                continue;
            }
            packetizer_.ingest(*datum);
        }
    }

    void publishPacket(fw::SensorPacket packet, PumpResult& result) {
        requireSingleReliableConsumer();
        const std::uint64_t sequence = packet.sequence;
        const auto publish_result = packet_pub_.publish(std::move(packet));
        // 判据是"那一个可靠订阅端（主消费者）收下了"，不是"没人满载"：
        // 话题在停止阶段被关掉时后者仍然为真，而下游一个包也没收到
        if (publish_result.deliveredTo(1)) {
            packetizer_.notifyAdmitted(sequence);
            ++published_;
            ++result.published;
        } else {
            // 下游没收下：接纳水位不动，这段 IMU 会并进下一包并标缺口
            packetizer_.notifyRejected(sequence);
            ++rejected_;
            ++result.rejected;
            if (publish_result.topic_closed) ++closed_publishes_;
        }
    }

    /// 首次发布时校验接线契约。装配期查不了：订阅是各模块 onInit 时才登记的。
    ///
    /// 标志在**校验通过之后**才置位：先置位再抛的话，异常被上层边界接住之后，
    /// 后续发布会直接跳过校验，接线错误就此蒙混过关。
    void requireSingleReliableConsumer() {
        if (checked_wiring_) return;
        const auto topic = core().topic<fw::SensorPacket>(resolve(topics::kSensorPacket));
        const std::size_t reliable = topic->reliableSubscriberCount();
        if (reliable != 1) {
            throw comm::WiringError(
                std::string("原始包的可靠订阅者必须恰好一个（主消费者），实得 ")
                + std::to_string(reliable)
                + "。观察者请改订 /slam/odometry 或用 kDropOldest / kLatest。");
        }
        checked_wiring_ = true;
    }

    [[nodiscard]] bool finished() const noexcept { return finished_; }
    /// 来源已经读完（next() 返回过空）。与 finished() 不同：读完之后还可能有触发帧在排空。
    [[nodiscard]] bool sourceExhausted() const noexcept { return source_exhausted_; }
    /// 来源的终态。未结束时是 kRunning；不完整的回放在这里可见。
    [[nodiscard]] fw::SourceOutcome outcome() const noexcept { return outcome_; }
    /// 来源读完时最后几帧是否有被下游拒收的。来源读完 ≠ 全部送达。
    [[nodiscard]] bool tailRejected() const noexcept { return tail_rejected_; }
    /// 因为话题已关而没发出去的包数。停止阶段的正常现象，但必须计数。
    [[nodiscard]] std::uint64_t closedPublishes() const noexcept { return closed_publishes_; }
    [[nodiscard]] std::uint64_t publishedPackets() const noexcept { return published_; }
    [[nodiscard]] std::uint64_t rejectedPackets() const noexcept { return rejected_; }
    [[nodiscard]] std::uint64_t consumedPackets() const noexcept { return consumed_; }
    [[nodiscard]] fw::Packetizer& packetizer() noexcept { return packetizer_; }

    /// 在线模式：按固定周期自驱动。离线模式不要调用它。
    void startPumping(comm::Duration period, std::size_t packets_per_tick = 1) {
        if (stopped()) return;
        pump_timer_ = createTimer(period, [this, packets_per_tick] {
            const auto result = pump(packets_per_tick);
            if (result.yielded) ++yields_;  // 让出次数是在线背压的直接读数
        });
    }

    /// 因为下游拒收或读取预算耗尽而提前返回的次数。
    [[nodiscard]] std::uint64_t yields() const noexcept { return yields_; }

protected:
    void onInit() override {
        // 入口话题：外部数据从这里进系统。停止时先关它，让已接纳的包把
        // "里程计 → 关键帧 → 图 → 轨迹"整条因果链走完。
        packet_pub_ = advertise<fw::SensorPacket>(topics::kSensorPacket,
                                                  comm::PublishOptions{false, true});
        // 消费进度是累计的，只有最新值有用：Latest 档不会因为确认太多而拒收，
        // 丢了中间几条也会被下一条补上
        subscribe(topics::kPacketConsumed, &SensorIoModule::onConsumed,
                  comm::SubscribeOptions{1, comm::Delivery::kLatest});
        (void)resources_;
    }

private:
    /// 下游确认用完：推进消费水位，原始 IMU 到这里才允许回收。
    void onConsumed(const fw::PacketConsumed& event) {
        packetizer_.notifyConsumedThrough(event.through);
        consumed_ = event.through;
    }

    std::unique_ptr<fw::ISensorSource> source_;
    fw::Packetizer packetizer_;
    SlamResources* resources_;
    comm::Publisher<fw::SensorPacket> packet_pub_;
    comm::TimerHandle pump_timer_;
    bool finished_ = false;
    bool source_exhausted_ = false;
    bool tail_rejected_ = false;
    bool checked_wiring_ = false;
    fw::SourceOutcome outcome_ = fw::SourceOutcome::kRunning;
    std::uint64_t published_ = 0;
    std::uint64_t rejected_ = 0;
    std::uint64_t consumed_ = 0;
    std::uint64_t closed_publishes_ = 0;
    std::uint64_t yields_ = 0;
};

}  // namespace simpleslam::modules
