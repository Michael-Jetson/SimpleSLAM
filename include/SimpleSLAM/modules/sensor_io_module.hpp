// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file sensor_io_module.hpp
/// 传感器接入模块：来源 → 规范化 → 打包 → 发布。
///
/// 离线由系统逐包调 pump() 步进，在线由定时器驱动同一个 pump()，模块代码只有一套。
/// EOF 是显式状态，不靠超时判断。下游的处理进度告诉打包器还有多少包在途，处理不过来时由打包器丢触发帧。
///
/// 在线回放数据集时按数据时间的节奏放行（Pacing::kDataTime）：第一条数据对到开始回放的时刻，
/// 之后数据时刻为 t 的样本，到 开始 + (t − t0) 才交给打包器。还没到时候的那一条留在模块里，
/// 定时器下一次再看，模块线程不睡眠，停止输入随时生效。
///
/// ## 两条纪律
///
/// 1. **pump 有预算**。一条 IMU 可能同时让好几帧凑齐括点，EOF 时也可能还有好几帧在等——预算用完
///    就停在"生成下一包之前"，已就绪的帧留在打包器里，下一步先发它们再读来源。来源是连续流
///    （IMU 高频、始终不凑成包）时，读取条数的上限同样让出执行权。
/// 2. **来源的终态要传上来**。数据文件被截断时，`next()` 返回空与正常读完长得一模一样；
///    不完整必须可见，否则一段残缺轨迹的评测结果会被当成全程的。

#include <SimpleSLAM/comm/module.hpp>
#include <SimpleSLAM/framework/sensor_source.hpp>
#include <SimpleSLAM/modules/topics.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

namespace simpleslam::modules {

class SensorIoModule final : public comm::Module {
public:
    /// 在线时按什么节奏读来源。
    enum class Pacing : std::uint8_t {
        kAsFastAsPossible,  ///< 能读多快读多快：来源自己就按真实时间产出数据，或要测吞吐
        kDataTime,          ///< 按数据时间的节奏读：回放数据集
    };

    /// 一个包大致要读多少条来源数据（10 Hz 扫描 + 100 Hz IMU ≈ 11）。
    /// 只用来给 pump 的读取预算一个默认量级，不是契约。
    static constexpr std::size_t kSamplesPerPacket = 64;

    SensorIoModule(std::unique_ptr<fw::ISensorSource> source, fw::PacketizerPolicy policy,
                   SlamResources& resources)
        : source_(std::move(source)), packetizer_(policy), resources_(&resources) {}

    /// 一次 pump 的结果。返回包数不够用：调用方要能区分"读完了"与"预算用完了"。
    struct PumpResult {
        std::size_t published = 0;   ///< 发出的包数
        std::size_t samples = 0;     ///< 从来源读取的条数
        bool finished = false;       ///< 来源已结束，且剩下的触发帧都已发出
        bool yielded = false;        ///< 读取预算耗尽或还有帧要排空，下一步接着来
        bool stopped = false;        ///< 运行时已停止输入，不再读取来源

        [[nodiscard]] operator std::size_t() const noexcept { return published; }  // NOLINT
    };

    /// 发出至多 max_packets 个包，读满 max_samples 条、或数据结束时返回。
    ///
    /// 每一步先发打包器里已经就绪的帧，再读来源；来源读完之后逐帧排空还在等括点的触发帧，
    /// 排空完才报 finished。模块已收尾时什么也不做。
    PumpResult pump(std::size_t max_packets = 1, std::size_t max_samples = 0);

    [[nodiscard]] bool finished() const noexcept { return finished_; }
    /// 来源已经读完（next() 返回过空）。与 finished() 不同：读完之后还可能有触发帧在排空。
    [[nodiscard]] bool sourceExhausted() const noexcept { return source_exhausted_; }
    /// 来源的终态。未结束时是 kRunning；不完整的回放在这里可见。
    [[nodiscard]] fw::SourceOutcome outcome() const noexcept { return outcome_; }
    [[nodiscard]] std::uint64_t publishedPackets() const noexcept { return published_; }
    [[nodiscard]] std::uint64_t consumedPackets() const noexcept { return consumed_; }
    [[nodiscard]] fw::Packetizer& packetizer() noexcept { return packetizer_; }

    /// 在线模式：按固定周期自驱动。离线模式不要调用它。
    ///
    /// `on_finished` 在来源读完、触发帧排空之后调用一次，在本模块的线程上。
    void startPumping(comm::Duration period, std::size_t packets_per_tick = 1,
                      Pacing pacing = Pacing::kAsFastAsPossible,
                      std::function<void()> on_finished = {});

protected:
    void onInit() override;

private:
    /// 下游处理到哪了：打包器据此判断过载。
    void onConsumed(const fw::PacketConsumed& event);
    /// 按数据时间节奏回放时，这条数据到时候了没有。第一次问时定下回放的原点。
    [[nodiscard]] bool due(const fw::SensorDatum& datum);
    void markFinished();

    /// 回放的原点：开始回放的时刻与第一条数据的数据时刻。
    struct ReplayOrigin {
        comm::Time wall;
        fw::Timestamp data;
    };

    std::unique_ptr<fw::ISensorSource> source_;
    fw::Packetizer packetizer_;
    SlamResources* resources_;
    comm::Publisher<fw::SensorPacket> packet_pub_;
    comm::TimerHandle pump_timer_;
    Pacing pacing_ = Pacing::kAsFastAsPossible;
    std::function<void()> on_finished_;
    /// 读出来、还没到时候交给打包器的那一条。
    std::optional<fw::SensorDatum> pending_;
    std::optional<ReplayOrigin> origin_;
    bool finished_ = false;
    bool source_exhausted_ = false;
    fw::SourceOutcome outcome_ = fw::SourceOutcome::kRunning;
    std::uint64_t published_ = 0;
    std::uint64_t consumed_ = 0;
};

}  // namespace simpleslam::modules
