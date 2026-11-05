// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file place_recognizer_module.hpp
/// 位置识别模块：回环检测与重定位合一，共用一套描述子库。
///
/// 关键帧通知只用来提示"消费到哪里了"：模块按序号从关键帧库补读，
/// 因此通知丢一条也不会让位姿图断链。
///
/// 回环约束是一次性信息，补读补不回来，所以走另一条路：每条约束带稳定身份
/// （ConstraintId），未被确认之前一直留在待办表里。消费侧按身份去重，
/// 因此重发不会在图里变成两条边（graph_optimizer_module.hpp）。
///
/// 待办表装的是**所有尚未确认的约束**，不只是发布被拒的那些。只记被拒的话，
/// 一条"发出去了但确认丢了"的约束就此消失：发送方以为送到了，消费方没收到，
/// 两边都不会再提它。
///
/// 重试由**消费端的提交确认**触发，不由新的业务数据触发：数据跑完之后没有下一帧，
/// 卡住的那条就永远发不出去。确认同时说明两件事——约束进没进真源，
/// 以及消费端的容量腾出来了。确认按身份去重，重复确认不重复计数。
///
/// 四种确认的处置各不相同：已提交与重复都是终态（从待办表移除）；挂起表示消费端
/// 收下了、正等端点补齐，发送方等得更久再查；拒绝是终态失败，计数并移除。
///
/// **确认本身也会丢，发送也会被拒**。待办表里的每一条都处在三个非终态之一：
/// 未送达（发布被拒）、等回执、消费端挂起。一个周期定时器覆盖**全部**非终态，
/// 不依赖新的关键帧或确认来推动：
/// - 未送达的，到点就重发；
/// - 等回执超时的，先查真源——位姿图是可读资源，约束进没进图一查便知，进了就地结案；
///   没进就重发（消费端按身份去重，重发安全）；
/// - 消费端挂起的等得更久（它在等端点补齐），超时后同样先查真源再重发。
///
/// 结案有两道上限：**重发次数**（只数真正送上总线的那几次，被拒的不算——背压不是
/// 消费端的否定）与**最长存活期**（从第一次尝试算起，不论处在哪个非终态）。后者保证
/// 每一条都在有限时间内结案：持续背压下一次也送不出去的约束，到期同样显式放弃并计数。
/// 于是"确认丢了""发送被拒之后再没有业务帧"都不会变成"永远挂着"。

#include <SimpleSLAM/comm/module.hpp>
#include <SimpleSLAM/plugins/place_recognizer.hpp>
#include <SimpleSLAM/framework/diagnostics.hpp>
#include <SimpleSLAM/modules/topics.hpp>

#include <map>
#include <memory>
#include <string>
#include <utility>

namespace simpleslam::modules {

class PlaceRecognizerModule final : public comm::Module {
public:
    PlaceRecognizerModule(std::unique_ptr<plugin::PlaceRecognizerBase> algorithm,
                          SlamResources& resources, fw::DiagnosticSink* diagnostics = nullptr)
        : algorithm_(std::move(algorithm)), resources_(&resources), diagnostics_(diagnostics) {}

    [[nodiscard]] plugin::PlaceRecognizerBase& algorithm() noexcept { return *algorithm_; }
    [[nodiscard]] std::uint64_t indexedFrames() const noexcept { return indexed_; }
    [[nodiscard]] std::uint64_t loopsFound() const noexcept { return loops_; }
    /// 尚未确认的约束数（含已送上总线正等确认的）。
    [[nodiscard]] std::size_t pendingLoops() const noexcept { return pending_.size(); }
    /// 已被消费端确认进入真源（或确认重复）的约束数。与"发出去了"不是一回事。
    [[nodiscard]] std::uint64_t confirmedLoops() const noexcept { return confirmed_; }
    [[nodiscard]] std::uint64_t acks() const noexcept { return acks_; }
    /// 已经结案的身份又收到确认的次数。重复确认是幂等的，这里只作诊断。
    [[nodiscard]] std::uint64_t duplicateAcks() const noexcept { return duplicate_acks_; }
    /// 被消费端终态拒绝的约束数。
    [[nodiscard]] std::uint64_t rejectedLoops() const noexcept { return rejected_; }
    /// 因待办表满而未受理的约束数。
    [[nodiscard]] std::uint64_t droppedLoops() const noexcept { return dropped_; }
    /// 回执丢失、但查真源发现已提交而就地结案的约束数。
    [[nodiscard]] std::uint64_t recoveredFromGraph() const noexcept { return recovered_from_graph_; }
    /// 重发次数用尽仍未进入真源、被显式放弃的约束数。
    [[nodiscard]] std::uint64_t abandonedLoops() const noexcept { return abandoned_; }

protected:
    /// 等确认的超时与重发上限。挂起在消费端的条目等得更久：它们在等端点补齐。
    static constexpr comm::Duration kAckTimeout = std::chrono::seconds(1);
    static constexpr std::uint32_t kMaxAttempts = 5;
    /// 最长存活期：从第一次尝试起，不论处在哪个非终态，到期即显式放弃。
    static constexpr comm::Duration kMaxAge = kAckTimeout * kMaxAttempts * 4;

    void onInit() override {
        recovery_timer_ = createTimer(kAckTimeout, [this] { recoverUnconfirmed(); });
        loop_pub_ = advertise<fw::LoopConstraint>(topics::kLoopConstraint);
        reloc_pub_ = advertise<fw::RelocationResult>(topics::kRelocationResult);
        subscribe(topics::kKeyframe, &PlaceRecognizerModule::onKeyframe,
                  comm::SubscribeOptions{64, comm::Delivery::kReliable});
        subscribe(topics::kRelocationRequest, &PlaceRecognizerModule::onRelocationRequest,
                  comm::SubscribeOptions{4, comm::Delivery::kReliable});
        // 确认到了就说明容量腾出来了：重试不必等下一帧业务数据
        subscribe(topics::kConstraintAck, &PlaceRecognizerModule::onAck,
                  comm::SubscribeOptions{64, comm::Delivery::kReliable});
    }

    /// 完成已接纳工作：主动核对一次补读游标并把能送的约束送出去，不等下一条通知。
    /// 送出去的约束在关闭时的排空里被消费、确认；仍没结案的如实留在待办表里。
    void onFinish() override {
        catchUpKeyframes();
        flushOutbox();
    }

private:
    /// 一条尚未结案的约束所处的非终态。
    enum class State : std::uint8_t {
        kUnsent,      ///< 还没送上总线（发布被拒）：到点就重发
        kAwaitingAck, ///< 已送上总线，等确认
        kHeld,        ///< 消费端已收下并挂起，正等端点补齐
    };

    struct Pending {
        fw::LoopConstraint constraint;
        State state = State::kUnsent;
        comm::Time first_attempt_at{};  ///< 第一次尝试的时刻：最长存活期从这里算
        comm::Time last_change_at{};    ///< 最近一次状态变化的时刻：超时从这里算
        std::uint32_t attempts = 0;     ///< 真正送上总线的次数（被拒的不算）
        std::uint64_t rejections = 0;   ///< 发布被拒的次数
    };

    void onKeyframe(const fw::KeyframeEvent& event) {
        (void)event;  // 通知只提示位置，真正的数据按序号补读
        flushOutbox();  // 先把还没送到的约束补发出去，再产生新的
        recoverUnconfirmed();
        catchUpKeyframes();
    }

    /// 按游标从关键帧库补读：建索引、找回环、发约束。
    void catchUpKeyframes() {
        const auto store = resources_->keyframes.snapshot();
        for (const auto& frame : store->since(consumed_sequence_)) {
            algorithm_->insert(*frame);
            ++indexed_;
            consumed_sequence_ = frame->sequence;

            for (auto& constraint : algorithm_->detectLoops(*frame)) {
                constraint.sequence = ++loop_sequence_;
                constraint.id = fw::ConstraintId{loop_sequence_};  // 稳定身份，重试可去重
                send(constraint);
            }
        }
    }

    /// 发一条约束，并把它留在待办表里直到收到确认。
    ///
    /// 判据是"那一个可靠订阅端确实接纳"，不是"没人满载"：话题在停止阶段被关掉时
    /// 后者仍然为真，而约束一条也没送出去。
    void send(const fw::LoopConstraint& constraint) {
        if (pending_.size() >= kMaxPending && pending_.count(constraint.id) == 0) {
            ++dropped_;  // 有界：超限时不再受理新的，并显式记下来
            report("place_recognizer.loop_dropped");
            return;
        }
        auto& entry = pending_[constraint.id];
        entry.constraint = constraint;
        entry.state = State::kUnsent;
        entry.first_attempt_at = clock().now();
        deliver(entry);
    }

    /// 送一次。成功进入"等回执"，被拒留在"未送达"，由定时器到点再试。
    void deliver(Pending& entry) {
        const auto result = loop_pub_.publish(entry.constraint);
        entry.last_change_at = clock().now();
        if (result.deliveredTo(1)) {
            if (entry.attempts == 0) ++loops_;
            ++entry.attempts;
            entry.state = State::kAwaitingAck;
            return;
        }
        ++entry.rejections;
        entry.state = State::kUnsent;
        report("place_recognizer.loop_deferred");
    }

    /// 推进全部非终态：查真源结案、到点重发、到期放弃。定时器每个周期跑一次，
    /// 新的关键帧与确认到来时也顺带跑，但结案不依赖它们。
    void recoverUnconfirmed() {
        if (pending_.empty()) return;
        const auto now = clock().now();
        const auto graph = resources_->graph.snapshot();
        for (auto it = pending_.begin(); it != pending_.end();) {
            auto& entry = it->second;
            const auto waited = now - entry.last_change_at;
            const bool due = entry.state == State::kUnsent
                             || (entry.state == State::kAwaitingAck && waited >= kAckTimeout)
                             || (entry.state == State::kHeld && waited >= kAckTimeout * 4);
            if (entry.state != State::kUnsent && due && graph->hasConstraint(it->first)) {
                // 回执丢了，但约束已经在真源里：就地结案
                ++confirmed_;
                ++recovered_from_graph_;
                it = pending_.erase(it);
                continue;
            }
            if (now - entry.first_attempt_at >= kMaxAge
                || (due && entry.state != State::kUnsent && entry.attempts >= kMaxAttempts)) {
                ++abandoned_;
                report("place_recognizer.loop_abandoned");
                it = pending_.erase(it);
                continue;
            }
            if (due) deliver(entry);
            ++it;
        }
    }

    /// 重发还没送达的约束。按 id 升序，离线可复现。第一次还送不出去就停：
    /// 消费端还满着，后面的留给定时器或下一次确认。
    void flushOutbox() {
        for (auto& [id, entry] : pending_) {
            (void)id;
            if (entry.state != State::kUnsent) continue;
            deliver(entry);
            if (entry.state == State::kUnsent) return;
        }
    }

    void onAck(const fw::ConstraintAck& ack) {
        ++acks_;
        const auto it = pending_.find(ack.id);
        if (it == pending_.end()) {
            ++duplicate_acks_;  // 已经结案的身份：重复确认是幂等的
            flushOutbox();
            return;
        }
        switch (ack.result) {
            case fw::ConstraintCommit::kCommitted:
            case fw::ConstraintCommit::kDuplicate:
                ++confirmed_;
                pending_.erase(it);
                break;
            case fw::ConstraintCommit::kDeferred:
                // 消费端收下了，正等端点补齐。它自己会重试，发送方等得更久再查
                it->second.state = State::kHeld;
                it->second.last_change_at = clock().now();
                break;
            case fw::ConstraintCommit::kRejected:
                ++rejected_;
                pending_.erase(it);
                break;
        }
        flushOutbox();
    }

    void onRelocationRequest(const fw::RelocationRequest& request) {
        const auto store = resources_->keyframes.snapshot();
        const auto frame = store->find(request.query);
        if (!frame || !frame->packet) {
            fw::RelocationResult result;
            result.generation = request.generation;
            result.session = resources_->session;
            result.success = false;
            reloc_pub_.publish(result);
            return;
        }
        auto result = algorithm_->relocalize(*frame->packet, request);
        result.session = resources_->session;
        reloc_pub_.publish(result);
        report(result.success ? "place_recognizer.reloc_ok" : "place_recognizer.reloc_failed");
    }

    /// 停止时如实记下未确认的约束数。实验产物因此能看出结论建立在不完整的图上。
    ///
    /// 分两项：一项是发送方手上还没送出去的，一项是消费端收下但还没进真源的。
    /// 两者都意味着最终的图少了几条边，但原因不同。
    void onShutdown() override {
        if (pending_.empty() || diagnostics_ == nullptr) return;
        std::size_t held = 0;
        for (const auto& [id, entry] : pending_) {
            if (entry.state == State::kHeld) ++held;
        }
        submitCount("place_recognizer.loops_unconfirmed", pending_.size());
        if (held > 0) submitCount("place_recognizer.loops_held_by_consumer", held);
    }

    void submitCount(const char* key, std::size_t count) {
        fw::DiagnosticRecord record;
        record.header.module = name();
        record.header.session = resources_->session;
        record.header.frame_sequence = consumed_sequence_;
        record.level = fw::DiagnosticClass::kControl;
        record.payload = fw::EventRecord{key, std::to_string(count)};
        diagnostics_->submit(record);
    }

    void report(const char* key) {
        if (diagnostics_ == nullptr) return;
        fw::DiagnosticRecord record;
        record.header.module = name();
        record.header.session = resources_->session;
        record.header.frame_sequence = consumed_sequence_;
        record.level = fw::DiagnosticClass::kControl;
        record.payload = fw::EventRecord{key, ""};
        diagnostics_->submit(record);
    }

    std::unique_ptr<plugin::PlaceRecognizerBase> algorithm_;
    SlamResources* resources_;
    fw::DiagnosticSink* diagnostics_;
    /// 待办表上限。按 id 有序，因此重发顺序是确定的，离线可复现。
    static constexpr std::size_t kMaxPending = 256;

    comm::Publisher<fw::LoopConstraint> loop_pub_;
    comm::Publisher<fw::RelocationResult> reloc_pub_;
    std::map<fw::ConstraintId, Pending> pending_;
    std::uint64_t consumed_sequence_ = 0;
    std::uint64_t loop_sequence_ = 0;
    std::uint64_t indexed_ = 0;
    std::uint64_t loops_ = 0;
    std::uint64_t acks_ = 0;
    std::uint64_t duplicate_acks_ = 0;
    std::uint64_t confirmed_ = 0;
    std::uint64_t rejected_ = 0;
    std::uint64_t dropped_ = 0;
    std::uint64_t recovered_from_graph_ = 0;
    std::uint64_t abandoned_ = 0;
    comm::TimerHandle recovery_timer_;
};

}  // namespace simpleslam::modules
