// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file graph_optimizer_module.hpp
/// 图优化模块：位姿图的唯一写者。
///
/// "唯一写者"由写令牌在类型上保证，不靠评审。
///
/// 三条约定：
///
/// 1. **约束的恢复要闭合**。端点尚未入图的约束不能就地丢弃：它带稳定身份
///    （ConstraintId），挂起等待，关键帧补齐后重试，重复提交按身份去重。
///    "图是否优化成功"与"约束是否已进入真源"因此是两个独立的结果。
///    挂起表有界，满了就拒并在确认里告知发送方。
/// 2. **校正事件携带解本身**。发布的是一份不可变的图快照，它自己带身份，
///    消费者不再"先校验身份、再去读最新图"——那两步之间可以插进新的解。
/// 3. **不可用的解不进真源，也不发校正**。"没收敛"与"数值失败"是两回事，
///    前者的解仍可用，后者会把发散的位姿带进图里。

#include <SimpleSLAM/comm/module.hpp>
#include <SimpleSLAM/plugins/graph_optimizer.hpp>
#include <SimpleSLAM/framework/diagnostics.hpp>
#include <SimpleSLAM/framework/solution.hpp>
#include <SimpleSLAM/modules/topics.hpp>

#include <map>
#include <memory>
#include <utility>

namespace simpleslam::modules {

/// 里程计边的兜底权重：算法没有给出跨关键帧的相对约束时用它，并如实标为启发式。
struct OdometryEdgePolicy {
    double translation_weight = 100.0;  ///< 1/m²
    double rotation_weight = 400.0;     ///< 1/rad²
    fw::PerturbationConvention convention{};
};

class GraphOptimizerModule final : public comm::Module {
public:
    GraphOptimizerModule(std::unique_ptr<plugin::GraphOptimizerBase> algorithm,
                         SlamResources& resources, fw::DiagnosticSink* diagnostics = nullptr,
                         OdometryEdgePolicy edge_policy = {})
        : algorithm_(std::move(algorithm)),
          resources_(&resources),
          graph_writer_(resources.graph.acquireWriteToken()),
          diagnostics_(diagnostics),
          edge_policy_(edge_policy) {}

    /// 挂起约束的上限。端点永远不会出现的约束不能无限堆着。
    static constexpr std::size_t kMaxDeferred = 1024;

    [[nodiscard]] plugin::GraphOptimizerBase& algorithm() noexcept { return *algorithm_; }
    [[nodiscard]] std::uint64_t optimizations() const noexcept { return solutions_; }
    [[nodiscard]] std::size_t deferredConstraints() const noexcept { return deferred_.size(); }
    [[nodiscard]] std::uint64_t committedConstraints() const noexcept { return committed_; }
    /// 因为挂起表满而被拒的约束数。非零说明有一批约束的端点始终没出现。
    [[nodiscard]] std::uint64_t rejectedByBacklog() const noexcept { return deferred_rejected_; }
    /// 求解失败（优化器异常或解不可用）的次数。
    [[nodiscard]] std::uint64_t failedSolutions() const noexcept { return failed_solutions_; }
    /// 因为图里混入了不同标定的节点而跳过求解的次数。
    [[nodiscard]] std::uint64_t skippedForMixedCalibration() const noexcept {
        return skipped_mixed_;
    }
    /// 没能送达发送方的提交确认数。非零意味着对面还在等一个不会来的回音。
    [[nodiscard]] std::uint64_t undeliveredAcks() const noexcept { return undelivered_acks_; }

    /// 后端有没有追平，按**输入版本**判断：图里每一条必需约束（回环、先验）是否都进了
    /// 最后一次成功的求解。返回没进去的条数。
    ///
    /// 只数节点不够：新增一条回环边并不增加节点数。反过来，最后一次求解之后只追加了
    /// 里程计节点也不算没追平——新节点的初值由上一节点的校正后位姿沿里程计延拓，本来
    /// 就是当前校正下的位置，不需要为它们重新优化。边只增不改，所以"进没进最后一次
    /// 求解"就是"下标是否在那次求解的边数之内"。
    [[nodiscard]] std::size_t unsolvedRequiredConstraints() const {
        const auto graph = resources_->graph.snapshot();
        std::size_t unsolved = 0;
        for (std::size_t i = last_solved_edges_; i < graph->edges.size(); ++i) {
            if (graph->edges.at(i).kind != fw::EdgeKind::kOdometry) ++unsolved;
        }
        return unsolved;
    }

protected:
    void onInit() override {
        correction_pub_ = advertise<fw::CorrectionEvent>(topics::kCorrection,
                                                         comm::PublishOptions{true});
        // 提交确认：让发送方知道约束进没进真源，也让它知道容量腾出来了
        ack_pub_ = advertise<fw::ConstraintAck>(topics::kConstraintAck);
        // 标定版本不在这里定：它来自第一个节点建立时实际用的那一份
        graph_writer_.modify([&](fw::PoseGraphData& graph) {
            graph.session = resources_->session;
            graph.identity.session = resources_->session;
        });
        subscribe(topics::kKeyframe, &GraphOptimizerModule::onKeyframe,
                  comm::SubscribeOptions{64, comm::Delivery::kReliable});
        subscribe(topics::kLoopConstraint, &GraphOptimizerModule::onLoop,
                  comm::SubscribeOptions{64, comm::Delivery::kReliable});
    }

    /// 完成已接纳工作：主动核对一次补读游标，不等下一条通知。末条关键帧通知丢了、
    /// 之后又没有新的输入时，靠通知驱动的补读不会发生；最后还有必需约束没进成功解的，
    /// 再求解一次。
    void onFinish() override {
        catchUpKeyframes();
        const bool retried = retryDeferred() > 0;
        if (retried || unsolvedRequiredConstraints() > 0) optimizeAndPublish();
    }

private:
    void onKeyframe(const fw::KeyframeEvent& event) {
        (void)event;  // 通知只提示位置，真正的数据按序号补读
        if (!catchUpKeyframes()) return;
        // 新节点到齐了，此前挂起的约束可能已经有了端点
        if (retryDeferred() > 0) optimizeAndPublish();
    }

    /// 按游标从关键帧库补读，追加节点与里程计边。返回是否有新节点。
    bool catchUpKeyframes() {
        const auto store = resources_->keyframes.snapshot();
        const auto pending = store->since(consumed_sequence_);
        if (pending.empty()) return false;

        // 私有游标与真源提交绑定：草稿失败不提交，但模块自己的 consumed_sequence_ /
        // last_frame_ 是在 mutate 里推进的，异常一抛就越过了真源的实际位置，
        // 之后那几帧永远补不回来。所以先在局部推进，提交成功了才落到成员上。
        fw::KeyframePtr last_frame = last_frame_;
        std::uint64_t consumed = consumed_sequence_;
        graph_writer_.modify([&](fw::PoseGraphData& graph) {
            fw::KeyframePtr cursor = last_frame_;
            std::uint64_t seen = consumed_sequence_;
            for (const auto& frame : pending) {
                appendNode(graph, frame, cursor);
                cursor = frame;
                seen = frame->sequence;
            }
            last_frame = cursor;
            consumed = seen;
        });
        last_frame_ = std::move(last_frame);
        consumed_sequence_ = consumed;
        return true;
    }

    void appendNode(fw::PoseGraphData& graph, const fw::KeyframePtr& frame,
                    const fw::KeyframePtr& previous_frame) {
        fw::PoseGraphNode node;
        node.id = frame->id;
        node.time = frame->time;
        node.calibration_revision = frame->calibration_revision;

        if (graph.nodeCount() == 0) {
            graph.calibration_revision = frame->calibration_revision;
            graph.identity.calibration_revision = frame->calibration_revision;
        } else if (frame->calibration_revision != graph.calibration_revision) {
            graph.calibration_mixed = true;  // 两份标定的混合：不再产出校正
        }
        // 关键帧没有前驱：首帧，或里程计重新初始化之后新的一段。新的一段的里程计系与
        // 旧链无关，拿两者的位姿之差当里程计边是在两个不相干的坐标系之间连线——
        // 所以不连边，另起一段并固定它自己的 gauge
        if (!previous_frame || !frame->previous.valid()) {
            node.T_world_body = frame->T_odom_body;
            node.fixed = true;  // 这一段的第一帧固定 gauge
            graph.addNode(node);
            graph.covered_until = frame->sequence;
            return;
        }

        const SE3d relative = previous_frame->T_odom_body.inverse().compose(frame->T_odom_body);
        const auto* previous = graph.node(previous_frame->id);
        node.T_world_body = previous != nullptr ? previous->T_world_body.compose(relative)
                                                : relative;
        graph.addNode(node);

        fw::PoseGraphEdge edge;
        edge.kind = fw::EdgeKind::kOdometry;
        edge.from = previous_frame->id;
        edge.to = frame->id;
        if (frame->constraint_from_previous && usableConstraint(*frame, *previous_frame)) {
            // 算法给出的跨关键帧约束：原样使用，来源与扰动约定一并带走
            const auto& constraint = *frame->constraint_from_previous;
            edge.measurement = constraint.measurement;
            edge.information = constraint.information;
            edge.information_source = constraint.source;
            edge.convention = constraint.convention;
        } else {
            // 算法没给：用里程计位姿之差作测量，权重按策略拟定并如实标为启发式。
            // 这里绝不拿"本帧配准信息矩阵"充数——那是另一个随机量。
            // 权重从本模块当前的参数快照读：运行期改了参数，下一条边就用新值。
            edge.measurement = relative;
            edge.information = fw::diagonalInformation(
                paramOr("/config/backend/odometry_translation_weight",
                        edge_policy_.translation_weight),
                paramOr("/config/backend/odometry_rotation_weight", edge_policy_.rotation_weight),
                edge_policy_.convention);
            edge.information_source = fw::InformationSource::kHeuristic;
            edge.convention = edge_policy_.convention;
        }
        graph.edges.push_back(edge);
        graph.covered_until = frame->sequence;
    }

    /// 算法给的跨关键帧约束能不能直接当边用。
    ///
    /// 两端都要核对：`previous` 指的确实是这条边的起点，起止时刻也要对得上
    /// 实际的两个关键帧。只核 id 的话，一条时刻错位的约束会被当成这两个节点之间的
    /// 测量接进图里。信息矩阵含非有限值、来源未声明，同样不能用——
    /// 这时退回启发式权重，如实标明，而不是把一个来历不明的矩阵塞进图。
    [[nodiscard]] bool usableConstraint(const fw::Keyframe& frame,
                                        const fw::Keyframe& previous) const {
        if (!(frame.previous == previous.id)) return false;
        const auto& constraint = *frame.constraint_from_previous;
        if (constraint.from_time != previous.time || constraint.to_time != frame.time) return false;
        if (constraint.source == fw::InformationSource::kUnset) return false;
        if (!fw::isValidInformation(constraint.information)) return false;
        return fw::isFinite(constraint.measurement);
    }

    void onLoop(const fw::LoopConstraint& constraint) {
        const auto outcome = commitConstraint(constraint);
        switch (outcome) {
            case fw::ConstraintCommit::kCommitted:
                ++committed_;
                submit("graph.constraint_committed");
                break;
            case fw::ConstraintCommit::kDuplicate:
                submit("graph.constraint_duplicate");
                break;
            case fw::ConstraintCommit::kDeferred:
                submit("graph.constraint_deferred");
                break;
            case fw::ConstraintCommit::kRejected:
                submit("graph.constraint_rejected");
                break;
        }
        // 先回确认再优化：确认表达的是"进没进真源"，与"优化收没收敛"是两件事
        ack(constraint, outcome);
        if (outcome == fw::ConstraintCommit::kCommitted) optimizeAndPublish();
    }

    /// 回一条提交确认。送不到就计数：发送方要靠确认结案，确认丢了它会一直等。
    void ack(const fw::LoopConstraint& constraint, fw::ConstraintCommit outcome) {
        const auto result =
            ack_pub_.publish(fw::ConstraintAck{constraint.id, constraint.sequence, outcome});
        if (!result.deliveredTo(1)) ++undelivered_acks_;
    }

    /// 把一条约束送进真源。三种结果都不丢信息：已提交、重复、挂起等待。
    fw::ConstraintCommit commitConstraint(const fw::LoopConstraint& constraint) {
        if (!constraint.id.valid()) return fw::ConstraintCommit::kRejected;
        // 内容先过关再谈入图：非有限的测量、不合法的信息矩阵会让整张图一起发散；
        // 两端是同一个节点的"回环"不约束任何东西
        if (!fw::isFinite(constraint.T_match_query)
            || !fw::isValidInformation(constraint.information)
            || constraint.query == constraint.match) {
            return fw::ConstraintCommit::kRejected;
        }
        const auto graph = resources_->graph.snapshot();
        if (graph->hasConstraint(constraint.id)) return fw::ConstraintCommit::kDuplicate;
        if (graph->node(constraint.query) == nullptr
            || graph->node(constraint.match) == nullptr) {
            // 挂起表有界：端点永远不会出现的约束不能无限堆着。满了就拒最新的一条，
            // 让发送方从确认里看见，而不是悄悄吃掉。
            if (deferred_.size() >= kMaxDeferred && deferred_.count(constraint.id) == 0) {
                ++deferred_rejected_;
                return fw::ConstraintCommit::kRejected;
            }
            deferred_.insert_or_assign(constraint.id, constraint);  // 稳定身份即去重键
            return fw::ConstraintCommit::kDeferred;
        }
        deferred_.erase(constraint.id);
        graph_writer_.modify([&](fw::PoseGraphData& draft) {
            fw::PoseGraphEdge edge;
            edge.kind = fw::EdgeKind::kLoop;
            edge.id = constraint.id;
            edge.from = constraint.match;
            edge.to = constraint.query;
            edge.measurement = constraint.T_match_query;
            edge.information = constraint.information;
            edge.information_source = constraint.information_source;
            edge.convention = constraint.convention;
            draft.edges.push_back(edge);
            draft.committed_constraints.insert(constraint.id);
        });
        return fw::ConstraintCommit::kCommitted;
    }

    /// 重试全部挂起的约束。按 ConstraintId 升序，因此离线复现是确定的。
    std::size_t retryDeferred() {
        if (deferred_.empty()) return 0;
        std::size_t committed = 0;
        auto pending = deferred_;  // 迭代副本：commitConstraint 会改 deferred_
        for (const auto& [id, constraint] : pending) {
            const auto outcome = commitConstraint(constraint);
            if (outcome == fw::ConstraintCommit::kCommitted) {
                ++committed;
                ++committed_;
                submit("graph.constraint_committed_after_defer");
                ack(constraint, outcome);
            }
        }
        return committed;
    }

    /// 优化并发布一份自洽的解快照：身份写进图本身，事件携带图。
    ///
    /// 解不可用（数值失败或问题退化）时不发校正：没有解就没有"新的世界系"，
    /// 发一个带着旧位姿的校正事件只会让下游以为图动过。
    ///
    /// 解的标定身份写的是**这张图自己的**版本，不是当前最新：消费方拿它和自己当前的
    /// 标定比，版本不同就不采纳。图里混进了不同标定的节点时干脆不求解——那样的解
    /// 对应的不是任何一份标定。
    ///
    /// 优化器抛异常、解不可用、约束已提交是三件独立的事，各有各的诊断：
    /// 约束在提交确认时就已进入真源，与这里求解成败无关。
    void optimizeAndPublish() {
        if (resources_->graph.snapshot()->calibration_mixed) {
            ++skipped_mixed_;
            submit("graph.calibration_mixed");
            return;
        }
        const auto before = resources_->graph.version();
        plugin::GraphOptimizerBase::Report report;
        std::shared_ptr<const fw::PoseGraphData> solution;
        std::size_t solved_edges = 0;
        try {
            solution = graph_writer_.modifyAndSnapshot([&](fw::PoseGraphData& graph) {
                solved_edges = graph.edges.size();
                report = algorithm_->optimize(graph);
                if (!report.usable()) return;
                graph.identity.session = resources_->session;
                graph.identity.source_revision = before;
                graph.identity.solution_revision = ++solutions_;
                graph.identity.covered_until = graph.covered_until;
                graph.identity.calibration_revision = graph.calibration_revision;
            });
        } catch (const std::exception&) {
            // 草稿失败不提交：真源停在上一版，约束的提交结论不受影响
            ++failed_solutions_;
            submit("graph.optimizer_exception");
            return;
        }

        if (!report.usable()) {
            ++failed_solutions_;
            submit(statusKey(report.status));
            return;
        }
        last_solved_edges_ = solved_edges;  // 这次成功的解包含了这之前的全部边

        fw::CorrectionEvent event;
        event.time = last_frame_ ? last_frame_->time : fw::Timestamp{0};
        event.solution = std::move(solution);
        correction_pub_.publish(event);
        submit(statusKey(report.status));
    }

    static const char* statusKey(plugin::GraphOptimizerBase::Status status) noexcept {
        switch (status) {
            case plugin::GraphOptimizerBase::Status::kConverged: return "graph.optimized";
            case plugin::GraphOptimizerBase::Status::kMaxIterations: return "graph.max_iterations";
            case plugin::GraphOptimizerBase::Status::kNumericFailure: return "graph.numeric_failure";
            case plugin::GraphOptimizerBase::Status::kDegenerate: return "graph.degenerate";
        }
        return "graph.unknown_status";
    }

    /// 从本模块当前的参数快照取值，没有就用构造时给的默认值。
    [[nodiscard]] double paramOr(const std::string& name, double fallback) const {
        const auto& snapshot = paramSnapshot();
        if (!snapshot) return fallback;
        return snapshot->get<double>(name).value_or(fallback);
    }

    void submit(const char* key) {
        if (diagnostics_ == nullptr) return;
        fw::DiagnosticRecord record;
        record.header.module = name();
        record.header.session = resources_->session;
        record.header.frame_sequence = consumed_sequence_;
        record.level = fw::DiagnosticClass::kControl;
        record.payload = fw::EventRecord{key, ""};
        diagnostics_->submit(record);
    }

    std::unique_ptr<plugin::GraphOptimizerBase> algorithm_;
    SlamResources* resources_;
    fw::WriteToken<fw::PoseGraphData> graph_writer_;
    fw::DiagnosticSink* diagnostics_;
    OdometryEdgePolicy edge_policy_;
    comm::Publisher<fw::CorrectionEvent> correction_pub_;
    comm::Publisher<fw::ConstraintAck> ack_pub_;
    fw::KeyframePtr last_frame_;
    /// 端点尚未入图的约束。按 id 有序，重试顺序确定。
    std::map<fw::ConstraintId, fw::LoopConstraint> deferred_;
    std::uint64_t consumed_sequence_ = 0;
    /// 最后一次成功求解时图里的边数。
    std::size_t last_solved_edges_ = 0;
    std::uint64_t solutions_ = 0;
    std::uint64_t committed_ = 0;
    std::uint64_t deferred_rejected_ = 0;
    std::uint64_t failed_solutions_ = 0;
    std::uint64_t skipped_mixed_ = 0;
    std::uint64_t undelivered_acks_ = 0;
};

}  // namespace simpleslam::modules
