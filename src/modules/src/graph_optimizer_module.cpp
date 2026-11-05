// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/modules/graph_optimizer_module.hpp>

#include <exception>
#include <memory>
#include <string>
#include <utility>

namespace simpleslam::modules {

GraphOptimizerModule::GraphOptimizerModule(std::unique_ptr<plugin::GraphOptimizerBase> algorithm,
                                           SlamResources& resources,
                                           fw::DiagnosticSink* diagnostics,
                                           OdometryEdgePolicy edge_policy)
    : algorithm_(std::move(algorithm)),
      resources_(&resources),
      graph_writer_(resources.graph.acquireWriteToken()),
      diagnostics_(diagnostics),
      edge_policy_(edge_policy) {}

std::size_t GraphOptimizerModule::unsolvedRequiredConstraints() const {
    const auto graph = resources_->graph.snapshot();
    std::size_t unsolved = 0;
    for (std::size_t i = last_solved_edges_; i < graph->edges.size(); ++i) {
        if (graph->edges.at(i).kind != fw::EdgeKind::kOdometry) ++unsolved;
    }
    return unsolved;
}

void GraphOptimizerModule::onInit() {
    correction_pub_ = advertise(topics::kCorrection, comm::PublishOptions{.latch = true});
    // 标定版本不在这里定：它来自第一个节点建立时实际用的那一份
    graph_writer_.modify([&](fw::PoseGraphData& graph) {
        graph.session = resources_->session;
        graph.identity.session = resources_->session;
    });
    subscribe(topics::kKeyframe, 0, &GraphOptimizerModule::onKeyframe);
    subscribe(topics::kLoopConstraint, 0, &GraphOptimizerModule::onLoop);
}

void GraphOptimizerModule::onFinish() {
    catchUpKeyframes();
    if (unsolvedRequiredConstraints() > 0) optimizeAndPublish();
}

void GraphOptimizerModule::onKeyframe(const fw::KeyframeEvent& event) {
    (void)event;  // 通知只提示位置，真正的数据按序号补读
    catchUpKeyframes();
}

bool GraphOptimizerModule::catchUpKeyframes() {
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

void GraphOptimizerModule::appendNode(fw::PoseGraphData& graph, const fw::KeyframePtr& frame,
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
    node.T_world_body =
        previous != nullptr ? previous->T_world_body.compose(relative) : relative;
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
            paramOr(resolve("~odometry_translation_weight"), edge_policy_.translation_weight),
            paramOr(resolve("~odometry_rotation_weight"), edge_policy_.rotation_weight),
            edge_policy_.convention);
        edge.information_source = fw::InformationSource::kHeuristic;
        edge.convention = edge_policy_.convention;
    }
    graph.edges.push_back(edge);
    graph.covered_until = frame->sequence;
}

bool GraphOptimizerModule::usableConstraint(const fw::Keyframe& frame,
                                            const fw::Keyframe& previous) const {
    if (!(frame.previous == previous.id)) return false;
    const auto& constraint = *frame.constraint_from_previous;
    if (constraint.from_time != previous.time || constraint.to_time != frame.time) return false;
    if (constraint.source == fw::InformationSource::kUnset) return false;
    if (!fw::isValidInformation(constraint.information)) return false;
    return fw::isFinite(constraint.measurement);
}

void GraphOptimizerModule::onLoop(const fw::LoopConstraint& constraint) {
    // 先追平：关键帧先入库后通知，约束的端点这时一定读得到
    catchUpKeyframes();
    if (!commitConstraint(constraint)) {
        ++rejected_;
        submit("graph.constraint_rejected");
        return;
    }
    ++committed_;
    submit("graph.constraint_committed");
    optimizeAndPublish();
}

bool GraphOptimizerModule::commitConstraint(const fw::LoopConstraint& constraint) {
    if (!constraint.id.valid() || !fw::isFinite(constraint.T_match_query)
        || !fw::isValidInformation(constraint.information)
        || constraint.query == constraint.match) {
        return false;
    }
    const auto graph = resources_->graph.snapshot();
    if (graph->node(constraint.query) == nullptr || graph->node(constraint.match) == nullptr) {
        return false;
    }
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
    });
    return true;
}

void GraphOptimizerModule::optimizeAndPublish() {
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

const char* GraphOptimizerModule::statusKey(plugin::GraphOptimizerBase::Status status) noexcept {
    using Status = plugin::GraphOptimizerBase::Status;
    switch (status) {
        case Status::kConverged: return "graph.optimized";
        case Status::kMaxIterations: return "graph.max_iterations";
        case Status::kNumericFailure: return "graph.numeric_failure";
        case Status::kDegenerate: return "graph.degenerate";
    }
    return "graph.unknown_status";
}

double GraphOptimizerModule::paramOr(const std::string& name, double fallback) const {
    const auto& snapshot = paramSnapshot();
    if (!snapshot) return fallback;
    return snapshot->get<double>(name).value_or(fallback);
}

void GraphOptimizerModule::submit(const char* key) {
    if (diagnostics_ == nullptr) return;
    fw::DiagnosticRecord record;
    record.header.module = name();
    record.header.session = resources_->session;
    record.header.frame_sequence = consumed_sequence_;
    record.level = fw::DiagnosticClass::kControl;
    record.payload = fw::EventRecord{key, ""};
    diagnostics_->submit(record);
}

}  // namespace simpleslam::modules
