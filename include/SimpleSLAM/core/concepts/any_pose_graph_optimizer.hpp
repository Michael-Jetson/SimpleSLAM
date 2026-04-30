#pragma once

/// @file any_pose_graph_optimizer.hpp
/// PoseGraphOptimizer 的 Sean Parent 式类型擦除包装
///
/// PGO 服务需要运行时选择具体实现（GTSAM iSAM2、mini-PGO 等）。
/// AnyPoseGraphOptimizer 通过 Concept/Model 桥接提供一次虚分派，
/// optimize() 每回环只调用一次，开销可忽略。
///
/// AnyPoseGraphOptimizer 自身满足 PoseGraphOptimizer concept。

#include <SimpleSLAM/core/concepts/pose_graph_optimizer.hpp>
#include <SimpleSLAM/core/infra/demangle.hpp>
#include <SimpleSLAM/core/infra/logger.hpp>

#include <cassert>
#include <memory>
#include <string>
#include <utility>

namespace simpleslam {

class AnyPoseGraphOptimizer final {
public:
    /// 包装任何满足 PoseGraphOptimizer concept 的类型
    /// 默认插件名 = 具体类型的类名（通过 demangle 反修饰）
    template <PoseGraphOptimizer T>
    explicit AnyPoseGraphOptimizer(T impl)
        : self_(std::make_unique<Model<T>>(std::move(impl)))
        , name_(detail::demangle<T>())
        , log_(Logger::get(name_)) {}

    AnyPoseGraphOptimizer(AnyPoseGraphOptimizer&&) noexcept = default;
    AnyPoseGraphOptimizer& operator=(AnyPoseGraphOptimizer&&) noexcept = default;

    AnyPoseGraphOptimizer(const AnyPoseGraphOptimizer&) = delete;
    AnyPoseGraphOptimizer& operator=(const AnyPoseGraphOptimizer&) = delete;

    // ── concept 要求的方法 ──

    void addOdometryEdge(uint64_t from_id, uint64_t to_id,
                         const SE3d& relative_pose, const Mat6d& information) {
        assert(self_);
        self_->doAddOdometryEdge(from_id, to_id, relative_pose, information);
    }

    void addLoopEdge(uint64_t from_id, uint64_t to_id,
                     const SE3d& relative_pose, const Mat6d& information) {
        assert(self_);
        self_->doAddLoopEdge(from_id, to_id, relative_pose, information);
    }

    [[nodiscard]] OptimizationResult optimize() {
        assert(self_);
        return self_->doOptimize();
    }

    void reset() {
        assert(self_);
        self_->doReset();
    }

    [[nodiscard]] bool valid() const { return self_ != nullptr; }

    // ── 命名与日志 ──

    void setName(std::string name) {
        name_ = std::move(name);
        refreshLogger();
    }

    void setOwner(const std::string& owner) {
        owner_ = owner;
        refreshLogger();
    }

    [[nodiscard]] const std::string& name() const { return name_; }
    [[nodiscard]] const std::string& owner() const { return owner_; }

private:
    struct Concept {
        virtual ~Concept() = default;
        virtual void doAddOdometryEdge(uint64_t, uint64_t, const SE3d&, const Mat6d&) = 0;
        virtual void doAddLoopEdge(uint64_t, uint64_t, const SE3d&, const Mat6d&) = 0;
        [[nodiscard]] virtual OptimizationResult doOptimize() = 0;
        virtual void doReset() = 0;
        virtual void injectLogger(std::shared_ptr<spdlog::logger>) {}
    };

    template <PoseGraphOptimizer T>
    struct Model final : Concept {
        T data;

        explicit Model(T impl) : data(std::move(impl)) {}

        void doAddOdometryEdge(uint64_t from, uint64_t to,
                               const SE3d& rel, const Mat6d& info) override {
            data.addOdometryEdge(from, to, rel, info);
        }

        void doAddLoopEdge(uint64_t from, uint64_t to,
                           const SE3d& rel, const Mat6d& info) override {
            data.addLoopEdge(from, to, rel, info);
        }

        [[nodiscard]] OptimizationResult doOptimize() override {
            return data.optimize();
        }

        void doReset() override { data.reset(); }

        void injectLogger(std::shared_ptr<spdlog::logger> log) override {
            if constexpr (requires(T& t) { t.setLogger(log); }) {
                data.setLogger(log);
            }
        }
    };

    void refreshLogger() {
        auto tag = owner_.empty() ? name_ : owner_ + ":" + name_;
        log_ = Logger::get(tag);
        if (self_) self_->injectLogger(log_);
    }

    std::unique_ptr<Concept> self_;
    std::string name_;
    std::string owner_;
    std::shared_ptr<spdlog::logger> log_;
};

static_assert(PoseGraphOptimizer<AnyPoseGraphOptimizer>,
              "AnyPoseGraphOptimizer must satisfy PoseGraphOptimizer concept");

}  // namespace simpleslam
