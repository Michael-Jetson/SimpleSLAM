#pragma once

/// @file any_loop_detector.hpp
/// LoopDetector 的 Sean Parent 式类型擦除包装
///
/// 回环检测服务需要运行时选择具体实现（ScanContext、STD 等）。
/// AnyLoopDetector 通过 Concept/Model 桥接提供一次虚分派，
/// detect() 每关键帧只调用一次，开销可忽略。
///
/// AnyLoopDetector 自身满足 LoopDetector concept。

#include <SimpleSLAM/core/concepts/loop_detector.hpp>
#include <SimpleSLAM/core/infra/demangle.hpp>
#include <SimpleSLAM/core/infra/logger.hpp>

#include <cassert>
#include <memory>
#include <string>
#include <utility>

namespace simpleslam {

class AnyLoopDetector final {
public:
    /// 包装任何满足 LoopDetector concept 的类型
    /// 默认插件名 = 具体类型的类名（通过 demangle 反修饰）
    template <LoopDetector T>
    explicit AnyLoopDetector(T impl)
        : self_(std::make_unique<Model<T>>(std::move(impl)))
        , name_(detail::demangle<T>())
        , log_(Logger::get(name_)) {}

    AnyLoopDetector(AnyLoopDetector&&) noexcept = default;
    AnyLoopDetector& operator=(AnyLoopDetector&&) noexcept = default;

    AnyLoopDetector(const AnyLoopDetector&) = delete;
    AnyLoopDetector& operator=(const AnyLoopDetector&) = delete;

    // ── concept 要求的方法 ──

    void addKeyframe(const KeyframeData& keyframe) {
        assert(self_);
        self_->doAddKeyframe(keyframe);
    }

    [[nodiscard]] std::optional<LoopCandidate> detect(const KeyframeData& keyframe) {
        assert(self_);
        return self_->doDetect(keyframe);
    }

    [[nodiscard]] bool valid() const { return self_ != nullptr; }

    // ── 命名与日志 ──

    /// 设置插件名（Registry::create 自动调用，也可手动设置）
    void setName(std::string name) {
        name_ = std::move(name);
        refreshLogger();
    }

    /// 设置归属模块名（持有此插件的 Service 在构造时调用）
    /// 日志前缀变为 "owner:name"，如 "LoopClosureService:scan_context"
    void setOwner(const std::string& owner) {
        owner_ = owner;
        refreshLogger();
    }

    [[nodiscard]] const std::string& name() const { return name_; }
    [[nodiscard]] const std::string& owner() const { return owner_; }

private:
    struct Concept {
        virtual ~Concept() = default;
        virtual void doAddKeyframe(const KeyframeData&) = 0;
        [[nodiscard]] virtual std::optional<LoopCandidate> doDetect(const KeyframeData&) = 0;
        /// 如果具体类型有 setLogger 方法，注入日志器
        virtual void injectLogger(std::shared_ptr<spdlog::logger>) {}
    };

    template <LoopDetector T>
    struct Model final : Concept {
        T data;

        explicit Model(T impl) : data(std::move(impl)) {}

        void doAddKeyframe(const KeyframeData& kf) override {
            data.addKeyframe(kf);
        }

        [[nodiscard]] std::optional<LoopCandidate> doDetect(const KeyframeData& kf) override {
            return data.detect(kf);
        }

        void injectLogger(std::shared_ptr<spdlog::logger> log) override {
            if constexpr (requires(T& t) { t.setLogger(log); }) {
                data.setLogger(log);
            }
        }
    };

    /// 根据 owner 和 name 创建/更新日志器
    void refreshLogger() {
        auto tag = owner_.empty() ? name_ : owner_ + ":" + name_;
        log_ = Logger::get(tag);
        if (self_) self_->injectLogger(log_);
    }

    std::unique_ptr<Concept> self_;
    std::string name_;                              ///< 插件名（默认=类名）
    std::string owner_;                             ///< 归属模块名（如 "LoopClosureService"）
    std::shared_ptr<spdlog::logger> log_;           ///< 专属日志器
};

static_assert(LoopDetector<AnyLoopDetector>,
              "AnyLoopDetector must satisfy LoopDetector concept");

}  // namespace simpleslam
