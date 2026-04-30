#pragma once

/// @file any_registration_target.hpp
/// RegistrationTarget 的 Sean Parent 式类型擦除包装
///
/// L1（YAML 驱动）模式下需要运行时选择具体 RegistrationTarget 实现。
/// AnyRegistrationTarget 通过 Concept/Model 桥接提供一次虚分派，
/// match() 每帧只调用一次（内部逐点迭代在具体实现内完成），开销可忽略。
///
/// AnyRegistrationTarget 自身满足 RegistrationTarget concept。

#include <SimpleSLAM/core/concepts/registration_target.hpp>
#include <SimpleSLAM/core/infra/demangle.hpp>
#include <SimpleSLAM/core/infra/logger.hpp>

#include <cassert>
#include <memory>
#include <string>
#include <utility>

namespace simpleslam {

class AnyRegistrationTarget final {
public:
    /// 包装任何满足 RegistrationTarget concept 的类型
    /// 包装任何满足 RegistrationTarget concept 的类型
    /// 默认插件名 = 具体类型的类名（通过 demangle 反修饰）
    template <RegistrationTarget T>
    explicit AnyRegistrationTarget(T impl)
        : self_(std::make_unique<Model<T>>(std::move(impl)))
        , name_(detail::demangle<T>())
        , log_(Logger::get(name_)) {}

    AnyRegistrationTarget(AnyRegistrationTarget&&) noexcept = default;
    AnyRegistrationTarget& operator=(AnyRegistrationTarget&&) noexcept = default;

    AnyRegistrationTarget(const AnyRegistrationTarget&) = delete;
    AnyRegistrationTarget& operator=(const AnyRegistrationTarget&) = delete;

    // ── concept 要求的方法 ──

    void match(const LidarScan& scan, const SE3d& pose, MatchResult& result) {
        assert(self_);
        self_->doMatch(scan, pose, result);
    }

    void update(const LidarScan& scan, const SE3d& pose) {
        assert(self_);
        self_->doUpdate(scan, pose);
    }

    [[nodiscard]] bool empty() const {
        assert(self_);
        return self_->doEmpty();
    }

    [[nodiscard]] size_t size() const {
        assert(self_);
        return self_->doSize();
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
        virtual void doMatch(const LidarScan&, const SE3d&, MatchResult&) = 0;
        virtual void doUpdate(const LidarScan&, const SE3d&) = 0;
        [[nodiscard]] virtual bool doEmpty() const = 0;
        [[nodiscard]] virtual size_t doSize() const = 0;
        virtual void injectLogger(std::shared_ptr<spdlog::logger>) {}
    };

    template <RegistrationTarget T>
    struct Model final : Concept {
        T data;

        explicit Model(T impl) : data(std::move(impl)) {}

        void doMatch(const LidarScan& scan, const SE3d& pose,
                     MatchResult& result) override {
            data.match(scan, pose, result);
        }

        void doUpdate(const LidarScan& scan, const SE3d& pose) override {
            data.update(scan, pose);
        }

        [[nodiscard]] bool doEmpty() const override { return data.empty(); }
        [[nodiscard]] size_t doSize() const override { return data.size(); }

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

static_assert(RegistrationTarget<AnyRegistrationTarget>,
              "AnyRegistrationTarget must satisfy RegistrationTarget concept");

}  // namespace simpleslam
