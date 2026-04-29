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

#include <cassert>
#include <memory>
#include <utility>

namespace simpleslam {

class AnyRegistrationTarget final {
public:
    /// 包装任何满足 RegistrationTarget concept 的类型
    template <RegistrationTarget T>
    explicit AnyRegistrationTarget(T impl)
        : self_(std::make_unique<Model<T>>(std::move(impl))) {}

    AnyRegistrationTarget(AnyRegistrationTarget&&) noexcept = default;
    AnyRegistrationTarget& operator=(AnyRegistrationTarget&&) noexcept = default;

    AnyRegistrationTarget(const AnyRegistrationTarget&) = delete;
    AnyRegistrationTarget& operator=(const AnyRegistrationTarget&) = delete;

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

private:
    struct Concept {
        virtual ~Concept() = default;
        virtual void doMatch(const LidarScan&, const SE3d&, MatchResult&) = 0;
        virtual void doUpdate(const LidarScan&, const SE3d&) = 0;
        [[nodiscard]] virtual bool doEmpty() const = 0;
        [[nodiscard]] virtual size_t doSize() const = 0;
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
    };

    std::unique_ptr<Concept> self_;
};

static_assert(RegistrationTarget<AnyRegistrationTarget>,
              "AnyRegistrationTarget must satisfy RegistrationTarget concept");

}  // namespace simpleslam
