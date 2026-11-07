// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file evaluator_contract.hpp
/// 评测函数（`eval::EvaluatorBase`）的契约测试模板。每个实现在自己的测试文件里一行实例化：
///
///     SIMPLESLAM_EVALUATOR_CONTRACT(MyEvaluator)
///
/// 评测函数在合成的输入（`SyntheticEvaluationInputs`）上评测。查的是评测函数都要守的约定，
/// 不查数字：能登记进目录、按类型名还原；只读输入，同样的输入两次结果一样；给的名字守规矩
/// （`eval::checkEvaluation`）；没有样本时没有标量，只有说明；要真值而输入没有就报错，不要真值
/// 的结果与真值无关；输入缺度量、事件或单独一条估计时，要么报错，要么结果与输入齐全时相同，
/// 不把缺了当成"一条都没有"。

#include <SimpleSLAM/config/plugin.hpp>
#include <SimpleSLAM/eval/evaluator.hpp>

#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>

#include <bit>
#include <cmath>
#include <cstdint>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace simpleslam::testing {

/// 合成的评测输入：每 0.1 秒沿 x 走 0.1 米的真值，估计在它旁边带一点确定的偏差；每帧一条
/// 度量（任何键都给），没有事件。每样东西都可以去掉，看评测函数缺了它时怎么办。
class SyntheticEvaluationInputs final : public eval::IEvaluationInputs {
public:
    struct Options {
        int poses = 50;           ///< 0 表示什么都比不了
        bool groundtruth = true;  ///< 没有时 `pair()` 报错
        bool metrics = true;      ///< 没有时按接口的默认实现报错
        bool events = true;
        bool estimates = true;    ///< 单独一条估计
    };

    SyntheticEvaluationInputs() = default;
    explicit SyntheticEvaluationInputs(Options options) : options_(options) {}

    [[nodiscard]] eval::ComparablePair pair(std::string_view /*estimate*/) const override {
        if (!options_.groundtruth) throw std::runtime_error("这份评测输入没有真值");
        return {.estimate = estimated(), .reference = truth(), .notes = {}};
    }
    [[nodiscard]] std::optional<fw::TimeRange> targetWindow() const override {
        if (options_.poses == 0) return std::nullopt;
        return fw::TimeRange{at(0), at(options_.poses - 1)};
    }
    [[nodiscard]] fw::Trajectory estimate(std::string_view name) const override {
        if (!options_.estimates) return IEvaluationInputs::estimate(name);
        return estimated();
    }
    [[nodiscard]] std::vector<eval::MetricSample> metrics(std::string_view key) const override {
        if (!options_.metrics) return IEvaluationInputs::metrics(key);
        std::vector<eval::MetricSample> out;
        for (int i = 0; i < options_.poses; ++i) {
            out.push_back({.frame = static_cast<std::uint64_t>(i),
                           .data_time = at(i),
                           .source = "odometry.plugin",
                           .value = 1.0 + 0.1 * static_cast<double>(i % 7),
                           .unit = "ms"});
        }
        return out;
    }
    [[nodiscard]] std::vector<eval::EventSample> events(std::string_view key) const override {
        if (!options_.events) return IEvaluationInputs::events(key);
        return {};
    }

private:
    static fw::Timestamp at(int i) {
        return fw::Timestamp{static_cast<std::int64_t>(i) * 100'000'000};
    }
    [[nodiscard]] fw::Trajectory truth() const {
        fw::Trajectory trajectory;
        for (int i = 0; i < options_.poses; ++i) {
            trajectory.push(at(i), SE3d(Vec3d(0.1 * i, 0.0, 0.0), Eigen::Quaterniond::Identity()));
        }
        return trajectory;
    }
    [[nodiscard]] fw::Trajectory estimated() const {
        fw::Trajectory trajectory;
        for (int i = 0; i < options_.poses; ++i) {
            const double d = 0.01 * std::sin(0.7 * i);
            const Eigen::Quaterniond q(Eigen::AngleAxisd(0.2 * d, Vec3d::UnitZ()));
            trajectory.push(at(i), SE3d(Vec3d(0.1 * i + d, d, 0.5 * d), q));
        }
        return trajectory;
    }

    Options options_{};
};

/// 两份结果逐位相同：样本数、说明、标量（名字、值、单位、短名、权重）与表格。
inline bool sameEvaluation(const eval::Evaluation& a, const eval::Evaluation& b) {
    const auto same = [](double x, double y) {
        return std::bit_cast<std::uint64_t>(x) == std::bit_cast<std::uint64_t>(y);
    };
    if (a.samples != b.samples || a.notes != b.notes) return false;
    if (a.scalars.size() != b.scalars.size() || a.tables.size() != b.tables.size()) return false;
    for (std::size_t i = 0; i < a.scalars.size(); ++i) {
        const auto& x = a.scalars[i];
        const auto& y = b.scalars[i];
        if (x.name != y.name || x.unit != y.unit || x.label != y.label) return false;
        if (!same(x.value, y.value) || x.weight.has_value() != y.weight.has_value()) return false;
        if (x.weight && !same(*x.weight, *y.weight)) return false;
    }
    for (std::size_t i = 0; i < a.tables.size(); ++i) {
        const auto& x = a.tables[i];
        const auto& y = b.tables[i];
        if (x.name != y.name || x.columns.size() != y.columns.size()) return false;
        for (std::size_t c = 0; c < x.columns.size(); ++c) {
            if (x.columns[c].name != y.columns[c].name) return false;
            if (x.columns[c].unit != y.columns[c].unit) return false;
        }
        if (x.rows.size() != y.rows.size()) return false;
        for (std::size_t r = 0; r < x.rows.size(); ++r) {
            if (x.rows[r].size() != y.rows[r].size()) return false;
            for (std::size_t c = 0; c < x.rows[r].size(); ++c) {
                const auto& u = x.rows[r][c];
                const auto& v = y.rows[r][c];
                if (u.index() != v.index()) return false;
                if (const auto* p = std::get_if<double>(&u); p && !same(*p, std::get<double>(v))) {
                    return false;
                }
                if (const auto* p = std::get_if<std::int64_t>(&u);
                    p && *p != std::get<std::int64_t>(v)) {
                    return false;
                }
            }
        }
    }
    return true;
}

namespace detail {

/// 报错就是没有结果。
inline std::optional<eval::Evaluation> tryEvaluate(const eval::EvaluatorBase& evaluator,
                                                   const eval::IEvaluationInputs& inputs) {
    try {
        return evaluator.evaluate(inputs);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

}  // namespace detail

/// 评测函数契约。`config` 是被测实现的配置。
template <class E>
    requires config::PluginImplementation<E, eval::EvaluatorBase>
void checkEvaluatorContract(const typename E::Config& config = {}) {
    using Options = SyntheticEvaluationInputs::Options;
    const E evaluator(config);
    const eval::EvaluatorBase& e = evaluator;  // 经接口调用，和执行器里的用法一样
    const std::string name(E::kTypeName);
    const SyntheticEvaluationInputs full;

    SECTION("能登记进目录，按类型名还原、构造") {
        config::Catalogs catalogs;
        catalogs.of<eval::EvaluatorBase>().template add<E>();
        // 用这份配置写成的插件节点还原，和 YAML 里写了它一样
        const auto spec = config::Slot<eval::EvaluatorBase>::of<E>(config).resolved();
        const auto slot =
            catalogs.resolve<eval::EvaluatorBase>(spec, "evaluation.evaluators." + name);
        CHECK(slot.type() == E::kTypeName);
        CHECK(slot.make() != nullptr);
    }

    SECTION("只读输入：同样的输入两次评测，结果一样") {
        CHECK(sameEvaluation(e.evaluate(full), e.evaluate(full)));
        const E other(config);
        CHECK(sameEvaluation(e.evaluate(full), other.evaluate(full)));
    }

    SECTION("给的名字守规矩") {
        CHECK_NOTHROW(eval::checkEvaluation(name, e.evaluate(full)));
    }

    SECTION("没有样本时没有标量，只有说明") {
        for (const auto& result : {detail::tryEvaluate(e, SyntheticEvaluationInputs({.poses = 0})),
                                   detail::tryEvaluate(e, full)}) {
            if (!result || result->samples > 0) continue;
            CHECK(result->scalars.empty());
            CHECK_FALSE(result->notes.empty());
        }
    }

    SECTION("要真值而输入没有就报错；不要真值的，结果与真值无关") {
        const SyntheticEvaluationInputs without_truth({.groundtruth = false});
        if (e.needs().groundtruth) {
            CHECK_THROWS(e.evaluate(without_truth));
        } else {
            CHECK(sameEvaluation(e.evaluate(without_truth), e.evaluate(full)));
        }
    }

    SECTION("缺度量、事件或单独一条估计：要么报错，要么结果与输入齐全时相同") {
        const eval::Evaluation expected = e.evaluate(full);
        for (const Options options : {Options{.metrics = false}, Options{.events = false},
                                      Options{.estimates = false}}) {
            const auto result = detail::tryEvaluate(e, SyntheticEvaluationInputs(options));
            if (result) CHECK(sameEvaluation(*result, expected));
        }
    }
}

}  // namespace simpleslam::testing

/// 给一个评测函数生成契约测试：`SIMPLESLAM_EVALUATOR_CONTRACT(MyEvaluator)`，默认配置。
#define SIMPLESLAM_EVALUATOR_CONTRACT(Type)                                        \
    TEST_CASE("评测函数契约：" #Type, "[contract][eval]") {                        \
        ::simpleslam::testing::checkEvaluatorContract<Type>();                     \
    }
