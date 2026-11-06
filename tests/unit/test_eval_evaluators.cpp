// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 内置评测函数 ate、rpe：只读输入、只返回值；写法不对在构造时报；没测出东西不给数；
/// 单位随取段方式；实际用的对齐方式写进说明；逐样本误差进表格。

#include <SimpleSLAM/eval/evaluators.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

using namespace simpleslam;
using namespace simpleslam::eval;
using Catch::Approx;

namespace {

fw::Timestamp at(int index) {
    return fw::Timestamp{static_cast<std::int64_t>(index) * 100'000'000};
}

/// 每 0.1 秒沿 x 走 step_m 米，y 恒为 offset_y。
Trajectory line(int count, double step_m, double offset_y = 0.0) {
    Trajectory trajectory;
    for (int i = 0; i < count; ++i) {
        trajectory.push(at(i),
                        SE3d(Vec3d(i * step_m, offset_y, 0.0), Eigen::Quaterniond::Identity()));
    }
    return trajectory;
}

/// 评测函数看到的输入：两条现成的轨迹，记下评测函数要的是哪条估计。
class FixedInputs final : public EvaluationInputs {
public:
    FixedInputs(Trajectory estimate, Trajectory reference)
        : estimate_(std::move(estimate)), reference_(std::move(reference)) {}

    [[nodiscard]] ComparablePair pair(std::string_view estimate) const override {
        asked = std::string(estimate);
        return {.estimate = estimate_, .reference = reference_, .notes = {"真值换算过"}};
    }
    [[nodiscard]] std::optional<fw::TimeRange> targetWindow() const override {
        return std::nullopt;
    }

    mutable std::string asked;

private:
    Trajectory estimate_;
    Trajectory reference_;
};

const Scalar* find(const Evaluation& evaluation, std::string_view name) {
    for (const auto& scalar : evaluation.scalars) {
        if (scalar.name == name) return &scalar;
    }
    return nullptr;
}

bool noted(const Evaluation& evaluation, std::string_view text) {
    for (const auto& note : evaluation.notes) {
        if (note.find(text) != std::string::npos) return true;
    }
    return false;
}

}  // namespace

TEST_CASE("ate：统计量带单位，汇总表只收 rmse，逐帧误差进表格", "[eval][evaluators]") {
    AteEvaluator::Config config;
    config.alignment = "none";
    const AteEvaluator ate(config);
    CHECK(ate.needs().groundtruth);

    const FixedInputs inputs(line(10, 1.0, 0.3), line(10, 1.0));  // 恒定偏移 0.3 米
    const auto result = ate.evaluate(inputs);
    CHECK(inputs.asked == "odometry");  // 默认评 odometry 这条估计
    CHECK(result.samples == 10);
    REQUIRE(find(result, "translation.rmse") != nullptr);
    CHECK(find(result, "translation.rmse")->value == Approx(0.3));
    CHECK(find(result, "translation.rmse")->unit == "m");
    CHECK(find(result, "translation.rmse")->label == "ATE_t");
    CHECK(find(result, "rotation.rmse")->unit == "deg");
    CHECK(find(result, "rotation.rmse")->label == "ATE_r");
    CHECK(find(result, "translation.mean")->label.empty());
    CHECK(find(result, "max_reference_gap")->unit == "s");
    CHECK(find(result, "target_coverage") == nullptr);  // 没声明目标窗口就没有任务完成度
    CHECK(noted(result, "真值换算过"));                 // 输入的说明原样带上

    REQUIRE(result.tables.size() == 1);
    const auto& table = result.tables.front();
    CHECK(table.name == "errors");
    REQUIRE(table.columns.size() == 3);
    CHECK(table.columns[0].name == "time_ns");
    CHECK(table.columns[1].unit == "m");
    REQUIRE(table.rows.size() == 10);
    CHECK(std::get<std::int64_t>(table.rows[3][0]) == at(3).count());
    CHECK(std::get<double>(table.rows[3][1]) == Approx(0.3));
}

TEST_CASE("ate：一对都没关联上就不给数，只写明为什么", "[eval][evaluators]") {
    Trajectory far;
    for (int i = 0; i < 5; ++i) far.push(at(1000 + i), SE3d::Identity());
    const AteEvaluator ate(AteEvaluator::Config{});
    const auto result = ate.evaluate(FixedInputs(line(5, 1.0), far));
    CHECK(result.samples == 0);
    CHECK(result.scalars.empty());  // 零误差与"无结果"不能长得一样
    CHECK(result.tables.empty());
    CHECK(noted(result, "无结果"));
}

TEST_CASE("ate：估计退化做不了 sim3 时退回 se3，说明里写实际用的", "[eval][evaluators]") {
    Trajectory still;
    for (int i = 0; i < 10; ++i) still.push(at(i), SE3d::Identity());
    AteEvaluator::Config config;
    config.alignment = "sim3";
    const auto result = AteEvaluator(config).evaluate(FixedInputs(still, line(10, 1.0)));
    CHECK(result.samples == 10);
    CHECK(noted(result, "请求的对齐方式是 sim3，实际用的是 se3"));
    REQUIRE(find(result, "translation.rmse") != nullptr);
    CHECK(std::isfinite(find(result, "translation.rmse")->value));
    CHECK(find(result, "scale")->value == Approx(1.0));
}

TEST_CASE("rpe：按距离取段时误差按每米归一，按时间取段时不归一", "[eval][evaluators]") {
    const auto truth = line(31, 1.0);     // 3 秒走 30 米
    const auto estimate = line(31, 1.1);  // 每米多走 0.1 米

    RpeEvaluator::Config by_distance;
    by_distance.delta_m = 10.0;
    const auto distance = RpeEvaluator(by_distance).evaluate(FixedInputs(estimate, truth));
    CHECK(distance.samples > 0);
    REQUIRE(find(distance, "translation.rmse") != nullptr);
    CHECK(find(distance, "translation.rmse")->unit == "m/m");
    CHECK(find(distance, "rotation.rmse")->unit == "deg/m");
    CHECK(find(distance, "translation.rmse")->label == "RPE_t");
    CHECK(find(distance, "translation.rmse")->value == Approx(0.1).margin(1e-6));
    REQUIRE(distance.tables.size() == 1);
    CHECK(distance.tables.front().name == "segments");
    CHECK(distance.tables.front().columns[1].unit == "m/m");

    RpeEvaluator::Config by_time;
    by_time.delta_s = 1.0;  // 大于 0 时以它为准
    const auto time = RpeEvaluator(by_time).evaluate(FixedInputs(estimate, truth));
    REQUIRE(find(time, "translation.rmse") != nullptr);
    CHECK(find(time, "translation.rmse")->unit == "m");
    CHECK(find(time, "translation.rmse")->value == Approx(1.0).margin(1e-6));  // 1 秒多走 1 米

    // 轨迹不够一个间隔：没有段，不给数
    RpeEvaluator::Config too_long;
    too_long.delta_m = 100.0;
    const auto none = RpeEvaluator(too_long).evaluate(FixedInputs(estimate, truth));
    CHECK(none.samples == 0);
    CHECK(none.scalars.empty());
    CHECK(noted(none, "RPE 无结果"));
}

TEST_CASE("写法不对在构造时报：装配时试构造一次就能发现", "[eval][evaluators]") {
    const auto ate_rejects = [](auto change) {
        AteEvaluator::Config config;
        change(config);
        CHECK_THROWS_AS(AteEvaluator{config}, std::invalid_argument);
    };
    ate_rejects([](auto& c) { c.alignment = "se4"; });
    ate_rejects([](auto& c) { c.association = "closest"; });
    ate_rejects([](auto& c) { c.estimate = "map"; });
    ate_rejects([](auto& c) { c.tolerance_s = -0.01; });
    ate_rejects([](auto& c) { c.max_interpolation_gap_s = std::nan(""); });

    RpeEvaluator::Config neither;
    neither.delta_m = 0.0;
    neither.delta_s = 0.0;
    CHECK_THROWS_AS(RpeEvaluator{neither}, std::invalid_argument);  // 两种间隔都没给
    RpeEvaluator::Config negative;
    negative.delta_m = -1.0;
    CHECK_THROWS_AS(RpeEvaluator{negative}, std::invalid_argument);
}
