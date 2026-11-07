// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 内置评测函数 ate、rpe、kitti、timing：只读输入、只返回值；写法不对在构造时报；没测出
/// 东西不给数；单位随取段方式；实际用的对齐方式写进说明；逐样本误差进表格；多序列要的权重
/// 随标量给出；评测输入的可选能力没有时明确报错。

#include <SimpleSLAM/eval/evaluators.hpp>
#include <SimpleSLAM/testing/evaluator_contract.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

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
class FixedInputs final : public IEvaluationInputs {
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

// ── 契约：所有评测函数都要满足 ──

TEMPLATE_LIST_TEST_CASE("评测函数契约", "[contract][eval]", BuiltinEvaluators) {
    simpleslam::testing::checkEvaluatorContract<TestType>();
}

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

namespace {

/// 只带度量的输入：timing 只读度量，不要真值。
class MetricInputs final : public IEvaluationInputs {
public:
    explicit MetricInputs(std::vector<MetricSample> samples) : samples_(std::move(samples)) {}

    [[nodiscard]] ComparablePair pair(std::string_view /*estimate*/) const override {
        throw std::runtime_error("没有真值");
    }
    [[nodiscard]] std::optional<fw::TimeRange> targetWindow() const override {
        return std::nullopt;
    }
    [[nodiscard]] std::vector<MetricSample> metrics(std::string_view key) const override {
        asked = std::string(key);
        return samples_;
    }

    mutable std::string asked;

private:
    std::vector<MetricSample> samples_;
};

MetricSample duration(std::uint64_t frame, double value, std::string unit = "ms") {
    return MetricSample{.frame = frame,
                        .data_time = at(static_cast<int>(frame)),
                        .source = "odometry",
                        .value = value,
                        .unit = std::move(unit)};
}

}  // namespace

TEST_CASE("kitti：平移是百分比、旋转是每米的度数，权重是段数，逐段进表格", "[eval][evaluators]") {
    // 每 0.1 秒沿 x 走 0.1 米，共 1000 米；估计每步多走 1%
    const auto truth = line(10'001, 0.1);
    const auto longer = line(10'001, 0.101);
    const auto out = KittiEvaluator(KittiEvaluator::Config{}).evaluate(FixedInputs(longer, truth));

    REQUIRE(out.samples > 0);
    const auto* translation = find(out, "translation.mean");
    REQUIRE(translation != nullptr);
    CHECK(translation->value == Approx(1.0).epsilon(2e-3));
    CHECK(translation->unit == "%");
    CHECK(translation->label == "KITTI_t");
    CHECK(translation->weight == static_cast<double>(out.samples));  // 多序列按段数加权
    const auto* rotation = find(out, "rotation.mean");
    REQUIRE(rotation != nullptr);
    CHECK(rotation->unit == "deg/m");
    CHECK(rotation->label == "KITTI_r");
    REQUIRE(out.tables.size() == 1);
    CHECK(out.tables[0].name == "segments");
    CHECK(out.tables[0].rows.size() == out.samples);
    CHECK(out.tables[0].columns.size() == 4);
    CHECK(noted(out, "按时刻关联"));
    CHECK(noted(out, "真值换算过"));  // 输入给的换算说明照样带上

    // 走过的路不足 100 米：没有段，不给数，说明为什么
    const KittiEvaluator kitti(KittiEvaluator::Config{});
    const auto none = kitti.evaluate(FixedInputs(line(50, 1.0), line(50, 1.0)));
    CHECK(none.samples == 0);
    CHECK(none.scalars.empty());
    CHECK(noted(none, "不足 100 米"));

    KittiEvaluator::Config zero;
    zero.step_frames = 0;
    CHECK_THROWS_AS(KittiEvaluator{zero}, std::invalid_argument);
}

TEST_CASE("timing：读一个度量给 mean、median、p95、max，单位随度量", "[eval][evaluators]") {
    std::vector<MetricSample> samples;
    for (int i = 1; i <= 5; ++i) samples.push_back(duration(static_cast<std::uint64_t>(i), i));
    samples.push_back(MetricSample{.frame = std::nullopt,
                                   .data_time = std::nullopt,
                                   .source = "odometry",
                                   .value = std::nan(""),
                                   .unit = "ms"});
    const MetricInputs inputs(samples);
    const auto out = TimingEvaluator(TimingEvaluator::Config{}).evaluate(inputs);

    CHECK(inputs.asked == "track.duration");  // 默认读里程计每帧的耗时
    CHECK(out.samples == 5);                  // 非有限的不计
    CHECK(noted(out, "不是有限数"));
    CHECK(find(out, "duration.mean")->value == Approx(3.0));
    CHECK(find(out, "duration.mean")->label == "T_mean");
    CHECK(find(out, "duration.median")->value == Approx(3.0));
    CHECK(find(out, "duration.p95")->value == Approx(4.8));
    CHECK(find(out, "duration.p95")->label == "T_p95");
    CHECK(find(out, "duration.max")->value == Approx(5.0));
    CHECK(find(out, "duration.max")->unit == "ms");
    REQUIRE(out.tables.size() == 1);
    CHECK(out.tables[0].rows.size() == 6);
    // 不针对某条输入的度量：帧序号那一格是空的
    CHECK(std::isnan(std::get<double>(out.tables[0].rows.back()[0])));

    // 一条都没有、单位不一致：都不给数，说明为什么
    const auto empty = TimingEvaluator(TimingEvaluator::Config{}).evaluate(MetricInputs({}));
    CHECK(empty.samples == 0);
    CHECK(noted(empty, "没有记下度量 track.duration"));
    const auto mixed = TimingEvaluator(TimingEvaluator::Config{})
                           .evaluate(MetricInputs({duration(1, 1.0), duration(2, 0.002, "s")}));
    CHECK(mixed.samples == 0);
    CHECK(noted(mixed, "单位不一致"));

    TimingEvaluator::Config keyless;
    keyless.key = "";
    CHECK_THROWS_AS(TimingEvaluator{keyless}, std::invalid_argument);
}

TEST_CASE("评测输入没有度量、事件或单独的估计时明确报错，不当作空", "[eval][evaluators]") {
    const FixedInputs inputs(line(3, 1.0), line(3, 1.0));  // 只实现了 pair 与 targetWindow
    CHECK_THROWS_AS(inputs.metrics("track.duration"), std::runtime_error);
    CHECK_THROWS_AS(inputs.events("odometry.lost"), std::runtime_error);
    CHECK_THROWS_AS(inputs.estimate("odometry"), std::runtime_error);
    CHECK_THROWS_AS(TimingEvaluator(TimingEvaluator::Config{}).evaluate(inputs),
                    std::runtime_error);
}

TEST_CASE("ate 收 posyaw：位置加航向的 4 自由度对齐", "[eval][evaluators]") {
    AteEvaluator::Config config;
    config.alignment = "posyaw";
    const auto out = AteEvaluator(config).evaluate(FixedInputs(line(10, 1.0, 0.5), line(10, 1.0)));
    REQUIRE(out.samples == 10);
    CHECK(find(out, "translation.rmse")->value == Approx(0.0).margin(1e-9));  // 平移被吸收
}
