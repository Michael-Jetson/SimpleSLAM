// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 精度基线：门限带方向、无法评价不算通过、配置哈希不符是"不可比较"而不是通过、
/// 缺指标算无效、报告可入仓。

#include <SimpleSLAM/eval/baseline.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>

using namespace simpleslam::eval;

namespace {

std::filesystem::path tempFile(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}

DatasetBaseline sampleBaseline() {
    DatasetBaseline baseline;
    baseline.dataset = "kitti/00";
    baseline.config_hash = "0123456789abcdef";
    baseline.recorded_at = "2026-09-21";
    baseline.bounds = {
        MetricBound{"ate.translation.rmse", 1.5, -std::numeric_limits<double>::infinity()},
        MetricBound{"ate.coverage", std::numeric_limits<double>::infinity(), 0.95},
    };
    return baseline;
}

}  // namespace

TEST_CASE("门限带方向：上限与下限不会写反", "[eval][baseline]") {
    const auto baseline = sampleBaseline();

    const auto good = checkBaseline(baseline, {{"ate.translation.rmse", 1.2},
                                               {"ate.coverage", 0.99},
                                               {"ate.pairs", 100.0}},
                                    "0123456789abcdef");
    CHECK(good.passed());
    CHECK(good.outcome == BaselineOutcome::kPass);
    CHECK_FALSE(good.config_mismatch);

    const auto too_big = checkBaseline(baseline, {{"ate.translation.rmse", 2.0},
                                                  {"ate.coverage", 0.99},
                                                  {"ate.pairs", 100.0}},
                                       "0123456789abcdef");
    REQUIRE(too_big.outcome == BaselineOutcome::kFail);
    REQUIRE(too_big.failures.size() == 1);
    CHECK(too_big.failures.front().upper);

    const auto too_small = checkBaseline(baseline, {{"ate.translation.rmse", 1.0},
                                                    {"ate.coverage", 0.5},
                                                    {"ate.pairs", 100.0}},
                                         "0123456789abcdef");
    REQUIRE(too_small.outcome == BaselineOutcome::kFail);
    REQUIRE(too_small.failures.size() == 1);
    CHECK_FALSE(too_small.failures.front().upper);
}

TEST_CASE("基线要求的指标没测出来算无效", "[eval][baseline]") {
    const auto verdict = checkBaseline(sampleBaseline(),
                                       {{"ate.coverage", 0.99}, {"ate.pairs", 100.0}},
                                       "0123456789abcdef");
    CHECK(verdict.outcome == BaselineOutcome::kInvalid);
    CHECK_FALSE(verdict.passed());
    REQUIRE(verdict.missing_metrics.size() == 1);
    CHECK(verdict.missing_metrics.front() == "ate.translation.rmse");
}

TEST_CASE("基线绑定了哈希，调用方不给就是不可比较", "[eval][baseline]") {
    // 缺少身份不是"默认相符"：无从判断这次结果能不能和这条基线比
    const auto verdict = checkBaseline(sampleBaseline(), {{"ate.translation.rmse", 0.1},
                                                          {"ate.coverage", 1.0},
                                                          {"ate.pairs", 100.0}});
    CHECK(verdict.outcome == BaselineOutcome::kIncomparable);
    CHECK(verdict.config_mismatch);
}

TEST_CASE("指标族各自验样本数", "[eval][baseline]") {
    // ATE 有 3 对、RPE 一对都没有时，一条 RPE 门限不能凭 rpe=0 过线
    DatasetBaseline baseline;
    baseline.dataset = "demo";
    baseline.bounds = {MetricBound{"rpe.translation.rmse", 0.5,
                                   -std::numeric_limits<double>::infinity()}};
    const auto verdict = checkBaseline(baseline, {{"ate.pairs", 3.0},
                                                  {"rpe.pairs", 0.0},
                                                  {"rpe.translation.rmse", 0.0}});
    CHECK(verdict.outcome == BaselineOutcome::kInvalid);
}

TEST_CASE("有效性检查不依赖有没有基线", "[eval][baseline]") {
    // 没有基线时同样要回答"这次测出东西了吗"，否则零关联会以"零误差"示人
    const auto verdict = checkMeasurementValidity({{"ate.pairs", 0.0},
                                                   {"ate.translation.rmse", 0.0}});
    CHECK(verdict.outcome == BaselineOutcome::kInvalid);
}

TEST_CASE("写错的基线不能静静地放行一切", "[eval][baseline]") {
    DatasetBaseline empty;
    empty.dataset = "demo";
    CHECK_FALSE(validateBaseline(empty).empty());  // 一条门限都没有

    DatasetBaseline nan_bound;
    nan_bound.dataset = "demo";
    nan_bound.bounds = {MetricBound{"ate.translation.rmse",
                                    std::numeric_limits<double>::quiet_NaN(),
                                    -std::numeric_limits<double>::infinity()}};
    CHECK_FALSE(validateBaseline(nan_bound).empty());  // NaN 门限的比较恒为假

    DatasetBaseline inverted;
    inverted.dataset = "demo";
    inverted.bounds = {MetricBound{"ate.coverage", 0.5, 0.9}};
    CHECK_FALSE(validateBaseline(inverted).empty());  // 下限大于上限

    DatasetBaseline bad_pairs = sampleBaseline();
    bad_pairs.min_pairs = 0.0;
    CHECK_FALSE(validateBaseline(bad_pairs).empty());
}

TEST_CASE("一条都关联不上不算通过", "[eval][baseline]") {
    // 两条毫无关联的轨迹 ATE=0、n=0，数字上过得了"RMSE < 1.5"，但它没有结论
    const auto verdict = checkBaseline(sampleBaseline(),
                                       {{"ate.translation.rmse", 0.0},
                                        {"ate.coverage", 1.0},
                                        {"ate.pairs", 0.0}},
                                       "0123456789abcdef");
    CHECK(verdict.outcome == BaselineOutcome::kInvalid);
    CHECK_FALSE(verdict.passed());
    CHECK_FALSE(verdict.detail.empty());
}

TEST_CASE("非有限指标不算通过", "[eval][baseline]") {
    const auto verdict = checkBaseline(
        sampleBaseline(),
        {{"ate.translation.rmse", std::numeric_limits<double>::quiet_NaN()},
         {"ate.coverage", 1.0},
         {"ate.pairs", 100.0}},
        "0123456789abcdef");
    CHECK(verdict.outcome == BaselineOutcome::kInvalid);
    REQUIRE(verdict.invalid_metrics.size() == 1);
    CHECK(verdict.invalid_metrics.front() == "ate.translation.rmse");
}

TEST_CASE("配置哈希不符是不可比较，不是通过", "[eval][baseline]") {
    const auto verdict = checkBaseline(sampleBaseline(),
                                       {{"ate.translation.rmse", 0.1},
                                        {"ate.coverage", 1.0},
                                        {"ate.pairs", 100.0}},
                                       "ffffffffffffffff");
    CHECK(verdict.outcome == BaselineOutcome::kIncomparable);
    CHECK_FALSE(verdict.passed());  // 数字过线，但这份基线不是为这套配置录的
    CHECK(verdict.config_mismatch);
}

TEST_CASE("基线文件往返", "[eval][baseline]") {
    const auto path = tempFile("simpleslam_baselines.yaml");
    writeBaselines(path.string(), {sampleBaseline()});
    const auto loaded = loadBaselines(path.string());

    REQUIRE(loaded.size() == 1);
    CHECK(loaded.front().dataset == "kitti/00");
    CHECK(loaded.front().config_hash == "0123456789abcdef");
    REQUIRE(loaded.front().bounds.size() == 2);
    std::filesystem::remove(path);
}

TEST_CASE("标量门限按上限理解", "[eval][baseline]") {
    const auto path = tempFile("simpleslam_baselines_scalar.yaml");
    {
        std::ofstream file(path);
        file << "baselines:\n  - dataset: demo\n    bounds:\n"
                "      ate.translation.rmse: 0.5\n      ate.coverage: {min: 0.9}\n";
    }
    const auto loaded = loadBaselines(path.string());
    REQUIRE(loaded.size() == 1);
    REQUIRE(loaded.front().bounds.size() == 2);
    const auto& rmse = loaded.front().bounds.front();
    CHECK(rmse.hasUpper());
    CHECK_FALSE(rmse.hasLower());
    std::filesystem::remove(path);
}

TEST_CASE("报告可渲染并落盘", "[eval][baseline]") {
    EvaluationReport report;
    report.dataset = "demo";
    report.config_hash = "abc";
    report.created_at = "2026-09-21T12:00:00Z";
    report.alignment = "se3";
    report.metrics = {{"ate.translation.rmse", 0.42}};
    report.verdict.outcome = BaselineOutcome::kPass;

    const auto text = renderReport(report);
    CHECK(text.find("ate.translation.rmse") != std::string::npos);
    CHECK(text.find("se3") != std::string::npos);

    const auto path = tempFile("simpleslam_report.yaml");
    writeReport(path.string(), report);
    CHECK(std::filesystem::exists(path));
    std::filesystem::remove(path);
}

TEST_CASE("报告的结论与指标读得回来", "[eval][baseline]") {
    EvaluationReport report;
    report.dataset = "demo";
    report.metrics = {{"ate.translation.rmse", 1.0 / 3.0}, {"ate.pairs", 20.0}};
    report.verdict.outcome = BaselineOutcome::kFail;
    report.verdict.detail = "ate.translation.rmse 超出上限";
    const auto path = tempFile("simpleslam_report_result.yaml");
    writeReport(path.string(), report);

    const ReportResult result = readReportResult(path.string());
    CHECK(result.outcome == BaselineOutcome::kFail);
    CHECK(result.detail == report.verdict.detail);
    CHECK(result.metrics == report.metrics);  // 实数按往返无损的写法落盘

    std::ofstream(path) << "metrics: {}\n";
    CHECK_THROWS_AS(readReportResult(path.string()), BaselineError);  // 没有结论就不是报告
    std::filesystem::remove(path);
}

TEST_CASE("没人要 RPE 时，凑不出 RPE 段不否定 ATE 的结论", "[eval][baseline]") {
    // 一条太短、凑不出一个 RPE 间隔的轨迹，不该连带把 ATE 也判成无效
    const auto verdict = checkMeasurementValidity({{"ate.pairs", 20.0},
                                                   {"ate.translation.rmse", 0.3},
                                                   {"rpe.pairs", 0.0},
                                                   {"rpe.translation.rmse", 0.0}});
    CHECK(verdict.outcome == BaselineOutcome::kPass);

    // 但基线一旦为 RPE 设了门限，它就必须真的有样本
    DatasetBaseline baseline;
    baseline.dataset = "demo";
    baseline.bounds = {MetricBound{"rpe.translation.rmse", 0.5,
                                   -std::numeric_limits<double>::infinity()}};
    CHECK(checkBaseline(baseline, {{"ate.pairs", 20.0},
                                   {"rpe.pairs", 0.0},
                                   {"rpe.translation.rmse", 0.0}})
              .outcome == BaselineOutcome::kInvalid);
}
