// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file baseline.hpp
/// 精度基线与门禁：让"改完之后精度有没有退"成为一个可回答的问题。
///
/// 基线是入仓的文件，不是某个人记忆里的数字。每条基线绑定数据集，可以同时绑定有效身份、
/// 真值的内容摘要与各评测函数的协议：换了配置、换了真值、换了协议的结果都不该拿来与它比，
/// 所以比对时先核对它们，不一致就如实报告而不是默默通过。
///
/// 门限带方向：ate.translation.rmse 是越小越好，ate.coverage 是越大越好。
/// 把两者混在一个 "threshold" 字段里，早晚会写反一次而且不会被发现。
///
/// **无法评价不等于通过**。"一条都没关联上"、"指标是 NaN"是这次评测没有产生有效结论；
/// "身份或协议与基线不符"是这次结论不可与基线比较。两类都不该让流水线绿灯通过，
/// 所以结论分四档。
///
/// 有效性检查**独立于有没有加载基线**：没有基线时同样要回答"这次测出东西了吗"。
///
/// 样本数**按评测函数分别核验**：ate 有三对关联、rpe 一段都没有时，只看 ate 的样本数
/// 会让一条 rpe 门限凭 0 轻松过线。指标名的第一段就是评测函数的名字。

#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace simpleslam::eval {

class BaselineError : public std::runtime_error {
public:
    explicit BaselineError(const std::string& what) : std::runtime_error(what) {}
};

struct MetricBound {
    std::string metric;
    double max_value = std::numeric_limits<double>::infinity();
    double min_value = -std::numeric_limits<double>::infinity();

    [[nodiscard]] bool hasUpper() const noexcept {
        return max_value != std::numeric_limits<double>::infinity();
    }
    [[nodiscard]] bool hasLower() const noexcept {
        return min_value != -std::numeric_limits<double>::infinity();
    }
};

/// 一个评测函数的协议：类型与参数，值都是规范化的文本。两份完全相同才可比——
/// 同一条轨迹按 SE3 与按 Sim3 对齐，ATE 可以差一个量级。
using Protocol = std::map<std::string, std::string>;

struct DatasetBaseline {
    std::string dataset;
    std::string config_hash;  ///< 产生这条基线的有效身份；空表示不限定
    /// 录这条基线时真值的内容摘要。给了就必须与本次一致：换了真值的误差不可比。
    std::string groundtruth_digest;
    std::string recorded_at;
    std::string note;
    std::vector<MetricBound> bounds;
    /// 门限用到的每个评测函数至少要有多少个样本，结论才算数。低于它是"没测出来"。
    double min_samples = 1.0;
    /// 录这条基线时各评测函数的协议，按名字。给了就必须与本次一致，否则不可比较。
    std::map<std::string, Protocol> evaluators;
    /// 这是一条"前缀运行"的基线：只跑了数据的一段。完整运行与前缀运行互不可比。
    bool prefix = false;
};

/// 基线自身的合法性。基线是入仓文件，一样可能写错；写错的基线会静静地
/// 放行一切（门限是 NaN、上下限反了、一条门限都没有）。返回空串表示合法。
[[nodiscard]] std::string validateBaseline(const DatasetBaseline& baseline);

/// 一次比对的结论。Pass 之外的三档各有各的原因，不能混成一个 bool。
enum class BaselineOutcome : std::uint8_t {
    kPass,          ///< 有效、可比较、全部过线
    kFail,          ///< 有效、可比较、有指标超出门限
    kInvalid,       ///< 本次评测没有产生有效结论（无样本、非有限值、缺指标）
    kIncomparable,  ///< 结论有效，但与这条基线不可比较（身份、真值或协议不符）
};

[[nodiscard]] constexpr const char* toString(BaselineOutcome outcome) noexcept {
    switch (outcome) {
        case BaselineOutcome::kPass: return "pass";
        case BaselineOutcome::kFail: return "fail";
        case BaselineOutcome::kInvalid: return "invalid";
        case BaselineOutcome::kIncomparable: return "incomparable";
    }
    return "invalid";
}
[[nodiscard]] std::optional<BaselineOutcome> parseBaselineOutcome(std::string_view text);

/// 一条门限的比对：过线的与没过线的都记下，结果文件里逐项列出。
struct BaselineCheck {
    std::string metric;
    double measured = 0.0;
    double limit = 0.0;
    bool upper = true;  ///< true：上限；false：下限
    bool passed = true;
};

struct BaselineVerdict {
    BaselineOutcome outcome = BaselineOutcome::kPass;
    /// 有效身份与基线不符。结论本身可能有效，但不能与这条基线比。
    bool config_mismatch = false;
    std::vector<BaselineCheck> checks;         ///< 每条门限的比对
    std::vector<std::string> missing_metrics;  ///< 基线要求但本次没测出来的
    std::vector<std::string> invalid_metrics;  ///< 非有限值（NaN / inf）
    std::string detail;

    /// 只有 kPass 才算通过。CI 默认只接受它。
    [[nodiscard]] bool passed() const noexcept { return outcome == BaselineOutcome::kPass; }
    /// 没过线的那几条。
    [[nodiscard]] std::vector<BaselineCheck> failures() const;
};

/// 只做有效性判断，不与任何基线比较。
///
/// 非有限指标一律无效；`required` 里的每个评测函数要有至少 `min_samples` 个样本
/// （指标 `<名字>.samples`）。一个评测函数有样本不能替另一个作数。
[[nodiscard]] BaselineVerdict checkMeasurementValidity(
    const std::map<std::string, double>& measured, double min_samples,
    const std::vector<std::string>& required);

/// 与一条基线比对：基线自身合法 → 有效性 → 门限 → 可比性（协议、真值、身份）。
/// 不可比较时照样附上逐项比对，但主结论只能是 Incomparable。
[[nodiscard]] BaselineVerdict checkBaseline(
    const DatasetBaseline& baseline, const std::map<std::string, double>& measured,
    const std::string& config_hash = {},
    const std::map<std::string, Protocol>* evaluators = nullptr,
    const std::string& groundtruth_digest = {});

/// 产生这条轨迹的那次运行是否完整。
///
/// 独立评测一条轨迹时，往往不知道产生它的运行是否完整——"不知道"是一个独立的状态，
/// 不能默认成"完整"。它只能来自两处：本次运行自己的结局，或者**绑定了这条轨迹内容**
/// 的运行结果；没有证据时可以人工声明完整，但声明要留在结果里。
enum class RunCompleteness : std::uint8_t {
    kComplete,          ///< 运行结局证明完整
    kIncomplete,        ///< 运行结局证明不完整
    kUnknown,           ///< 没有证据
    kDeclaredComplete,  ///< 没有证据，由使用者人工声明完整
};

[[nodiscard]] constexpr const char* toString(RunCompleteness completeness) noexcept {
    switch (completeness) {
        case RunCompleteness::kComplete: return "complete";
        case RunCompleteness::kIncomplete: return "incomplete";
        case RunCompleteness::kUnknown: return "unknown";
        case RunCompleteness::kDeclaredComplete: return "declared_complete";
    }
    return "unknown";
}
[[nodiscard]] std::optional<RunCompleteness> parseRunCompleteness(std::string_view text);

/// 一次门禁判定的全部输入。任务的执行器、单独评测与 simpleslam-eval 都经由 gate()
/// 下结论，判定规则只有这一份。
struct GateRequest {
    /// 全部标量（`<名字>.<量>.<统计>`）与各评测函数的样本数（`<名字>.samples`）。
    std::map<std::string, double> metrics;
    /// 要匹配的基线名。有基线文件时必填：不给就无从匹配，不会默认取第一条。
    std::string dataset;
    std::string config_hash;
    std::string groundtruth_digest;
    /// 这次跑了哪些评测函数、各自的协议。
    std::map<std::string, Protocol> evaluators;
    /// 产生这条轨迹的运行是否完整。默认"未知"：没有证据就不假定完整。
    RunCompleteness completeness = RunCompleteness::kUnknown;
    /// 允许拿不完整的运行下结论（前缀分析）。只能与 prefix 基线比较。
    bool allow_incomplete = false;
    /// 基线列表；为空表示没有基线，只做有效性判断。
    const std::vector<DatasetBaseline>* baselines = nullptr;
};

/// 有效性 → 完整性 → 身份匹配 → 门限比较，按这个顺序下结论。
///
/// 没有匹配的基线时，至少要有一个评测函数测出了东西；有基线时，门限用到的每个评测函数
/// 都要有足够的样本。完整性未知时，没有基线也照样给出有效的测量，但结论里写明
/// "完整性未知"；有基线时不可比较——说不清是否覆盖全程的数字，不能拿去和全程基线比高低。
[[nodiscard]] BaselineVerdict gate(const GateRequest& request);

/// 结论对应的进程退出码：通过 0，失败 3，无效 4，不可比较 5（允许时为 0）。
[[nodiscard]] int exitCodeFor(const BaselineVerdict& verdict, bool allow_incomparable = false);

[[nodiscard]] std::vector<DatasetBaseline> loadBaselines(const std::string& path);
void writeBaselines(const std::string& path, const std::vector<DatasetBaseline>& baselines);

}  // namespace simpleslam::eval
