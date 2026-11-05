// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file baseline.hpp
/// 精度基线与评测报告：让"改完之后精度有没有退"成为一个可回答的问题。
///
/// 基线是入仓的文件，不是某个人记忆里的数字。每条基线绑定
/// 数据集 + 配置哈希：换了配置的基线不该拿来卡另一份配置的结果，
/// 所以比对时会先核对哈希，不一致就如实报告而不是默默通过。
///
/// 门限带方向：ate.translation.rmse 是越小越好，ate.coverage 是越大越好。
/// 把两者混在一个 "threshold" 字段里，早晚会写反一次而且不会被发现。
///
/// **无法评价不等于通过**。"一条都没关联上"（ATE=0，n=0）、"指标是 NaN"是
/// 这次评测没有产生有效结论；"配置哈希与基线不符"或"基线绑定了哈希而调用方没给"
/// 是这次结论不可与基线比较。两类都不该让流水线绿灯通过，所以结论分四档。
///
/// 有效性检查**独立于有没有加载基线**：没有基线时同样要回答"这次测出东西了吗"，
/// 否则一次零关联的运行会以"零误差"的面目出现在报告里。
///
/// 样本数**按指标族分别核验**：ATE 有三对关联、RPE 一对都没有时，
/// 只看 ate.pairs 会让一条 RPE 门限凭 rpe=0 轻松过线。

#include <SimpleSLAM/eval/metrics.hpp>

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

/// 一次评测所用的协议。它和配置哈希不是一回事：同一份配置也可以用不同的容差、
/// 对齐方式或 RPE 间隔来评测，结论不可直接互比，所以协议要随结果一起保存，
/// 基线也可以绑定它——协议不同的结果与这条基线不可比较。
struct EvaluationProtocol {
    std::string alignment;              ///< 实际采用的对齐方式
    std::string alignment_requested;    ///< 请求的对齐方式（样本不足时可能退化）
    std::string association;            ///< "nearest" 或 "interpolate"
    double association_tolerance_s = 0.0;  ///< 最近邻关联容差（秒）
    double max_interpolation_gap_s = 0.0;  ///< 插值关联的最大括点间隔（秒）
    std::string rpe_delta_kind;         ///< "time" 或 "distance"
    double rpe_delta = 0.0;             ///< 间隔大小（秒或米）
    bool rpe_normalized = false;        ///< 是否按间隔归一化
    double window_begin_s = 0.0;        ///< 评测时间窗（秒），两端为零表示不限
    double window_end_s = 0.0;

    /// 可比性只看影响数字的那几项：请求的对齐方式不参与（实际采用的才算）。
    [[nodiscard]] bool sameMeasurement(const EvaluationProtocol& other) const noexcept {
        return alignment == other.alignment && association == other.association
               && association_tolerance_s == other.association_tolerance_s
               && max_interpolation_gap_s == other.max_interpolation_gap_s
               && rpe_delta_kind == other.rpe_delta_kind && rpe_delta == other.rpe_delta
               && rpe_normalized == other.rpe_normalized
               && window_begin_s == other.window_begin_s && window_end_s == other.window_end_s;
    }
};

struct DatasetBaseline {
    std::string dataset;
    std::string config_hash;  ///< 产生这条基线的配置哈希；空表示不限定
    std::string recorded_at;
    std::string note;
    std::vector<MetricBound> bounds;
    /// 至少要有多少对有效关联，结论才算数。低于它是"没测出来"，不是"测得很好"。
    double min_pairs = 1.0;
    /// 录这条基线时的评测协议。给了就必须与本次一致，否则不可比较。
    std::optional<EvaluationProtocol> protocol;
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
    kInvalid,       ///< 本次评测没有产生有效结论（无关联、非有限值、缺指标）
    kIncomparable,  ///< 结论有效，但与这条基线不可比较（配置哈希不符）
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

struct BaselineFailure {
    std::string metric;
    double measured = 0.0;
    double bound = 0.0;
    bool upper = true;  ///< true：超过上限；false：低于下限
};

struct BaselineVerdict {
    BaselineOutcome outcome = BaselineOutcome::kPass;
    /// 配置哈希与基线不符。结论本身可能有效，但不能与这条基线比。
    bool config_mismatch = false;
    std::vector<BaselineFailure> failures;
    std::vector<std::string> missing_metrics;  ///< 基线要求但本次没测出来的
    std::vector<std::string> invalid_metrics;  ///< 非有限值（NaN / inf）
    std::string detail;

    /// 只有 kPass 才算通过。CI 默认只接受它。
    [[nodiscard]] bool passed() const noexcept { return outcome == BaselineOutcome::kPass; }
};

/// 只做有效性判断，不与任何基线比较。没有基线时也必须跑这一步。
///
/// `required_families` 列出哪些指标族必须真的有样本。默认只要求 `ate`：
/// 它是主结论，一对都没关联上就是"这次没测出东西"。RPE 只有在基线为它设了门限时
/// 才成为必需——否则一条太短、凑不出一个 RPE 间隔的轨迹会连带把 ATE 的结论也否掉。
///
/// 非有限指标一律无效，与是哪个族无关。
[[nodiscard]] BaselineVerdict checkMeasurementValidity(
    const std::map<std::string, double>& measured, double min_pairs = 1.0,
    const std::vector<std::string>& required_families = {"ate"});

[[nodiscard]] BaselineVerdict checkBaseline(const DatasetBaseline& baseline,
                                            const std::map<std::string, double>& measured,
                                            const std::string& config_hash = {},
                                            const EvaluationProtocol* protocol = nullptr);

/// 产生这条轨迹的那次运行是否完整。
///
/// 独立评测一条轨迹时，往往不知道产生它的运行是否完整——"不知道"是一个独立的状态，
/// 不能默认成"完整"。它只能来自两处：本次运行自己的结局，或者**绑定了这条轨迹内容**
/// 的运行报告；没有证据时可以人工声明完整，但声明要留在报告里。
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

/// 一次门禁判定的全部输入。CLI 与示例入口都经由 gate() 下结论，判定规则只有这一份。
struct GateRequest {
    std::map<std::string, double> metrics;
    /// 要匹配的基线名。有基线文件时必填：不给就无从匹配，不会默认取第一条。
    std::string dataset;
    std::string config_hash;
    EvaluationProtocol protocol;
    /// 产生这条轨迹的运行是否完整。默认"未知"：没有证据就不假定完整。
    RunCompleteness completeness = RunCompleteness::kUnknown;
    /// 允许拿不完整的运行下结论（前缀分析）。只能与 prefix 基线比较。
    bool allow_incomplete = false;
    /// 基线列表；为空表示没有基线，只做有效性判断。
    const std::vector<DatasetBaseline>* baselines = nullptr;
};

/// 有效性 → 完整性 → 身份匹配 → 门限比较，按这个顺序下结论。
///
/// 任一环节不成立就给出对应的档位并停下：无效（没测出东西、运行不完整）、
/// 不可比较（完整性未知却要与基线比、没指定数据集、没有匹配的基线、配置或协议不符、
/// 前缀对完整）、失败（有效可比但超限）、通过。报告里的结论与退出码都取自这里。
///
/// 完整性未知时，没有基线也照样给出有效的误差测量，但结论里写明"完整性未知"；
/// 有基线时不可比较——说不清是否覆盖全程的数字，不能拿去和全程基线比高低。
[[nodiscard]] BaselineVerdict gate(const GateRequest& request);

/// 结论对应的进程退出码：通过 0，失败 3，无效 4，不可比较 5（允许时为 0）。
[[nodiscard]] int exitCodeFor(const BaselineVerdict& verdict, bool allow_incomparable = false);

[[nodiscard]] std::vector<DatasetBaseline> loadBaselines(const std::string& path);
void writeBaselines(const std::string& path, const std::vector<DatasetBaseline>& baselines);

/// 一次评测的完整记录。入仓之后，"上次是多少"不再依赖记忆。
struct EvaluationReport {
    std::string dataset;
    std::string config_hash;
    std::string code_revision;
    std::string created_at;
    std::string alignment;
    /// 本次评测所用的协议。只写配置哈希不足以固定结论。
    EvaluationProtocol protocol;
    /// 输入是否完整。回放一段被截断的录制时为 false——同样的数字，
    /// 一个是跑完全程测出来的，一个是只跑了前半段，不能混为一谈。
    bool input_complete = true;
    /// 产生这条轨迹的运行是否完整，以及这个判断从哪里来。
    RunCompleteness completeness = RunCompleteness::kUnknown;
    /// 完整性的出处：本次运行，或绑定了轨迹内容的运行报告路径。
    std::string completeness_source;
    /// 人工声明完整时的说明。声明不是证据，所以原话留在报告里。
    std::string completeness_declaration;
    /// 这份报告描述的估计轨迹：文件与内容摘要。别处拿这份报告当运行凭据时，
    /// 必须核对轨迹内容与这里一致——报告只为它描述的那条轨迹作证。
    std::string trajectory_file;
    std::string trajectory_digest;
    /// 运行不完整的原因（RunOutcome::reasons）。为空表示输入、处理、输出都完整。
    std::vector<std::string> run_problems;
    /// 来源说明：同源关系、缺省解释等（SourceDescriptor::notes）。
    std::vector<std::string> notes;
    /// 实际生效、影响结论的输入身份：解析后的打包策略及其出处、标定内容哈希、
    /// 有效配置哈希。配置文件里写了什么不等于实际用了什么，要记的是后者。
    std::map<std::string, std::string> effective;
    std::map<std::string, double> metrics;
    BaselineVerdict verdict;
};

void writeReport(const std::string& path, const EvaluationReport& report);
[[nodiscard]] std::string renderReport(const EvaluationReport& report);

/// 运行报告里可以当作**运行凭据**的那几项：完整性、它绑定的轨迹、评测协议。
struct RunManifest {
    RunCompleteness completeness = RunCompleteness::kUnknown;
    std::string trajectory_file;
    std::string trajectory_digest;
    std::vector<std::string> run_problems;
    EvaluationProtocol protocol;
};

/// 读一份运行报告里的运行凭据。读不懂就抛：凭据含糊不如没有。
[[nodiscard]] RunManifest readRunManifest(const std::string& path);

/// 把指标摊平成 map，供基线比对使用。
[[nodiscard]] std::map<std::string, double> toMetricMap(const AteResult& ate, const RpeResult& rpe);

}  // namespace simpleslam::eval
