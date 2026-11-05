// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// @file main.cpp
/// 一个能跑的 LIO 入口：配置 → 数据集 → 装配 → 跑完 → 导出轨迹 → 对真值评测。
///
/// 这个文件里没有算法。算法在 my_lio.hpp 的 `track()` 里，换掉它就行。
/// 之所以把入口也写出来，是因为"跑起来"这件事本身有一堆容易错的接缝：
/// 标定从哪来、打包策略跟数据集怎么对上、真值在哪个系、时间戳怎么对齐。
/// 这些都在下面一次性解决掉，你不用再碰。
///
/// 用法：
///   simpleslam-lio-skeleton <配置.yaml> [覆盖...]
///   simpleslam-lio-skeleton configs/new_stack/kitti_odometry.yaml dataset.max_scans=200

#include "my_lio.hpp"

#include <SimpleSLAM/config/calibration_io.hpp>
#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/eval/baseline.hpp>
#include <SimpleSLAM/eval/metrics.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>
#include <SimpleSLAM/app/bindings.hpp>
#include <SimpleSLAM/plugins/sources/kitti.hpp>
#include <SimpleSLAM/record/diagnostic_file_sink.hpp>
#include <SimpleSLAM/record/reader.hpp>
#include <SimpleSLAM/record/writer.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace app = simpleslam::app;
namespace cfg = simpleslam::config;
namespace eval = simpleslam::eval;
namespace fw = simpleslam::fw;
namespace kitti = simpleslam::plugin::kitti;
namespace modules = simpleslam::modules;

/// 数据集、它自带的标定、它的真值、它推荐的打包策略——四样一起给出来。
/// 分开取就会出现"换了数据集但打包策略忘了改"这种错。
struct Dataset {
    std::unique_ptr<fw::ISensorSource> source;
    fw::CalibrationData calibration;
    fw::Trajectory ground_truth;
    bool has_ground_truth = false;
    /// 读取器按**实测数据**给出的推荐策略。用不用它由配置决定。
    fw::PacketizerPolicy recommended_policy;
    std::size_t frames = 0;
};

/// 最终生效的打包策略，以及它的出处。
///
/// 推荐值不能悄悄压过用户写的配置——那样用户看着自己填的 post_support_s
/// 却不知道它没生效。所以这里是一个显式选择，出处写进产物。
struct ResolvedPolicy {
    fw::PacketizerPolicy policy;
    std::string origin;  // "source" 或 "config"
};

ResolvedPolicy resolvePolicy(const cfg::SlamConfig& config, const Dataset& dataset) {
    if (config.packetizer.source_policy == "config") {
        return {app::toPacketizerPolicy(config.packetizer), "config"};
    }
    return {dataset.recommended_policy, "source"};
}

Dataset openDataset(const cfg::SlamConfig& config) {
    Dataset dataset;
    const auto& section = config.dataset;

    if (section.kind == "kitti_odometry") {
        kitti::OdometryOptions options;
        options.skip_scans = static_cast<std::size_t>(section.skip_scans);
        options.max_scans = static_cast<std::size_t>(section.max_scans);
        // 读取器换算 body 系真值用的是数据集自己的 Tr。外部标定会取代它，两者可能对不上：
        // 有外部标定时读原生的相机系真值，由评测按生效的外参换算（matchTrackedSensor）
        const bool native_ground_truth =
            section.groundtruth_frame == "camera" || !config.calibration.file.empty();
        options.ground_truth_frame = native_ground_truth ? kitti::GroundTruthFrame::kCamera
                                                         : kitti::GroundTruthFrame::kBody;
        auto source = std::make_unique<kitti::OdometrySource>(section.path, options);
        dataset.calibration = source->calibration();
        dataset.has_ground_truth = source->hasGroundTruth();
        if (dataset.has_ground_truth) dataset.ground_truth = source->groundTruth();
        dataset.recommended_policy = source->packetizerPolicy();
        dataset.frames = source->frameCount();
        dataset.source = std::move(source);
        return dataset;
    }
    if (section.kind == "kitti_raw") {
        kitti::RawOptions options;
        options.skip_scans = static_cast<std::size_t>(section.skip_scans);
        options.max_scans = static_cast<std::size_t>(section.max_scans);
        // 时间异常的帧默认拒绝读取；修补必须显式打开，修补了哪些帧进有效身份
        options.repair_scan_timing = section.repair_scan_timing;
        // 读取器按"最后一帧终点加上余量之后的第一条样本"留右括点：余量要与实际生效的
        // 打包策略一致，否则末帧等不到
        if (config.packetizer.source_policy == "config") {
            options.post_support = app::secondsToDuration(config.packetizer.post_support_s);
        }
        auto source = std::make_unique<kitti::RawSource>(section.path, options);
        dataset.calibration = source->calibration();
        dataset.has_ground_truth = source->hasGroundTruth();
        if (dataset.has_ground_truth) dataset.ground_truth = source->groundTruth();
        dataset.recommended_policy = source->packetizerPolicy();
        dataset.frames = source->frameCount();
        dataset.source = std::move(source);
        return dataset;
    }
    if (section.kind == "replay") {
        auto source = std::make_unique<simpleslam::record::ReplaySource>(section.path);
        dataset.recommended_policy = app::toPacketizerPolicy(config.packetizer);
        dataset.source = std::move(source);
        return dataset;
    }
    throw std::runtime_error("未知的 dataset.kind: " + section.kind
                             + "（可选 kitti_odometry / kitti_raw / replay）");
}

void printStats(const char* title, const eval::Statistics& stats, const char* unit) {
    std::cout << std::left << std::setw(12) << title << std::right << std::fixed
              << std::setprecision(4) << " rmse=" << std::setw(10) << stats.rmse
              << " mean=" << std::setw(10) << stats.mean << " max=" << std::setw(10) << stats.max
              << "  [" << unit << "] n=" << stats.count << '\n';
}


/// 本次运行实际生效、影响结论的输入。进报告，并作为基线可比性的身份。
std::map<std::string, std::string> effectiveInputs(const cfg::SlamConfig& config,
                                                   const fw::CalibrationData& calibration,
                                                   const ResolvedPolicy& resolved,
                                                   const fw::SourceDescriptor& descriptor,
                                                   const simpleslam::plugin::OdometryBase& odometry) {
    const auto seconds = [](fw::Duration d) {
        std::ostringstream out;
        out << std::setprecision(9) << std::chrono::duration<double>(d).count();
        return out.str();
    };
    std::map<std::string, std::string> effective;
    effective["config_hash"] = cfg::configHash(config);
    effective["calibration_hash"] = cfg::calibrationHash(calibration);
    effective["packetizer.origin"] = resolved.origin;
    effective["packetizer.imu_required"] =
        resolved.policy.imu == fw::ImuRequirement::kRequired ? "true" : "false";
    effective["packetizer.post_support_s"] = seconds(resolved.policy.post_support);
    effective["packetizer.pre_support_s"] = seconds(resolved.policy.pre_support);
    effective["packetizer.max_imu_gap_s"] = seconds(resolved.policy.max_imu_gap);
    effective["source.name"] = descriptor.name;
    effective["source.imu_rate_hz"] = std::to_string(descriptor.imu_rate_hz);
    // 修补过的数据不是原来那份数据：政策与受影响的帧都进身份
    effective["source.timing_repair"] = descriptor.timing_repair.empty() ? "none"
                                                                          : descriptor.timing_repair;
    std::string repaired;
    for (const auto frame : descriptor.repaired_frames) {
        repaired += (repaired.empty() ? "" : ",") + std::to_string(frame);
    }
    effective["source.repaired_frames"] = repaired;
    // 算法对缺支撑的处理与它自己的连续性上限，都会改变哪些帧有位姿
    effective["algorithm.name"] = odometry.name();
    effective["algorithm.support_gap_policy"] =
        simpleslam::plugin::toString(odometry.supportGapPolicy());
    const auto limit = odometry.maxImuGap();
    effective["algorithm.max_imu_gap_s"] = limit ? seconds(*limit) : "none";
    return effective;
}

/// 让真值与估计跟踪同一个传感器，换算只用**生效的**标定。返回换算说明，进报告备注。
///
/// 读取器贴的 "body" 是按数据集自带标定定义的 body，绑在 `descriptor.body_sensor` 上。
/// 外部标定取代数据集标定时，body 的定义可能随之改变：先把真值改标成它实际跟踪的
/// 那个传感器，再按生效外参换算。odometry 的 body 系真值还用到了数据集的 Tr，所以有
/// 外部标定时直接读原生相机系真值（见 openDataset），道理相同。
///
/// 默认在 body 上比较（换算真值）；配置要求相机系真值时在真值的传感器上比较（换算估计）。
/// 左乘对齐消不掉右侧外参，所以这一步必须显式做。
std::vector<std::string> matchTrackedSensor(fw::Trajectory& estimate,
                                            fw::Trajectory& ground_truth,
                                            const fw::CalibrationData& calibration,
                                            const fw::SourceDescriptor& descriptor,
                                            bool external_calibration,
                                            bool compare_at_ground_truth) {
    std::vector<std::string> notes;
    // 外部真值文件不声明跟踪哪个传感器：由使用者负责
    if (ground_truth.tracked().empty()) return notes;
    if (external_calibration && ground_truth.tracked() == "body" && !descriptor.body_sensor.empty()
        && calibration.T_body_sensor(descriptor.body_sensor)) {
        ground_truth.setTracked(descriptor.body_sensor);
    }
    const auto extrinsic = [&](const std::string& sensor) {
        const auto T = calibration.T_body_sensor(sensor);
        if (!T) throw std::runtime_error("比较要用 " + sensor + " 的外参，但生效的标定里没有它");
        return *T;
    };
    if (estimate.tracked() != "body") {
        throw std::runtime_error("估计跟踪的不是 body（" + estimate.tracked() + "），无从换算");
    }
    const std::string target = compare_at_ground_truth ? ground_truth.tracked() : "body";
    if (ground_truth.tracked() != target) {
        const std::string from = ground_truth.tracked();
        ground_truth = eval::rightMultiplied(ground_truth, extrinsic(from).inverse(), target);
        notes.push_back("真值从 " + from + " 按生效外参换算到 " + target);
    }
    if (estimate.tracked() != target) {
        estimate = eval::rightMultiplied(estimate, extrinsic(target), target);
        notes.push_back("估计从 body 按生效外参换算到 " + target);
    }
    return notes;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "用法: simpleslam-lio-skeleton <配置.yaml> [路径=值 ...]\n";
        return 2;
    }

    try {
        auto config = cfg::loadConfigFile(argv[1], std::vector<std::string>(argv + 2, argv + argc));
        std::cout << "配置: " << argv[1] << "  哈希: " << cfg::configHash(config) << '\n';

        // 读取来源：它只负责说"有哪些标定、需要哪些外参"，缺外参时读取本身不失败
        Dataset dataset = openDataset(config);
        // 外部标定文件优先于数据集自带的：数据集的标定是出厂值，重标过就该用新的。
        // 必需的外参补没补齐，由 SlamSystem::init() 在装配完成时核对
        if (!config.calibration.file.empty()) {
            dataset.calibration = cfg::loadCalibrationFile(config.calibration.file);
        }

        // 数据源的能力描述：补没补偿过、参考时刻是什么、惯性流的实测频率。
        // 这些是装配要据以决定打包与去畸变的事实，不能靠类名暗示。
        const auto descriptor = dataset.source->descriptor();
        const auto resolved = resolvePolicy(config, dataset);
        std::cout << "数据: " << dataset.source->name() << "  帧数: " << dataset.frames << '\n'
                  << "标定: " << dataset.calibration.device << "  哈希: "
                  << cfg::calibrationHash(dataset.calibration) << '\n'
                  << "打包策略: " << resolved.origin << "  post_support=" << std::fixed
                  << std::setprecision(3)
                  << std::chrono::duration<double>(resolved.policy.post_support).count() << "s"
                  << "  惯性实测: " << std::setprecision(1) << descriptor.imu_rate_hz << "Hz\n";

        // 录制：配置里打开了就真的录。写了录制、入口却不录，比明说不支持更糟。
        // 录制只是旁路，不改变来源的描述与有效身份
        std::shared_ptr<simpleslam::record::RecordWriter> recorder;
        if (config.recording.enabled) {
            simpleslam::record::RecordMetadata metadata;
            metadata.name = config.name;
            metadata.config_hash = cfg::configHash(config);
            metadata.seed = config.seed;
            recorder = std::make_shared<simpleslam::record::RecordWriter>(config.recording.output,
                                                                         metadata);
            dataset.source = std::make_unique<simpleslam::record::RecordingSource>(
                std::move(dataset.source), recorder);
        }

        modules::SlamSystem system(app::toSystemOptions(config));
        // 版本由提交入口生成，不由文件声明：帧产物上标的就是这个号
        std::cout << "标定版本: " << system.setCalibration(dataset.calibration) << '\n';
        system.setSource(std::move(dataset.source), resolved.policy);
        auto& odometry = system.setOdometry<example::MyLio>();
        // 可调字段进参数存储：算法每帧从本帧快照读，运行期改了下一帧就生效
        cfg::applyToParameters(config, system.runtime().params());
        system.init();

        std::cerr << "注意: MyLio 是占位实现（没有配准），下面的精度数字只说明链路通了，"
                     "不说明算法有效。\n";
        const std::size_t packets = system.runOffline();

        // 先终结再下结论：关闭时的排空也可能改变结局。终结之后结局固定、系统只读，
        // 模块与轨迹仍然可读
        system.shutdown();
        const auto& result = system.result();
        const auto& outcome = result.outcome;
        if (recorder) {
            // 收尾记录写不进去就抛：半个文件不能当成录完了
            recorder->close();
            std::cout << "录制: " << config.recording.output << '\n';
            if (config.recording.record_diagnostics) {
                const std::string path = config.recording.output + ".diagnostics.jsonl";
                simpleslam::record::FileDiagnosticSink sink(path);
                for (const auto& record : system.diagnostics().records()) sink.submit(record);
                sink.close();
                if (sink.failed() > 0) {
                    throw std::runtime_error("诊断落盘有 " + std::to_string(sink.failed())
                                             + " 条没写进去: " + path);
                }
                std::cout << "诊断: " << path << '\n';
            }
        }
        std::cout << "跑完: " << packets << " 包, " << odometry.processedFrames() << " 帧, "
                  << odometry.keyframeCount() << " 关键帧\n";
        for (const auto& reason : outcome.reasons()) std::cerr << "运行不完整: " << reason << '\n';
        for (const auto& note : outcome.notes()) std::cout << "说明: " << note << '\n';

        // 终结之后读固定下来的结果视图，不读已经关闭的模块
        auto estimate = result.odometry;
        eval::EvaluationReport report;
        if (!config.evaluation.output.empty()) {
            const auto format =
                eval::parseTrajectoryFormat(config.evaluation.format).value_or(
                    eval::TrajectoryFormat::kTum);
            eval::writeTrajectory(config.evaluation.output, estimate, format);
            // 报告为这条轨迹作证：独立评测拿报告当运行凭据时，按内容摘要核对
            report.trajectory_file = config.evaluation.output;
            report.trajectory_digest = eval::trajectoryFileDigest(config.evaluation.output);
            std::cout << "轨迹: " << config.evaluation.output << '\n';
        }

        if (!dataset.has_ground_truth && !config.evaluation.groundtruth.empty()) {
            dataset.ground_truth = eval::readTrajectory(
                config.evaluation.groundtruth,
                eval::detectTrajectoryFormat(config.evaluation.groundtruth));
            dataset.has_ground_truth = true;
        }

        report.dataset = config.name;
        report.config_hash = cfg::configHash(config);
        report.input_complete = outcome.inputComplete();
        report.completeness =
            outcome.complete() ? eval::RunCompleteness::kComplete : eval::RunCompleteness::kIncomplete;
        report.completeness_source = "本次运行";
        report.run_problems = outcome.reasons();
        report.notes = descriptor.notes;
        for (const auto& note : outcome.notes()) report.notes.push_back(note);
        // 目标窗口：这次运行应当产出位姿的范围。协议里记下它，任务完成度按它算
        const auto target = descriptor.evaluation_window;
        const bool has_target = target.end >= target.begin && target.end.count() > 0;
        if (has_target) {
            report.protocol.window_begin_s = std::chrono::duration<double>(target.begin).count();
            report.protocol.window_end_s = std::chrono::duration<double>(target.end).count();
        }
        report.effective = effectiveInputs(config, *system.resources().calibrationSnapshot(),
                                           resolved, descriptor, odometry.algorithm());
        const std::string effective_hash = cfg::effectiveHash(report.effective);
        report.effective["effective_hash"] = effective_hash;
        const bool evaluation_requested =
            !config.evaluation.baselines.empty() || !config.evaluation.report.empty();

        if (!dataset.has_ground_truth) {
            if (!evaluation_requested) {
                std::cout << "仅运行模式：没有真值，也没有要求评测\n";
                return 0;
            }
            // 要求了评测却没有真值：这是"没有结论"，不是"评测通过"
            report.verdict.outcome = eval::BaselineOutcome::kInvalid;
            report.verdict.detail = "没有真值，无从评测";
            if (!config.evaluation.report.empty()) eval::writeReport(config.evaluation.report, report);
            std::cerr << "评测结论: invalid 没有真值，无从评测\n";
            return eval::exitCodeFor(report.verdict);
        }

        for (const auto& note : matchTrackedSensor(estimate, dataset.ground_truth,
                                                   *system.resources().calibrationSnapshot(),
                                                   descriptor, !config.calibration.file.empty(),
                                                   config.dataset.groundtruth_frame == "camera")) {
            std::cout << note << '\n';
            report.notes.push_back(note);
        }

        const bool interpolate = config.evaluation.association == "interpolate";
        eval::AteOptions ate_options;
        ate_options.alignment =
            eval::parseAlignmentKind(config.evaluation.alignment).value_or(
                eval::AlignmentKind::kSe3);
        ate_options.tolerance = app::secondsToDuration(config.evaluation.association_tolerance_s);
        ate_options.association =
            interpolate ? eval::AssociationMode::kInterpolate : eval::AssociationMode::kNearest;
        ate_options.max_interpolation_gap =
            app::secondsToDuration(config.evaluation.max_interpolation_gap_s);
        if (has_target) ate_options.target_window = target;
        const auto ate = eval::absoluteTrajectoryError(estimate, dataset.ground_truth, ate_options);

        eval::RpeOptions rpe_options;
        rpe_options.tolerance = ate_options.tolerance;
        rpe_options.association = ate_options.association;
        rpe_options.max_interpolation_gap = ate_options.max_interpolation_gap;
        if (config.evaluation.rpe_delta_s > 0.0) {
            rpe_options.delta_kind = eval::RpeDelta::kTime;
            rpe_options.delta_time = app::secondsToDuration(config.evaluation.rpe_delta_s);
            rpe_options.normalize = false;
        } else {
            rpe_options.delta_kind = eval::RpeDelta::kDistance;
            rpe_options.delta_distance_m = config.evaluation.rpe_delta_m;
            rpe_options.normalize = true;
        }
        const auto rpe = eval::relativePoseError(estimate, dataset.ground_truth, rpe_options);

        std::cout << "对齐: " << eval::toString(ate.alignment.kind) << "  关联("
                  << eval::toString(ate_options.association) << "): " << ate.pairs << " 对 ("
                  << std::fixed << std::setprecision(1) << ate.coverage() * 100.0 << "%)\n";
        if (ate.target_declared) {
            std::cout << "目标窗口完成度: " << ate.targetCoverage() * 100.0 << "%  参考样本利用: "
                      << ate.referenceCoverage() * 100.0 << "%\n";
        }
        if (ate.pairs == 0) {
            std::cout << "ATE        无结果（没有关联上的样本）\n";
        } else {
            printStats("ATE 平移", ate.translation, "m");
            printStats("ATE 旋转", ate.rotation_deg, "deg");
        }
        if (rpe.pairs == 0) {
            // 没测出来就不打印数字：零误差与"无结果"不能长得一样
            std::cout << "RPE        无结果（轨迹长度不足一个 RPE 间隔，没有可比的段）\n";
        } else {
            printStats("RPE 平移", rpe.translation, rpe.normalized ? "m/m" : "m");
            printStats("RPE 旋转", rpe.rotation_deg, rpe.normalized ? "deg/m" : "deg");
        }

        report.alignment = eval::toString(ate.alignment.kind);
        report.metrics = eval::toMetricMap(ate, rpe);
        // 评测协议随结果保存：同一份配置换个容差或 RPE 间隔，结论不可直接互比
        report.protocol.alignment = eval::toString(ate.alignment.kind);
        report.protocol.alignment_requested = config.evaluation.alignment;
        report.protocol.association = eval::toString(ate_options.association);
        report.protocol.association_tolerance_s = config.evaluation.association_tolerance_s;
        report.protocol.max_interpolation_gap_s =
            interpolate ? config.evaluation.max_interpolation_gap_s : 0.0;
        report.protocol.rpe_delta_kind =
            rpe.delta_kind == eval::RpeDelta::kTime ? "time" : "distance";
        report.protocol.rpe_delta = rpe.delta_kind == eval::RpeDelta::kTime
                                        ? config.evaluation.rpe_delta_s
                                        : config.evaluation.rpe_delta_m;
        report.protocol.rpe_normalized = rpe.normalized;

        std::vector<eval::DatasetBaseline> baselines;
        if (!config.evaluation.baselines.empty()) {
            baselines = eval::loadBaselines(config.evaluation.baselines);
        }
        eval::GateRequest request;
        request.metrics = report.metrics;
        request.dataset = config.name;
        // 基线绑定的是实际用了什么：有效身份，而不只是配置文件的哈希
        request.config_hash = effective_hash;
        request.protocol = report.protocol;
        request.completeness = report.completeness;
        request.allow_incomplete = config.evaluation.allow_incomplete;
        request.baselines = config.evaluation.baselines.empty() ? nullptr : &baselines;
        report.verdict = eval::gate(request);

        if (!config.evaluation.report.empty()) {
            eval::writeReport(config.evaluation.report, report);
            std::cout << "报告: " << config.evaluation.report << '\n';
        }
        if (!report.verdict.passed()) {
            std::cerr << "评测结论: " << eval::toString(report.verdict.outcome) << ' '
                      << report.verdict.detail << '\n';
        }
        return eval::exitCodeFor(report.verdict);
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << '\n';
        return 1;
    }
}
