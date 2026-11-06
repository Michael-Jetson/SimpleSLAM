// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include "archive.hpp"
#include "sequences.hpp"

#include <SimpleSLAM/app/bindings.hpp>
#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/run.hpp>
#include <SimpleSLAM/app/task.hpp>
#include <SimpleSLAM/config/calibration_io.hpp>
#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/eval/association.hpp>
#include <SimpleSLAM/eval/baseline.hpp>
#include <SimpleSLAM/eval/metrics.hpp>
#include <SimpleSLAM/eval/trajectory_io.hpp>
#include <SimpleSLAM/modules/slam_system.hpp>
#include <SimpleSLAM/record/diagnostic_file_sink.hpp>
#include <SimpleSLAM/record/writer.hpp>

#include <charconv>
#include <chrono>
#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#ifndef SIMPLESLAM_VERSION
#define SIMPLESLAM_VERSION "unknown"
#endif

namespace simpleslam::app {

namespace {

namespace fs = std::filesystem;

/// 档案里由执行器写的文件。`--overwrite` 只删这些，目录里别的文件不动。
constexpr const char* kArchiveFiles[] = {"task.yaml", "identity.yaml", "outcome.yaml",
                                         "report.yaml", "recording.ssrec",
                                         "recording.ssrec.diagnostics.jsonl", "trajectory"};

/// 最终生效的打包策略，以及它的出处。推荐值不能悄悄压过用户写的配置：
/// 那样用户看着自己填的 post_support_s，却不知道它没生效。
struct ResolvedPolicy {
    fw::PacketizerPolicy policy;
    std::string origin;
};

ResolvedPolicy resolvePolicy(const config::TaskConfig& c, const plugin::Dataset& dataset) {
    if (c.system.packetizer.source_policy == "config") {
        return {toPacketizerPolicy(c.system.packetizer), "config"};
    }
    if (dataset.recommended_policy) return {*dataset.recommended_policy, "source"};
    return {toPacketizerPolicy(c.system.packetizer), "config（来源没有推荐值）"};
}

/// 组合规则：运行之前核对，不满足就报错。
void checkCombination(const config::TaskConfig& c) {
    if (c.system.pipeline == "localization") {
        throw std::runtime_error("定位管线要等地图的保存与加载，还不支持（system.pipeline = "
                                 "localization）");
    }
    if (c.task.mode == "online") {
        throw std::runtime_error("在线运行还在施工（task.mode = online），先用离线");
    }
}

/// 打开数据：外部标定取代自带的标定，外部真值取代自带的真值。
/// 实验要有真值：在运行之前就核对，而不是跑完才发现无从评测。
plugin::Dataset openData(const ResolvedTask& task) {
    const auto& c = task.config;
    const bool external_calibration = !c.data.calibration.empty();
    const plugin::DatasetRequest request{
        .external_calibration = external_calibration,
        .post_support = c.system.packetizer.source_policy == "config"
                            ? secondsToDuration(c.system.packetizer.post_support_s)
                            : fw::Duration{0}};
    plugin::Dataset dataset = task.data.make()->open(request);
    if (external_calibration) {
        dataset.calibration = config::loadCalibrationFile(c.data.calibration);
    }
    const auto& groundtruth = c.evaluation.groundtruth;
    if (!groundtruth.empty()) {
        dataset.ground_truth =
            eval::readTrajectory(groundtruth, eval::detectTrajectoryFormat(groundtruth));
    }
    if (c.task.purpose == "experiment" && !dataset.ground_truth) {
        throw std::runtime_error("实验要有真值：数据没有自带，evaluation.groundtruth 也没给；"
                                 "只出结果把 task.purpose 改成 run");
    }
    return dataset;
}

std::string seconds(fw::Duration d) {
    std::ostringstream out;
    out << std::setprecision(9) << std::chrono::duration<double>(d).count();
    return out.str();
}

/// 本次运行实际生效、影响结论的输入。进报告，并作为基线可比性的身份。
std::map<std::string, std::string> effectiveInputs(const config::TaskConfig& c,
                                                   const fw::CalibrationData& calibration,
                                                   const ResolvedPolicy& resolved,
                                                   const fw::SourceDescriptor& descriptor,
                                                   const plugin::OdometryBase& odometry) {
    std::map<std::string, std::string> effective;
    effective["config_hash"] = config::configHash(c);
    effective["system_hash"] = config::systemHash(c);
    effective["calibration_hash"] = config::calibrationHash(calibration);
    effective["packetizer.origin"] = resolved.origin;
    effective["packetizer.imu_required"] =
        resolved.policy.imu == fw::ImuRequirement::kRequired ? "true" : "false";
    effective["packetizer.post_support_s"] = seconds(resolved.policy.post_support);
    effective["packetizer.pre_support_s"] = seconds(resolved.policy.pre_support);
    effective["packetizer.max_imu_gap_s"] = seconds(resolved.policy.max_imu_gap);
    effective["source.name"] = descriptor.name;
    effective["source.imu_rate_hz"] = std::format("{}", descriptor.imu_rate_hz);
    effective["source.timing_repair"] =
        descriptor.timing_repair.empty() ? "none" : descriptor.timing_repair;
    std::string repaired;
    for (const auto frame : descriptor.repaired_frames) {
        repaired += (repaired.empty() ? "" : ",") + std::to_string(frame);
    }
    effective["source.repaired_frames"] = repaired;
    effective["algorithm.name"] = odometry.name();
    effective["algorithm.support_gap_policy"] = plugin::toString(odometry.supportGapPolicy());
    const auto limit = odometry.maxImuGap();
    effective["algorithm.max_imu_gap_s"] = limit ? seconds(*limit) : "none";
    return effective;
}

/// 让估计与真值跟踪同一个传感器。左乘对齐消不掉右侧外参，所以换算是显式的一步，
/// 用的是本次生效的外参。
std::vector<std::string> matchTrackedSensor(fw::Trajectory& estimate,
                                            fw::Trajectory& ground_truth,
                                            const fw::CalibrationData& calibration,
                                            const fw::SourceDescriptor& descriptor,
                                            bool external_calibration,
                                            bool compare_at_ground_truth) {
    std::vector<std::string> notes;
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

void printStats(std::ostream& out, const char* title, const eval::Statistics& stats,
                const char* unit) {
    out << std::left << std::setw(12) << title << std::right << std::fixed << std::setprecision(4)
        << " rmse=" << std::setw(10) << stats.rmse << " mean=" << std::setw(10) << stats.mean
        << " max=" << std::setw(10) << stats.max << "  [" << unit << "] n=" << stats.count
        << '\n';
}

/// 一次运行的事实：装配之后、终结之前读出来的东西都在这里，写档案与评测都用它。
struct RunFacts {
    std::size_t packets = 0;
    std::uint64_t frames = 0;
    std::uint64_t keyframes = 0;
    fw::SourceDescriptor descriptor;
    ResolvedPolicy policy;
    std::map<std::string, std::string> effective;
};

void writeIdentity(const fs::path& dir, const ResolvedTask& task, const RunFacts& facts) {
    using archive::quoted;
    const auto& e = facts.effective;
    archive::writeText(
        dir / "identity.yaml",
        std::format("software_version: {}\ntask_hash: {}\nsystem_hash: {}\n"
                    "calibration_hash: {}\neffective_hash: {}\nseed: {}\n"
                    "task_file: {}\nsystem_file: {}\noverrides: {}\n",
                    quoted(SIMPLESLAM_VERSION), quoted(e.at("config_hash")),
                    quoted(e.at("system_hash")), quoted(e.at("calibration_hash")),
                    quoted(e.at("effective_hash")), task.config.task.seed,
                    quoted(task.source.task_file), quoted(task.source.system_file),
                    archive::quotedList(task.source.overrides)));
}

fs::path writeTrajectories(const fs::path& dir, const modules::RunResult& result,
                           const config::TaskConfig& c, std::ostream& out) {
    const auto format =
        eval::parseTrajectoryFormat(c.evaluation.format).value_or(eval::TrajectoryFormat::kTum);
    const std::string extension = format == eval::TrajectoryFormat::kKitti ? ".txt" : ".tum";
    const std::pair<const char*, const fw::Trajectory*> trajectories[] = {
        {"odometry", &result.odometry},
        {"online", &result.online},
        {"keyframes", &result.keyframes}};
    for (const auto& [name, trajectory] : trajectories) {
        eval::writeTrajectory((dir / "trajectory" / (std::string(name) + extension)).string(),
                              *trajectory, format);
    }
    out << "轨迹: " << (dir / "trajectory").string() << '\n';
    return dir / "trajectory" / ("odometry" + extension);
}

/// 实验：对真值评测，写报告，过基线门禁。返回退出码。
int evaluate(const ResolvedTask& task, const fs::path& dir, const fs::path& trajectory_file,
             plugin::Dataset& dataset, fw::Trajectory estimate,
             const modules::RunOutcome& outcome, RunFacts& facts,
             const fw::CalibrationData& calibration, std::ostream& out, std::ostream& err) {
    const auto& c = task.config;
    const auto& ec = c.evaluation;
    eval::EvaluationReport report;
    report.trajectory_file = trajectory_file.string();
    report.trajectory_digest = eval::trajectoryFileDigest(trajectory_file.string());
    report.dataset = c.task.name;
    report.config_hash = config::configHash(c);
    report.input_complete = outcome.inputComplete();
    report.completeness =
        outcome.complete() ? eval::RunCompleteness::kComplete : eval::RunCompleteness::kIncomplete;
    report.completeness_source = "本次运行";
    report.run_problems = outcome.reasons();
    report.notes = facts.descriptor.notes;
    for (const auto& note : outcome.notes()) report.notes.push_back(note);
    const auto target = facts.descriptor.evaluation_window;
    const bool has_target = target.end >= target.begin && target.end.count() > 0;
    if (has_target) {
        report.protocol.window_begin_s = std::chrono::duration<double>(target.begin).count();
        report.protocol.window_end_s = std::chrono::duration<double>(target.end).count();
    }
    report.effective = facts.effective;

    const fs::path report_path = dir / "report.yaml";
    if (!dataset.ground_truth) throw std::logic_error("实验的真值应当在运行之前核对过");
    fw::Trajectory& ground_truth = *dataset.ground_truth;
    for (const auto& note :
         matchTrackedSensor(estimate, ground_truth, calibration, facts.descriptor,
                            !c.data.calibration.empty(), dataset.evaluate_at_ground_truth_sensor)) {
        out << note << '\n';
        report.notes.push_back(note);
    }

    const bool interpolate = ec.association == "interpolate";
    eval::AteOptions ate_options;
    ate_options.alignment =
        eval::parseAlignmentKind(ec.alignment).value_or(eval::AlignmentKind::kSe3);
    ate_options.tolerance = secondsToDuration(ec.association_tolerance_s);
    ate_options.association =
        interpolate ? eval::AssociationMode::kInterpolate : eval::AssociationMode::kNearest;
    ate_options.max_interpolation_gap = secondsToDuration(ec.max_interpolation_gap_s);
    if (has_target) ate_options.target_window = target;
    const auto ate = eval::absoluteTrajectoryError(estimate, ground_truth, ate_options);
    eval::RpeOptions rpe_options;
    rpe_options.tolerance = ate_options.tolerance;
    rpe_options.association = ate_options.association;
    rpe_options.max_interpolation_gap = ate_options.max_interpolation_gap;
    if (ec.rpe_delta_s > 0.0) {
        rpe_options.delta_kind = eval::RpeDelta::kTime;
        rpe_options.delta_time = secondsToDuration(ec.rpe_delta_s);
        rpe_options.normalize = false;
    } else {
        rpe_options.delta_kind = eval::RpeDelta::kDistance;
        rpe_options.delta_distance_m = ec.rpe_delta_m;
        rpe_options.normalize = true;
    }
    const auto rpe = eval::relativePoseError(estimate, ground_truth, rpe_options);

    out << "对齐: " << eval::toString(ate.alignment.kind) << "  关联("
        << eval::toString(ate_options.association) << "): " << ate.pairs << " 对 (" << std::fixed
        << std::setprecision(1) << ate.coverage() * 100.0 << "%)\n";
    if (ate.target_declared) {
        out << "目标窗口完成度: " << ate.targetCoverage() * 100.0 << "%  参考样本利用: "
            << ate.referenceCoverage() * 100.0 << "%\n";
    }
    if (ate.pairs == 0) {
        out << "ATE        无结果（没有关联上的样本）\n";
    } else {
        printStats(out, "ATE 平移", ate.translation, "m");
        printStats(out, "ATE 旋转", ate.rotation_deg, "deg");
    }
    if (rpe.pairs == 0) {
        // 没测出来就不打印数字：零误差与"无结果"不能长得一样
        out << "RPE        无结果（轨迹长度不足一个 RPE 间隔，没有可比的段）\n";
    } else {
        printStats(out, "RPE 平移", rpe.translation, rpe.normalized ? "m/m" : "m");
        printStats(out, "RPE 旋转", rpe.rotation_deg, rpe.normalized ? "deg/m" : "deg");
    }

    report.alignment = eval::toString(ate.alignment.kind);
    report.metrics = eval::toMetricMap(ate, rpe);
    // 评测协议随结果保存：同一份配置换个容差或 RPE 间隔，结论不可直接互比
    report.protocol.alignment = eval::toString(ate.alignment.kind);
    report.protocol.alignment_requested = ec.alignment;
    report.protocol.association = eval::toString(ate_options.association);
    report.protocol.association_tolerance_s = ec.association_tolerance_s;
    report.protocol.max_interpolation_gap_s = interpolate ? ec.max_interpolation_gap_s : 0.0;
    report.protocol.rpe_delta_kind = rpe.delta_kind == eval::RpeDelta::kTime ? "time" : "distance";
    report.protocol.rpe_delta =
        rpe.delta_kind == eval::RpeDelta::kTime ? ec.rpe_delta_s : ec.rpe_delta_m;
    report.protocol.rpe_normalized = rpe.normalized;

    std::vector<eval::DatasetBaseline> baselines;
    if (!ec.baselines.empty()) baselines = eval::loadBaselines(ec.baselines);
    eval::GateRequest request;
    request.metrics = report.metrics;
    request.dataset = c.task.name;
    // 基线绑定的是实际用了什么：有效身份，而不只是配置文件的哈希
    request.config_hash = facts.effective.at("effective_hash");
    request.protocol = report.protocol;
    request.completeness = report.completeness;
    request.allow_incomplete = ec.allow_incomplete;
    request.baselines = ec.baselines.empty() ? nullptr : &baselines;
    report.verdict = eval::gate(request);

    eval::writeReport(report_path.string(), report);
    out << "报告: " << report_path.string() << '\n';
    if (!report.verdict.passed()) {
        err << "评测结论: " << eval::toString(report.verdict.outcome) << ' '
            << report.verdict.detail << '\n';
    }
    return eval::exitCodeFor(report.verdict);
}

int usage(std::ostream& err) {
    err << R"(用法:
  <任务.yaml> [路径=值 …] [--overwrite] [--jobs=N]
                                          跑任务（也可以写成 run <任务.yaml> …）
  check <任务.yaml> [路径=值 …]           校验，插件节点由目录还原
  dump  <任务.yaml> [路径=值 …]           还原后的完整任务
  hash  <任务.yaml> [路径=值 …]           任务哈希
  schema                                  配置 schema，含目录里每个实现的字段
  default                                 全部默认值

覆盖写成 路径=值，例如 system.modules.odometry.plugin.map.voxel_size_m=0.8
多序列（data.sequences）时每个序列一个子进程，--jobs=N 同时跑 N 个（默认 1）
)";
    return 2;
}

}  // namespace

int runTask(const RunOptions& options, const config::Catalogs& catalogs, std::ostream& out,
            std::ostream& err) {
    try {
        const ResolvedTask task = loadTask(options.task_file, options.overrides, catalogs);
        const auto& c = task.config;
        out << "任务: " << options.task_file << "  任务哈希: " << config::configHash(c)
            << "  系统哈希: " << config::systemHash(c) << '\n';
        // 不支持的组合、占用的档案目录、没有真值的实验，都在动档案目录之前报错
        checkCombination(c);
        if (!c.data.sequences.empty()) return runSequences(task, options, out);
        const fs::path dir = archive::directory(c);
        archive::refuseOccupied(dir, options.overwrite);
        plugin::Dataset dataset = openData(task);
        archive::prepare(dir, kArchiveFiles);
        fs::create_directories(dir / "trajectory");
        config::writeConfigFile((dir / "task.yaml").string(), c);

        RunFacts facts;
        facts.descriptor = dataset.source->descriptor();
        facts.policy = resolvePolicy(c, dataset);
        out << "数据: " << dataset.source->name() << "  帧数: "
            << (dataset.frames ? std::to_string(*dataset.frames) : std::string("未知")) << '\n'
            << "标定: " << dataset.calibration.device << "  哈希: "
            << config::calibrationHash(dataset.calibration) << '\n'
            << "打包策略: " << facts.policy.origin << "  post_support=" << std::fixed
            << std::setprecision(3)
            << std::chrono::duration<double>(facts.policy.policy.post_support).count() << "s"
            << "  惯性实测: " << std::setprecision(1) << facts.descriptor.imu_rate_hz << "Hz\n";

        std::shared_ptr<record::RecordWriter> recorder;
        const fs::path recording = dir / "recording.ssrec";
        if (c.outputs.record) {
            record::RecordMetadata metadata;
            metadata.name = c.task.name;
            metadata.config_hash = config::configHash(c);
            metadata.seed = c.task.seed;
            recorder = std::make_shared<record::RecordWriter>(recording.string(), metadata);
            dataset.source =
                std::make_unique<record::RecordingSource>(std::move(dataset.source), recorder);
        }

        // ── 装配：管线按工作方式分 ──
        modules::SlamSystem system(toSystemOptions(c));
        out << "标定版本: " << system.setCalibration(dataset.calibration) << '\n';
        system.setSource(std::move(dataset.source), facts.policy.policy);
        auto& odometry = system.setOdometry(task.odometry.make());
        if (c.system.pipeline == "mapping") {
            system.setPlaceRecognizer(task.place_recognizer.make());
            system.setGraphOptimizer(task.graph_optimizer.make(),
                                     toEdgePolicy(c.system.modules.graph_optimizer));
        }
        applyToParameters(c, system.runtime().params());
        system.init();

        // ── 驱动与终结 ──
        facts.packets = system.runOffline();
        system.shutdown();
        const auto& result = system.result();
        const auto& outcome = result.outcome;
        facts.frames = odometry.processedFrames();
        facts.keyframes = odometry.keyframeCount();
        const auto calibration = *system.resources().calibrationSnapshot();
        facts.effective = effectiveInputs(c, calibration, facts.policy, facts.descriptor,
                                          odometry.algorithm());
        facts.effective["effective_hash"] = config::effectiveHash(facts.effective);

        if (recorder) {
            recorder->close();
            out << "录制: " << recording.string() << '\n';
            if (c.outputs.record_diagnostics) {
                const std::string path = recording.string() + ".diagnostics.jsonl";
                record::FileDiagnosticSink sink(path);
                for (const auto& record : system.diagnostics().records()) sink.submit(record);
                sink.close();
                if (sink.failed() > 0) {
                    throw std::runtime_error(
                        std::format("诊断落盘有 {} 条没写进去: {}", sink.failed(), path));
                }
                out << "诊断: " << path << '\n';
            }
        }
        out << "跑完: " << facts.packets << " 包, " << facts.frames << " 帧, " << facts.keyframes
            << " 关键帧\n";
        for (const auto& reason : outcome.reasons()) err << "运行不完整: " << reason << '\n';
        for (const auto& note : outcome.notes()) out << "说明: " << note << '\n';

        // ── 任务档案 ──
        const fs::path trajectory_file = writeTrajectories(dir, result, c, out);
        writeIdentity(dir, task, facts);
        archive::writeOutcome(dir, outcome, {facts.packets, facts.frames, facts.keyframes});
        out << "档案: " << dir.string() << '\n';

        if (c.task.purpose == "run") return 0;
        return evaluate(task, dir, trajectory_file, dataset, result.odometry, outcome, facts,
                        calibration, out, err);
    } catch (const std::exception& e) {
        err << "错误: " << e.what() << '\n';
        return 1;
    }
}

int configCommand(const std::vector<std::string>& args, const config::Catalogs& catalogs,
                  std::ostream& out, std::ostream& err) {
    if (args.empty()) return usage(err);
    const std::string& command = args.front();
    if (command == "schema") {
        out << config::schemaJson(slotInterfaces(), catalogs.schemas());
        return 0;
    }
    if (command == "default") {
        out << config::dumpConfig(config::TaskConfig{});
        return 0;
    }
    if (command != "check" && command != "dump" && command != "hash") return -1;
    if (args.size() < 2) return usage(err);
    try {
        const auto task =
            loadTask(args[1], std::vector<std::string>(args.begin() + 2, args.end()), catalogs);
        if (command == "check") {
            out << "任务有效: " << args[1] << "\n任务哈希: " << config::configHash(task.config)
                << "\n系统哈希: " << config::systemHash(task.config) << '\n';
        } else if (command == "dump") {
            out << config::dumpConfig(task.config);
        } else {
            out << config::configHash(task.config) << '\n';
        }
        return 0;
    } catch (const std::exception& e) {
        err << "错误: " << e.what() << '\n';
        return 1;
    }
}

int run(int argc, char** argv, const config::Catalogs& catalogs) {
    std::vector<std::string> args(argv + 1, argv + argc);
    if (args.empty()) return usage(std::cerr);
    if (const int code = configCommand(args, catalogs, std::cout, std::cerr); code != -1) {
        return code;
    }
    if (args.front() == "run") args.erase(args.begin());
    RunOptions options;
    for (const auto& arg : args) {
        if (arg == "--overwrite") {
            options.overwrite = true;
        } else if (arg.starts_with("--jobs=")) {
            const std::string_view text = std::string_view(arg).substr(7);
            const auto [end, error] =
                std::from_chars(text.data(), text.data() + text.size(), options.jobs);
            if (error != std::errc() || end != text.data() + text.size() || options.jobs == 0) {
                std::cerr << "--jobs 要写成正整数，得到 " << text << '\n';
                return usage(std::cerr);
            }
        } else if (arg.find('=') != std::string::npos) {
            options.overrides.push_back(arg);
        } else if (options.task_file.empty()) {
            options.task_file = arg;
        } else {
            std::cerr << "多余的参数: " << arg << '\n';
            return usage(std::cerr);
        }
    }
    if (options.task_file.empty()) return usage(std::cerr);
    return runTask(options, catalogs, std::cout, std::cerr);
}

}  // namespace simpleslam::app
