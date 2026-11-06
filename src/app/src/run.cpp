// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include "evaluation.hpp"
#include "line_stream.hpp"
#include "run_dir.hpp"
#include "sequences.hpp"

#include <SimpleSLAM/app/bindings.hpp>
#include <SimpleSLAM/app/catalogs.hpp>
#include <SimpleSLAM/app/run.hpp>
#include <SimpleSLAM/app/task.hpp>
#include <SimpleSLAM/config/calibration_io.hpp>
#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/eval/association.hpp>
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

/// 任务记录里由执行器写的文件。`--overwrite` 只删这些，目录里别的文件不动。
constexpr const char* kRunDirFiles[] = {"task.yaml",        "identity.yaml",
                                        "outcome.yaml",     "calibration.yaml",
                                        "result.json",      "run.log",
                                        "recording.ssrec",  "recording.ssrec.diagnostics.jsonl",
                                        "queues.txt",       "trajectory",
                                        "evaluation"};

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
}

/// 打开数据：外部标定取代自带的标定，外部真值取代自带的真值。
/// 所选评测函数要真值时，在运行之前就核对，而不是跑完才发现无从评测。
plugin::Dataset openData(const ResolvedTask& task, const logging::Logger& runner) {
    const auto& c = task.config;
    const bool external_calibration = !c.data.calibration.empty();
    const plugin::DatasetRequest request{
        .external_calibration = external_calibration,
        .post_support = c.system.packetizer.source_policy == "config"
                            ? secondsToDuration(c.system.packetizer.post_support_s)
                            : fw::Duration{0}};
    plugin::Dataset dataset = task.data.make(runner, "data.source")->open(request);
    if (external_calibration) {
        dataset.calibration = config::loadCalibrationFile(c.data.calibration);
    }
    const auto& groundtruth = c.evaluation.groundtruth;
    if (!groundtruth.empty()) {
        dataset.ground_truth =
            eval::readTrajectory(groundtruth, eval::detectTrajectoryFormat(groundtruth));
    }
    if (!dataset.ground_truth && needsGroundTruth(task)) {
        throw std::runtime_error("所选的评测函数要真值：数据没有自带，evaluation.groundtruth "
                                 "也没给；只跑不评测就去掉 evaluation.evaluators");
    }
    return dataset;
}

std::string seconds(fw::Duration d) {
    std::ostringstream out;
    out << std::setprecision(9) << std::chrono::duration<double>(d).count();
    return out.str();
}

/// 本次运行实际生效、影响结论的输入。进 identity.yaml，并作为基线可比性的身份。
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
    // 目标窗口进有效输入：评测函数算任务完成度要它，事后补评从任务记录里读
    const auto& window = descriptor.evaluation_window;
    if (window.end >= window.begin && window.end.count() > 0) {
        effective["source.evaluation_window_begin_ns"] = std::to_string(window.begin.count());
        effective["source.evaluation_window_end_ns"] = std::to_string(window.end.count());
    }
    return effective;
}

/// 写进任务记录的真值：换算到比较用的传感器上。左乘对齐消不掉右侧外参，所以换算是显式的
/// 一步，用的是本次生效的外参；估计（跟踪 body）在评测时按任务记录里的生效标定换算过去。
fw::Trajectory comparisonGroundTruth(fw::Trajectory ground_truth,
                                     const fw::CalibrationData& calibration,
                                     const fw::SourceDescriptor& descriptor,
                                     bool external_calibration, bool compare_at_ground_truth,
                                     std::vector<std::string>& notes) {
    if (ground_truth.tracked().empty()) return ground_truth;
    if (external_calibration && ground_truth.tracked() == "body" && !descriptor.body_sensor.empty()
        && calibration.T_body_sensor(descriptor.body_sensor)) {
        ground_truth.setTracked(descriptor.body_sensor);
    }
    const std::string target = compare_at_ground_truth ? ground_truth.tracked() : "body";
    if (ground_truth.tracked() != target) {
        const std::string from = ground_truth.tracked();
        const auto T_body_from = calibration.T_body_sensor(from);
        if (!T_body_from) {
            throw std::runtime_error("比较要用 " + from + " 的外参，但生效的标定里没有它");
        }
        ground_truth = eval::rightMultiplied(ground_truth, T_body_from->inverse(), target);
        notes.push_back("真值从 " + from + " 按生效外参换算到 " + target);
    }
    return ground_truth;
}


/// 一次运行的事实：装配之后、终结之前读出来的东西都在这里，写任务记录时用。
struct RunFacts {
    std::size_t packets = 0;
    std::uint64_t frames = 0;
    std::uint64_t keyframes = 0;
    fw::SourceDescriptor descriptor;
    ResolvedPolicy policy;
    std::map<std::string, std::string> effective;
};

void writeIdentity(const fs::path& dir, const ResolvedTask& task, const RunFacts& facts) {
    using run_dir::quoted;
    const auto& e = facts.effective;
    std::string effective;
    for (const auto& [key, value] : e) {
        effective += "  " + quoted(key) + ": " + quoted(value) + "\n";
    }
    run_dir::writeText(
        dir / "identity.yaml",
        std::format("software_version: {}\ntask_hash: {}\nsystem_hash: {}\n"
                    "calibration_hash: {}\neffective_hash: {}\nseed: {}\n"
                    "task_file: {}\nsystem_file: {}\noverrides: {}\neffective:\n{}",
                    quoted(SIMPLESLAM_VERSION), quoted(e.at("config_hash")),
                    quoted(e.at("system_hash")), quoted(e.at("calibration_hash")),
                    quoted(e.at("effective_hash")), task.config.task.seed,
                    quoted(task.source.task_file), quoted(task.source.system_file),
                    run_dir::quotedList(task.source.overrides), effective));
}

/// 三条轨迹总写 TUM（带语义，评测读它）；要的话另写一份 KITTI 格式给官方评测程序。
void writeTrajectories(const fs::path& dir, const modules::RunResult& result,
                       const config::TaskConfig& c, std::ostream& out) {
    const std::pair<const char*, const fw::Trajectory*> trajectories[] = {
        {"odometry", &result.odometry},
        {"online", &result.online},
        {"keyframes", &result.keyframes}};
    for (const auto& [name, trajectory] : trajectories) {
        const fs::path base = dir / "trajectory" / name;
        eval::writeTrajectory(base.string() + ".tum", *trajectory, eval::TrajectoryFormat::kTum);
        if (c.outputs.kitti_trajectories) {
            eval::writeTrajectory(base.string() + ".txt", *trajectory,
                                  eval::TrajectoryFormat::kKitti);
        }
    }
    out << "轨迹: " << (dir / "trajectory").string() << '\n';
}

int usage(std::ostream& err) {
    err << R"(用法:
  <任务.yaml> [路径=值 …] [--overwrite] [--jobs=N]
                                          跑任务（也可以写成 run <任务.yaml> …）
  check <任务.yaml> [路径=值 …]           校验，插件节点由目录还原
  dump  <任务.yaml> [路径=值 …]           还原后的完整任务
  hash  <任务.yaml> [路径=值 …]           任务哈希
  evaluate <任务记录> [evaluation.…=值 …]  单独评测：补评或换评测函数、协议重评，不重跑
  schema                                  配置 schema，含目录里每个实现的字段
  default                                 全部默认值

覆盖写成 路径=值，例如 system.modules.odometry.plugin.map.voxel_size_m=0.8
多序列（data.sequences）时每个序列一个子进程，--jobs=N 同时跑 N 个（默认 1）
)";
    return 2;
}

/// 门禁的结论：没过时写到 err，结论文件的位置写到 out。
void reportGate(const EvaluationOutcome& evaluated, const fs::path& dir, std::ostream& out,
                std::ostream& err) {
    const auto& verdict = evaluated.verdict;
    if (verdict.passed()) {
        out << "门禁: pass" << (verdict.detail.empty() ? "" : "（" + verdict.detail + "）") << '\n';
    } else {
        err << "门禁: " << eval::toString(verdict.outcome) << ' ' << verdict.detail << '\n';
    }
    out << "结论: " << (dir / "result.json").string() << '\n';
}

/// 单个序列的任务。日志组件是系统自带的，所以系统最先建：读数据阶段的日志也进任务记录。
/// 任务记录目录建好之前 run.log 还没接上，那之前的记录先存着；任务在动任务记录目录之前出错时，
/// 它们只出现在终端上。
int runSingle(const ResolvedTask& task, const RunOptions& options, std::ostream& out,
              std::ostream& err) {
    const auto& c = task.config;
    modules::SlamSystemOptions system_options = toSystemOptions(c);
    system_options.log.out = &out;
    system_options.log.err = &err;
    system_options.log.defer_file = true;
    modules::SlamSystem system(system_options);
    const logging::Logger runner = system.logRouter().root("task", "", true);
    LineStream info(runner, logging::Level::kInfo);
    LineStream warn(runner, logging::Level::kWarn);
    try {
        info << "任务: " << options.task_file << "  任务哈希: " << config::configHash(c)
             << "  系统哈希: " << config::systemHash(c) << '\n';
        for (const auto& note : task.notes) warn << "注意: " << note << '\n';
        // 不支持的组合、占用的任务记录目录、要真值却没有真值，都在动任务记录目录之前报错
        checkCombination(c);
        const fs::path dir = run_dir::directory(c);
        run_dir::refuseOccupied(dir, options.overwrite);
        plugin::Dataset dataset = openData(task, runner);
        if (task.evaluators.empty() && dataset.ground_truth) {
            // v2 的任务默认评测，v3 没列评测函数就不评：只靠默认值的旧任务认不出来，提示一句
            info << "提示: 数据有真值，任务里没有评测函数，只跑不评测；要评测就在 "
                    "evaluation.evaluators 里列出，如 ate: {type: ate}\n";
        }
        run_dir::prepare(dir, kRunDirFiles);
        fs::create_directories(dir / "trajectory");
        system.logRouter().openFile(dir / "run.log");
        config::writeConfigFile((dir / "task.yaml").string(), c);

        RunFacts facts;
        facts.descriptor = dataset.source->descriptor();
        facts.policy = resolvePolicy(c, dataset);
        info << "数据: " << dataset.source->name() << "  帧数: "
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

        // ── 装配：管线按工作方式分。经槽构造的插件挂到各自模块的日志下面 ──
        info << "标定版本: " << system.setCalibration(dataset.calibration) << '\n';
        system.setSource(std::move(dataset.source), facts.policy.policy);
        auto& router = system.logRouter();
        auto& odometry =
            system.setOdometry(task.odometry.make(router.root(modules::kOdometryModule), "plugin"));
        if (c.system.pipeline == "mapping") {
            system.setPlaceRecognizer(
                task.place_recognizer.make(router.root(modules::kPlaceRecognizerModule), "plugin"));
            system.setGraphOptimizer(
                task.graph_optimizer.make(router.root(modules::kGraphOptimizerModule), "plugin"),
                toEdgePolicy(c.system.modules.graph_optimizer));
        }
        applyToParameters(c, system.runtime().params());
        system.init();

        // ── 驱动与终结：离线逐步跑到读完；在线一个模块一个线程，按数据时间的节奏回放 ──
        const bool online = c.task.mode == "online";
        info << "运行方式: " << (online ? "在线，按数据时间的节奏回放" : "离线") << std::endl;
        facts.packets = online ? system.runOnline() : system.runOffline();
        system.shutdown();
        if (online) {
            // 调 online_max_in_flight 要看队列峰值：统计表打出来，也进任务记录
            const std::string queues = system.runtime().statsTable();
            run_dir::writeText(dir / "queues.txt", queues);
            info << queues;
        }
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
            info << "录制: " << recording.string() << '\n';
            if (c.outputs.record_diagnostics) {
                const std::string path = recording.string() + ".diagnostics.jsonl";
                record::FileDiagnosticSink sink(path);
                for (const auto& record : system.diagnostics().records()) sink.submit(record);
                sink.close();
                if (sink.failed() > 0) {
                    throw std::runtime_error(
                        std::format("诊断落盘有 {} 条没写进去: {}", sink.failed(), path));
                }
                info << "诊断: " << path << '\n';
            }
        }
        info << "跑完: " << facts.packets << " 包, " << facts.frames << " 帧, " << facts.keyframes
            << " 关键帧\n";
        for (const auto& reason : outcome.reasons()) warn << "运行不完整: " << reason << '\n';
        for (const auto& note : outcome.notes()) info << "说明: " << note << '\n';

        // ── 任务记录：评测只读它，所以评测要的输入（真值、生效标定）也写进来 ──
        writeTrajectories(dir, result, c, info);
        if (dataset.ground_truth) {
            std::vector<std::string> notes;
            const auto truth = comparisonGroundTruth(
                std::move(*dataset.ground_truth), calibration, facts.descriptor,
                !c.data.calibration.empty(), dataset.evaluate_at_ground_truth_sensor, notes);
            eval::writeTrajectory((dir / "trajectory" / "groundtruth.tum").string(), truth,
                                  eval::TrajectoryFormat::kTum);
            for (const auto& note : notes) info << note << '\n';
        }
        config::writeCalibrationFile((dir / "calibration.yaml").string(), calibration);
        writeIdentity(dir, task, facts);
        // 缓冲满了丢掉的 debug、info 也是结局的一部分：写下来，不静默
        router.flush();
        const std::uint64_t dropped = router.dropped();
        if (dropped > 0) warn << "日志缓冲满了，丢掉 " << dropped << " 条 debug、info\n";
        run_dir::writeOutcome(dir, outcome,
                              {.packets = facts.packets,
                               .frames = facts.frames,
                               .keyframes = facts.keyframes,
                               .log_dropped = dropped});
        info << "任务记录: " << dir.string() << '\n';

        const Completeness completeness{.state = outcome.complete()
                                                     ? eval::RunCompleteness::kComplete
                                                     : eval::RunCompleteness::kIncomplete,
                                        .source = "本次运行",
                                        .declaration = {}};
        if (task.evaluators.empty()) {  // 只跑不评测
            writeResult(dir, identityOf(task), completeness, nullptr);
            return 0;
        }
        const auto evaluated = evaluateRunDir(dir, task, completeness.state, info);
        writeResult(dir, identityOf(task), completeness, &evaluated);
        reportGate(evaluated, dir, info, warn);
        return evaluated.exit_code;
    } catch (const std::exception& e) {
        warn << "错误: " << e.what() << '\n';
        return 1;
    }
}

}  // namespace

int runTask(const RunOptions& options, const config::Catalogs& catalogs, std::ostream& out,
            std::ostream& err) {
    try {
        const ResolvedTask task = loadTask(options.task_file, options.overrides, catalogs);
        const auto& c = task.config;
        if (c.data.sequences.empty()) return runSingle(task, options, out, err);
        out << "任务: " << options.task_file << "  任务哈希: " << config::configHash(c)
            << "  系统哈希: " << config::systemHash(c) << '\n';
        for (const auto& note : task.notes) err << "注意: " << note << '\n';
        checkCombination(c);
        return runSequences(task, options, out);
    } catch (const std::exception& e) {
        err << "错误: " << e.what() << '\n';
        return 1;
    }
}

int evaluateTask(const std::string& dir_text, const std::vector<std::string>& overrides,
                 const config::Catalogs& catalogs, std::ostream& out, std::ostream& err) {
    try {
        const fs::path dir(dir_text);
        if (!fs::exists(dir / "task.yaml")) {
            throw std::runtime_error(dir_text + " 不是任务记录：里面没有 task.yaml");
        }
        // v2 写的任务记录缺评测的输入（真值、生效的标定、轨迹的语义），补齐要回头读数据、
        // 还要假设轨迹跟踪的传感器，所以不补：说清楚还能怎么办
        if (const auto major = run_dir::writtenByMajor(dir); major && *major < 3) {
            throw std::runtime_error(
                dir_text + " 是 v" + std::to_string(*major) + " 写的任务记录，缺评测要的输入"
                "（真值、生效的标定、轨迹的语义）：用现在的版本重跑这个任务（它的 task.yaml "
                "照样能用），或用 simpleslam-eval 评测它的轨迹"
                "（--run-report 指向它的 report.yaml）");
        }
        for (const auto& item : overrides) {
            if (!item.starts_with("evaluation.")) {
                throw std::runtime_error("单独评测只改评测，覆盖项要以 evaluation. 开头，得到 "
                                         + item);
            }
        }
        const ResolvedTask task = loadTask((dir / "task.yaml").string(), overrides, catalogs);
        if (!task.config.data.sequences.empty()) {
            return evaluateSequences(task, dir, overrides, catalogs, out, err);
        }
        if (task.evaluators.empty()) {
            throw std::runtime_error("没有评测函数：在 evaluation.evaluators 里列出，"
                                     "或者用覆盖项加一个，如 evaluation.evaluators.ate.type=ate");
        }
        const Completeness completeness = completenessOf(dir);
        const auto evaluated = evaluateRunDir(dir, task, completeness.state, out);
        writeResult(dir, identityOf(task), completeness, &evaluated);
        // 评测一节随这次评测更新：评测不进任务哈希，运行的身份不变
        config::writeConfigFile((dir / "task.yaml").string(), task.config);
        reportGate(evaluated, dir, out, err);
        return evaluated.exit_code;
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
        for (const auto& note : task.notes) err << "注意: " << note << '\n';
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
    if (args.front() == "evaluate") {
        if (args.size() < 2) return usage(std::cerr);
        return evaluateTask(args[1], std::vector<std::string>(args.begin() + 2, args.end()),
                            catalogs, std::cout, std::cerr);
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
