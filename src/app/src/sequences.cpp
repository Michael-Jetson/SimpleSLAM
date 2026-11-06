// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include "sequences.hpp"

#include "archive.hpp"

#include <SimpleSLAM/config/loader.hpp>
#include <SimpleSLAM/eval/baseline.hpp>

#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <map>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#ifndef SIMPLESLAM_VERSION
#define SIMPLESLAM_VERSION "unknown"
#endif

extern char** environ;  // NOLINT(readability-redundant-declaration)：posix_spawn 要传环境

namespace simpleslam::app {

namespace {

namespace fs = std::filesystem;

/// 多序列档案顶层由执行器写的文件。`--overwrite` 只删这些；各序列的子档案由子进程自己清。
constexpr const char* kSuiteFiles[] = {"task.yaml", "identity.yaml", "summary.yaml", "logs"};

/// 汇总表的列与表头。
constexpr const char* kColumns[] = {"ate.translation.rmse", "ate.rotation_deg.rmse",
                                     "rpe.translation.rmse", "rpe.rotation_deg.rmse"};
constexpr const char* kHeaders[] = {"ATE_t", "ATE_r", "RPE_t", "RPE_r"};

/// 一个序列：怎么跑，跑成什么样。
struct SequenceRun {
    std::string name;
    fs::path archive;
    fs::path log;
    std::vector<std::string> args;
    int exit_code = -1;
    std::string problem;  ///< 被信号终止、结果读不懂这类，空表示没有
    std::optional<bool> complete;
    std::optional<eval::ReportResult> report;
};

/// 支持多序列的读取器有一个字符串参数 `sequence`；任务自己再写一个，就是两处说法。
void requireSequenceField(const config::TaskConfig& c) {
    const auto& source = c.data.source;
    const auto it = source.params.find("sequence");
    if (it == source.params.end() || !std::holds_alternative<std::string>(it->second)) {
        throw std::runtime_error(std::format(
            "数据来源 {} 没有字符串参数 sequence，不支持多序列（data.sequences）", source.type));
    }
    if (!std::get<std::string>(it->second).empty()) {
        throw std::runtime_error("data.sequences 与 data.source.sequence 只能写一个："
                                 "多序列时每个序列的 sequence 由执行器给");
    }
}

fs::path currentExecutable() {
    std::error_code error;
    fs::path path = fs::read_symlink("/proc/self/exe", error);
    if (error) {
        throw std::runtime_error("找不到当前的可执行文件（/proc/self/exe）：多序列要用它为每个"
                                 "序列起一个子进程");
    }
    return path;
}

/// 每个序列的子进程命令行：同一个任务文件、同样的命令行覆盖项，再加四个覆盖项。
std::vector<SequenceRun> plan(const ResolvedTask& task, const RunOptions& options,
                              const fs::path& dir) {
    const auto& c = task.config;
    const std::string executable =
        options.executable.empty() ? currentExecutable().string() : options.executable;
    std::vector<SequenceRun> runs;
    for (const auto& sequence : c.data.sequences) {
        SequenceRun run;
        run.name = sequence;
        run.archive = dir / sequence;
        run.log = dir / "logs" / (sequence + ".log");
        run.args = {executable, options.task_file};
        run.args.insert(run.args.end(), options.overrides.begin(), options.overrides.end());
        run.args.push_back("task.name=" + archive::quoted(c.task.name + "-" + sequence));
        run.args.emplace_back("data.sequences=[]");
        run.args.push_back("data.source.sequence=" + archive::quoted(sequence));
        run.args.push_back("outputs.dir=" + archive::quoted(run.archive.string()));
        if (options.overwrite) run.args.emplace_back("--overwrite");
        runs.push_back(std::move(run));
    }
    return runs;
}

/// posix_spawn 的文件动作，析构时释放。
class FileActions {
public:
    FileActions() { check(posix_spawn_file_actions_init(&actions_), "初始化"); }
    ~FileActions() { posix_spawn_file_actions_destroy(&actions_); }
    FileActions(const FileActions&) = delete;
    FileActions& operator=(const FileActions&) = delete;

    /// 标准输出与标准错误都写进这个文件。
    void redirectOutput(const fs::path& file) {
        check(posix_spawn_file_actions_addopen(&actions_, STDOUT_FILENO, file.c_str(),
                                               O_WRONLY | O_CREAT | O_TRUNC, 0644),
              "重定向输出");
        check(posix_spawn_file_actions_adddup2(&actions_, STDOUT_FILENO, STDERR_FILENO),
              "重定向错误输出");
    }

    [[nodiscard]] const posix_spawn_file_actions_t* get() const noexcept { return &actions_; }

private:
    static void check(int error, const char* what) {
        if (error != 0) {
            throw std::runtime_error(
                std::format("子进程的文件动作（{}）失败: {}", what, std::strerror(error)));
        }
    }

    posix_spawn_file_actions_t actions_{};
};

pid_t spawn(const SequenceRun& run) {
    FileActions actions;
    actions.redirectOutput(run.log);
    std::vector<char*> argv;
    argv.reserve(run.args.size() + 1);
    // posix_spawn 的参数表类型是 char* const[]，它不改参数
    for (const auto& arg : run.args) argv.push_back(const_cast<char*>(arg.c_str()));
    argv.push_back(nullptr);
    pid_t pid = 0;
    if (const int error =
            posix_spawn(&pid, argv.front(), actions.get(), nullptr, argv.data(), environ);
        error != 0) {
        throw std::runtime_error(std::format("起不了序列 {} 的子进程（{}）: {}", run.name,
                                             run.args.front(), std::strerror(error)));
    }
    return pid;
}

void record(SequenceRun& run, int status) {
    if (WIFEXITED(status)) {
        run.exit_code = WEXITSTATUS(status);
        return;
    }
    run.exit_code = 1;
    run.problem = WIFSIGNALED(status) ? std::format("子进程被信号 {} 终止", WTERMSIG(status))
                                      : std::string("子进程异常结束");
}

/// 同时最多跑 jobs 个序列。中途出错（起不了子进程）时，先停掉已经起来的，再抛。
void runAll(std::vector<SequenceRun>& runs, std::size_t jobs, std::ostream& out) {
    std::map<pid_t, std::size_t> running;
    std::size_t next = 0;
    try {
        while (next < runs.size() || !running.empty()) {
            while (running.size() < jobs && next < runs.size()) {
                running.emplace(spawn(runs[next]), next);
                out << "序列 " << runs[next].name << " 开始，日志 " << runs[next].log.string()
                    << std::endl;
                ++next;
            }
            int status = 0;
            const pid_t pid = ::waitpid(-1, &status, 0);
            if (pid < 0) {
                if (errno == EINTR) continue;
                throw std::runtime_error(std::string("等待子进程失败: ") + std::strerror(errno));
            }
            const auto it = running.find(pid);
            if (it == running.end()) continue;  // 不是这里起的子进程
            SequenceRun& run = runs[it->second];
            running.erase(it);
            record(run, status);
            out << "序列 " << run.name << " 结束，退出码 " << run.exit_code << std::endl;
        }
    } catch (...) {
        for (const auto& [pid, index] : running) ::kill(pid, SIGTERM);
        for (const auto& [pid, index] : running) {
            int status = 0;
            ::waitpid(pid, &status, 0);
        }
        throw;
    }
}

/// 日志里最后一条"错误: "：出错的序列在汇总里写明原因，不必先翻日志。
std::string lastError(const fs::path& log) {
    std::ifstream file(log);
    std::string line;
    std::string last;
    constexpr std::string_view kPrefix = "错误: ";
    while (std::getline(file, line)) {
        if (line.starts_with(kPrefix)) last = line.substr(kPrefix.size());
    }
    return last.empty() ? "见日志" : last;
}

/// 子进程跑到了评测（或只出结果时跑完了）才读它的档案：出错退出的子进程可能留下
/// 上一次的旧文件，读了就是拿旧结果冒充这一次的。
void collect(SequenceRun& run, const std::string& purpose) {
    if (run.exit_code != 0 && run.exit_code != 3 && run.exit_code != 4 && run.exit_code != 5) {
        if (run.problem.empty()) run.problem = lastError(run.log);
        return;
    }
    try {
        run.complete = archive::readComplete(run.archive);
        const fs::path report = run.archive / "report.yaml";
        if (fs::exists(report)) run.report = eval::readReportResult(report.string());
        if (purpose == "experiment" && !run.report) {
            throw std::runtime_error("实验没有写出评测报告: " + report.string());
        }
    } catch (const std::exception& e) {
        run.problem = e.what();
        run.exit_code = 1;
    }
}

std::string status(const SequenceRun& run, const std::string& purpose) {
    if (run.report) return eval::toString(run.report->outcome);
    if (run.exit_code == 0 && purpose == "run") return "done";
    return "error";
}

/// 一个序列在汇总表里的指标。结论无效、或这一族一对都没关联上时没有值：
/// 零误差与没有结果不能长得一样。
std::map<std::string, double> values(const SequenceRun& run) {
    std::map<std::string, double> out;
    if (!run.report || run.report->outcome == eval::BaselineOutcome::kInvalid) return out;
    const auto& metrics = run.report->metrics;
    for (const std::string key : kColumns) {
        const auto pairs = metrics.find(key.substr(0, key.find('.')) + ".pairs");
        const auto value = metrics.find(key);
        if (pairs != metrics.end() && pairs->second > 0.0 && value != metrics.end()) {
            out.emplace(key, value->second);
        }
    }
    return out;
}

/// 各序列的平均。有一个序列没有这一项，这一项就不算平均：缺了的序列不能当零。
std::map<std::string, double> mean(const std::vector<SequenceRun>& runs) {
    std::map<std::string, double> out;
    for (const std::string key : kColumns) {
        double sum = 0.0;
        bool all = !runs.empty();
        for (const auto& run : runs) {
            const auto v = values(run);
            const auto it = v.find(key);
            if (it == v.end()) {
                all = false;
                break;
            }
            sum += it->second;
        }
        if (all) out.emplace(key, sum / static_cast<double>(runs.size()));
    }
    return out;
}

/// 出错 > 无效 > 不可比较 > 失败 > 通过。
int severity(int exit_code) {
    switch (exit_code) {
        case 0: return 0;
        case 3: return 1;
        case 5: return 2;
        case 4: return 3;
        default: return 4;
    }
}

int worstExitCode(const std::vector<SequenceRun>& runs) {
    int worst = 0;
    for (const auto& run : runs) {
        if (severity(run.exit_code) > severity(worst)) {
            worst = severity(run.exit_code) == 4 ? 1 : run.exit_code;
        }
    }
    return worst;
}

void emitMetrics(std::ostream& text, const std::map<std::string, double>& metrics,
                 const char* indent) {
    if (metrics.empty()) {
        text << " {}\n";
        return;
    }
    text << '\n';
    for (const auto& [key, value] : metrics) {
        text << indent << key << ": " << std::format("{}", value) << '\n';
    }
}

void writeIdentity(const fs::path& dir, const ResolvedTask& task) {
    const auto& c = task.config;
    archive::writeText(
        dir / "identity.yaml",
        std::format("software_version: {}\ntask_hash: {}\nsystem_hash: {}\nseed: {}\n"
                    "task_file: {}\nsystem_file: {}\noverrides: {}\nsequences: {}\n",
                    archive::quoted(SIMPLESLAM_VERSION), archive::quoted(config::configHash(c)),
                    archive::quoted(config::systemHash(c)), c.task.seed,
                    archive::quoted(task.source.task_file),
                    archive::quoted(task.source.system_file),
                    archive::quotedList(task.source.overrides),
                    archive::quotedList(c.data.sequences)));
}

void writeSummary(const fs::path& dir, const config::TaskConfig& c,
                  const std::vector<SequenceRun>& runs, int exit_code) {
    std::ostringstream text;
    text << "task: " << archive::quoted(c.task.name) << "\nexit_code: " << exit_code
         << "\nsequences:\n";
    for (const auto& run : runs) {
        text << "  - name: " << archive::quoted(run.name) << '\n'
             << "    archive: " << archive::quoted(run.archive.string()) << '\n'
             << "    log: " << archive::quoted(run.log.string()) << '\n'
             << "    exit_code: " << run.exit_code << '\n'
             << "    status: " << archive::quoted(status(run, c.task.purpose)) << '\n';
        if (run.complete) text << "    complete: " << (*run.complete ? "true" : "false") << '\n';
        if (!run.problem.empty()) text << "    problem: " << archive::quoted(run.problem) << '\n';
        text << "    metrics:";
        emitMetrics(text, values(run), "      ");
    }
    text << "mean:";
    emitMetrics(text, mean(runs), "  ");
    archive::writeText(dir / "summary.yaml", text.str());
}

/// 按终端的显示宽度补齐：汉字占两列。
std::string pad(std::string_view text, std::size_t width) {
    std::size_t columns = 0;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const auto byte = static_cast<unsigned char>(text[i]);
        if ((byte & 0xC0U) == 0x80U) continue;  // 多字节字符的后续字节
        columns += byte >= 0xE0U ? 2 : 1;
    }
    return std::string(text) + std::string(columns < width ? width - columns : 1, ' ');
}

void printTable(std::ostream& out, const std::vector<SequenceRun>& runs,
                const std::string& purpose) {
    const auto cell = [](const std::map<std::string, double>& metrics, const char* key) {
        const auto it = metrics.find(key);
        return it == metrics.end() ? std::string("-") : std::format("{:.4f}", it->second);
    };
    out << pad("序列", 28) << pad("退出码", 8) << pad("结论", 14);
    for (const char* header : kHeaders) out << pad(header, 12);
    out << '\n';
    for (const auto& run : runs) {
        const auto metrics = values(run);
        out << pad(run.name, 28) << pad(std::to_string(run.exit_code), 8)
            << pad(status(run, purpose), 14);
        for (const char* key : kColumns) out << pad(cell(metrics, key), 12);
        out << '\n';
    }
    const auto average = mean(runs);
    out << pad("平均（每个序列都有结果才算）", 50);
    for (const char* key : kColumns) out << pad(cell(average, key), 12);
    out << '\n';
}

}  // namespace

int runSequences(const ResolvedTask& task, const RunOptions& options, std::ostream& out) {
    const auto& c = task.config;
    if (options.jobs == 0) throw std::invalid_argument("--jobs 至少为 1");
    requireSequenceField(c);
    const fs::path dir = archive::directory(c);
    archive::refuseOccupied(dir, options.overwrite);
    auto runs = plan(task, options, dir);
    archive::prepare(dir, kSuiteFiles);
    fs::create_directories(dir / "logs");
    config::writeConfigFile((dir / "task.yaml").string(), c);
    writeIdentity(dir, task);
    out << "多序列: " << runs.size() << " 个，同时跑 " << std::min(options.jobs, runs.size())
        << " 个" << std::endl;

    runAll(runs, options.jobs, out);
    for (auto& run : runs) collect(run, c.task.purpose);
    const int exit_code = worstExitCode(runs);
    writeSummary(dir, c, runs, exit_code);
    printTable(out, runs, c.task.purpose);
    out << "汇总: " << (dir / "summary.yaml").string() << '\n';
    return exit_code;
}

}  // namespace simpleslam::app
