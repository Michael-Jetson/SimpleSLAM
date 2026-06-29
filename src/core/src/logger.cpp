/// @file logger.cpp
/// 日志系统实现

#include <SimpleSLAM/core/infra/logger.hpp>

#include <mutex>
#include <unordered_map>

#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <yaml-cpp/yaml.h>

namespace simpleslam {

namespace {

inline constexpr size_t kDefaultBacktraceSize = 32;
inline constexpr size_t kDefaultQueueSize = 8192;

struct LoggerState {
    std::mutex mutex;
    std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> loggers;
    bool initialized = false;
    bool async_mode = true;
    spdlog::level::level_enum console_level = spdlog::level::info;
    spdlog::level::level_enum file_level = spdlog::level::debug;
    std::string file_path;
    size_t backtrace_size = kDefaultBacktraceSize;
    std::unordered_map<std::string, spdlog::level::level_enum> module_levels;
};

LoggerState& state() {
    static LoggerState s;
    return s;
}

spdlog::level::level_enum parseLevel(const std::string& str) {
    // spdlog 自带 level_from_str，但不处理 "warning"/"err" 等别名
    static const std::unordered_map<std::string, spdlog::level::level_enum> table{
        {"trace", spdlog::level::trace},     {"debug", spdlog::level::debug},
        {"info", spdlog::level::info},       {"warn", spdlog::level::warn},
        {"warning", spdlog::level::warn},    {"error", spdlog::level::err},
        {"err", spdlog::level::err},         {"critical", spdlog::level::critical},
        {"off", spdlog::level::off},
    };
    auto it = table.find(str);
    return (it != table.end()) ? it->second : spdlog::level::info;
}

std::shared_ptr<spdlog::logger> createLogger(const std::string& name) {
    auto& s = state();
    std::vector<spdlog::sink_ptr> sinks;

    // 控制台 sink（带颜色）—— %n 是 spdlog 内置的 logger name 占位符
    auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console->set_level(s.console_level);
    console->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
    sinks.push_back(console);

    // 文件 sink（可选）
    if (!s.file_path.empty()) {
        auto file = std::make_shared<spdlog::sinks::basic_file_sink_mt>(s.file_path, false);
        file->set_level(s.file_level);
        file->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");
        sinks.push_back(file);
    }

    std::shared_ptr<spdlog::logger> logger;
    if (s.async_mode) {
        logger = std::make_shared<spdlog::async_logger>(
            name, sinks.begin(), sinks.end(),
            spdlog::thread_pool(),
            spdlog::async_overflow_policy::overrun_oldest);
    } else {
        logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
    }

    // 模块专属级别覆盖（如 odometry: debug）
    if (auto it = s.module_levels.find(name); it != s.module_levels.end()) {
        logger->set_level(it->second);
    } else {
        logger->set_level(spdlog::level::trace);  // logger 不过滤，由 sink 控制
    }

    if (s.backtrace_size > 0) {
        logger->enable_backtrace(s.backtrace_size);
    }

    // drop_all() 后重新创建同名 logger 时，spdlog 注册表中可能已有残留
    // 先 drop 再 register 避免重复注册异常
    spdlog::drop(name);
    spdlog::register_logger(logger);
    return logger;
}

}  // namespace

void Logger::init(const YAML::Node& config) {
    auto& s = state();
    std::lock_guard lock(s.mutex);

    if (s.initialized) {
        // 防止二次初始化导致 spdlog 线程池重复创建
        fprintf(stderr, "[SimpleSLAM] Logger::init() called twice, ignoring\n");
        return;
    }

    s.async_mode = config["async"].as<bool>(true);
    s.console_level = parseLevel(config["console_level"].as<std::string>("info"));
    s.file_level = parseLevel(config["file_level"].as<std::string>("debug"));
    s.file_path = config["file_path"].as<std::string>("");
    s.backtrace_size = config["backtrace_size"].as<size_t>(kDefaultBacktraceSize);

    auto queue_size = config["async_queue_size"].as<size_t>(kDefaultQueueSize);

    if (config["levels"] && config["levels"].IsMap()) {
        for (auto it = config["levels"].begin(); it != config["levels"].end(); ++it) {
            s.module_levels[it->first.as<std::string>()] =
                parseLevel(it->second.as<std::string>());
        }
    }

    if (s.async_mode) {
        spdlog::init_thread_pool(queue_size, 1);
    }

    s.initialized = true;
}

void Logger::initDefault() {
    auto& s = state();
    std::lock_guard lock(s.mutex);

    if (s.initialized) return;

    s.async_mode = false;
    s.console_level = spdlog::level::info;
    s.file_path = "";
    s.backtrace_size = kDefaultBacktraceSize;
    s.initialized = true;
}

std::shared_ptr<spdlog::logger> Logger::get(const std::string& module_name) {
    auto& s = state();
    std::lock_guard lock(s.mutex);

    if (auto it = s.loggers.find(module_name); it != s.loggers.end()) {
        return it->second;
    }

    // 首次 get 且未初始化——静默启用默认配置
    if (!s.initialized) {
        s.async_mode = false;
        s.console_level = spdlog::level::info;
        s.initialized = true;
    }

    auto logger = createLogger(module_name);
    s.loggers[module_name] = logger;
    return logger;
}

void Logger::dumpBacktrace(const std::string& module_name) {
    auto& s = state();
    std::lock_guard lock(s.mutex);

    if (auto it = s.loggers.find(module_name); it != s.loggers.end()) {
        it->second->dump_backtrace();
    }
}

void Logger::shutdown() {
    auto& s = state();
    std::lock_guard lock(s.mutex);

    // 显式释放所有 logger 引用，避免 static 析构顺序问题
    s.loggers.clear();
    spdlog::shutdown();
    s.initialized = false;
}

}  // namespace simpleslam
