#pragma once

#include <atomic>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <spdlog/logger.h>
#include <spdlog/spdlog.h>

#include <simpleslam/core/IService.h>

namespace SimpleSLAM {

struct LogConfig {
    bool enable_console = true;
    bool enable_file = true;
    std::string file_path = "logs/slam.log";
    spdlog::level::level_enum console_level = spdlog::level::info;
    spdlog::level::level_enum file_level = spdlog::level::debug;
    bool async = true;
    size_t async_queue_size = 8192;
    size_t async_threads = 1;
    std::string console_pattern = "[%H:%M:%S.%e][%^%l%$][%t]%P %v";
    std::string file_pattern = "[%Y-%m-%d %H:%M:%S.%e][%l]%P %v";
    // custom_console_colors: reserved for Phase 3+, not yet consumed by LogService
    // bool custom_console_colors = false;
    std::string tag_delim = "][";
};

class Log {
public:
    Log() = default;
    explicit Log(std::shared_ptr<spdlog::logger> lg);

    template <typename... Args>
    void trace(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
        get_safe()->trace(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void debug(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
        get_safe()->debug(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void info(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
        get_safe()->info(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void warn(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
        get_safe()->warn(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void error(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
        get_safe()->error(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void critical(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
        get_safe()->critical(fmt, std::forward<Args>(args)...);
    }

    spdlog::logger* operator->() const;
    std::shared_ptr<spdlog::logger> shared() const;
    explicit operator bool() const;

private:
    std::shared_ptr<spdlog::logger> lg_;

    static std::shared_ptr<spdlog::logger> null_logger();
    spdlog::logger* get_safe() const;
};

class LogService : public IService {
public:
    explicit LogService(const LogConfig& cfg);
    ~LogService() override;

    LogService(const LogService&) = delete;
    LogService& operator=(const LogService&) = delete;

    Log getLogger(std::string_view module);
    Log getLogger(std::string_view module, std::string_view plugin);
    Log getLogger(std::initializer_list<std::string_view> tags);

    void setLoggerLevel(std::string_view encoded_name, spdlog::level::level_enum lv);

    void shutdown();
    void flush();

private:
    std::mutex mu_;
    std::atomic<bool> inited_{false};
    LogConfig cfg_;

    std::vector<spdlog::sink_ptr> sinks_;
    std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> loggers_;
    std::unordered_map<std::string, spdlog::level::level_enum> level_overrides_;

    std::shared_ptr<std::atomic<size_t>> tag_width_;

    std::shared_ptr<spdlog::logger> getOrCreate(const std::string& encoded_name);
    std::string encode(std::initializer_list<std::string_view> tags);
    void updateTagWidthFor(const std::string& encoded_name);
};

} // namespace SimpleSLAM
