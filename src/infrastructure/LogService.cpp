#include <simpleslam/infrastructure/LogService.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>

#include <spdlog/async.h>
#include <spdlog/async_logger.h>
#include <spdlog/details/thread_pool.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace SimpleSLAM {
namespace {

using ThreadPoolPtr = std::shared_ptr<spdlog::details::thread_pool>;

std::mutex& threadPoolMutex() {
    static std::mutex mutex;
    return mutex;
}

std::unordered_map<const LogService*, ThreadPoolPtr>& threadPools() {
    static std::unordered_map<const LogService*, ThreadPoolPtr> pools;
    return pools;
}

void setThreadPool(const LogService* service, ThreadPoolPtr pool) {
    std::lock_guard lock(threadPoolMutex());
    threadPools()[service] = std::move(pool);
}

ThreadPoolPtr getThreadPool(const LogService* service) {
    std::lock_guard lock(threadPoolMutex());
    auto it = threadPools().find(service);
    return it == threadPools().end() ? nullptr : it->second;
}

void eraseThreadPool(const LogService* service) {
    std::lock_guard lock(threadPoolMutex());
    threadPools().erase(service);
}

std::string internalLoggerName(const LogService* service, const std::string& encoded_name) {
    return "simpleslam:" + std::to_string(reinterpret_cast<std::uintptr_t>(service)) + "::" + encoded_name;
}

std::string_view displayLoggerName(std::string_view internal_name) {
    const auto pos = internal_name.find("::");
    if (pos == std::string_view::npos) {
        return internal_name;
    }
    return internal_name.substr(pos + 2);
}

std::string rewritePattern(std::string_view pattern) {
    std::string result;
    result.reserve(pattern.size());

    for (size_t i = 0; i < pattern.size(); ++i) {
        if (pattern[i] == '%' && (i + 1) < pattern.size() && pattern[i + 1] == 'P') {
            result += "%Q";
            ++i;
            continue;
        }
        result.push_back(pattern[i]);
    }

    return result;
}

class PaddedTagFlag final : public spdlog::custom_flag_formatter {
public:
    explicit PaddedTagFlag(std::shared_ptr<std::atomic<size_t>> width)
        : width_(std::move(width)) {}

    void format(const spdlog::details::log_msg& msg, const std::tm&, spdlog::memory_buf_t& dest) override {
        const std::string_view internal_name(msg.logger_name.data(), msg.logger_name.size());
        const std::string_view display_name = displayLoggerName(internal_name);

        dest.append(display_name.begin(), display_name.end());

        const size_t width = width_ ? width_->load(std::memory_order_relaxed) : 0;
        for (size_t i = display_name.size(); i < width; ++i) {
            dest.push_back(' ');
        }
    }

    std::unique_ptr<spdlog::custom_flag_formatter> clone() const override {
        return std::make_unique<PaddedTagFlag>(width_);
    }

private:
    std::shared_ptr<std::atomic<size_t>> width_;
};

std::unique_ptr<spdlog::formatter> makeFormatter(const std::string& pattern,
                                                 const std::shared_ptr<std::atomic<size_t>>& width) {
    auto formatter = std::make_unique<spdlog::pattern_formatter>();
    formatter->add_flag<PaddedTagFlag>('Q', width).set_pattern(rewritePattern(pattern));
    return formatter;
}

} // namespace

Log::Log(std::shared_ptr<spdlog::logger> lg)
    : lg_(std::move(lg)) {}

spdlog::logger* Log::operator->() const {
    return get_safe();
}

std::shared_ptr<spdlog::logger> Log::shared() const {
    return lg_ ? lg_ : null_logger();
}

Log::operator bool() const {
    return static_cast<bool>(lg_);
}

std::shared_ptr<spdlog::logger> Log::null_logger() {
    static auto logger = [] {
        auto sink = std::make_shared<spdlog::sinks::null_sink_mt>();
        auto lg = std::make_shared<spdlog::logger>("simpleslam.null", std::move(sink));
        lg->set_level(spdlog::level::trace);
        return lg;
    }();
    return logger;
}

spdlog::logger* Log::get_safe() const {
    // Avoid shared_ptr copy (atomic refcount +1/-1) on every log call.
    // null_logger() returns a static shared_ptr whose pointee lives forever,
    // so the raw pointer remains valid without extending the refcount.
    return lg_ ? lg_.get() : null_logger().get();
}

LogService::LogService(const LogConfig& cfg)
    : cfg_(cfg),
      tag_width_(std::make_shared<std::atomic<size_t>>(0)) {
    if (cfg_.enable_console) {
        auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        sink->set_level(cfg_.console_level);
        sink->set_formatter(makeFormatter(cfg_.console_pattern, tag_width_));
        sinks_.push_back(std::move(sink));
    }

    if (cfg_.enable_file) {
        const std::filesystem::path file_path(cfg_.file_path);
        const auto parent = file_path.parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }

        auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(cfg_.file_path, false);
        sink->set_level(cfg_.file_level);
        sink->set_formatter(makeFormatter(cfg_.file_pattern, tag_width_));
        sinks_.push_back(std::move(sink));
    }

    // #19 fix: warn if no sinks configured (silent log loss)
    if (sinks_.empty()) {
        std::fprintf(stderr, "[LogService] WARNING: no sinks configured "
                     "(enable_console=%d, enable_file=%d). All logs will be discarded.\n",
                     cfg_.enable_console, cfg_.enable_file);
    }

    if (cfg_.async) {
        setThreadPool(this, std::make_shared<spdlog::details::thread_pool>(cfg_.async_queue_size, cfg_.async_threads));
    }

    inited_.store(true, std::memory_order_release);
}

LogService::~LogService() {
    shutdown();
    eraseThreadPool(this);
}

Log LogService::getLogger(std::string_view module) {
    return Log(getOrCreate(encode({module})));
}

Log LogService::getLogger(std::string_view module, std::string_view plugin) {
    return Log(getOrCreate(encode({module, plugin})));
}

Log LogService::getLogger(std::initializer_list<std::string_view> tags) {
    return Log(getOrCreate(encode(tags)));
}

void LogService::setLoggerLevel(std::string_view encoded_name, spdlog::level::level_enum lv) {
    std::lock_guard lock(mu_);

    const std::string name(encoded_name);
    auto it = loggers_.find(name);
    if (it != loggers_.end()) {
        it->second->set_level(lv);
    }

    level_overrides_[name] = lv;
}

void LogService::shutdown() {
    std::lock_guard lock(mu_);
    if (!inited_.load(std::memory_order_relaxed)) {
        return;
    }

    for (auto& [name, logger] : loggers_) {
        (void)name;
        logger->flush();
    }

    for (auto& [name, logger] : loggers_) {
        (void)name;
        spdlog::drop(logger->name());
    }
    loggers_.clear();

    inited_.store(false, std::memory_order_release);
}

void LogService::flush() {
    std::lock_guard lock(mu_);
    for (auto& [name, logger] : loggers_) {
        (void)name;
        logger->flush();
    }
}

std::shared_ptr<spdlog::logger> LogService::getOrCreate(const std::string& encoded_name) {
    std::lock_guard lock(mu_);

    auto it = loggers_.find(encoded_name);
    if (it != loggers_.end()) {
        return it->second;
    }

    std::shared_ptr<spdlog::logger> logger;
    const auto logger_name = internalLoggerName(this, encoded_name);

    if (cfg_.async) {
        logger = std::make_shared<spdlog::async_logger>(
            logger_name,
            sinks_.begin(),
            sinks_.end(),
            getThreadPool(this),
            spdlog::async_overflow_policy::overrun_oldest);
    } else {
        logger = std::make_shared<spdlog::logger>(logger_name, sinks_.begin(), sinks_.end());
    }

    // #20 fix: logger level = min(all sink levels), not just file_level
    // This ensures console logs at DEBUG are not filtered when file is at INFO
    auto override_it = level_overrides_.find(encoded_name);
    if (override_it != level_overrides_.end()) {
        logger->set_level(override_it->second);
    } else {
        auto min_level = spdlog::level::off;
        for (const auto& sink : sinks_) {
            if (sink->level() < min_level) {
                min_level = sink->level();
            }
        }
        logger->set_level(min_level);
    }

    spdlog::register_logger(logger);
    loggers_[encoded_name] = logger;
    updateTagWidthFor(encoded_name);
    return logger;
}

std::string LogService::encode(std::initializer_list<std::string_view> tags) {
    std::string encoded;
    bool first = true;

    for (const auto tag : tags) {
        if (tag.empty()) {
            continue;
        }

        if (first) {
            encoded.push_back('[');
            first = false;
        } else {
            encoded += cfg_.tag_delim;
        }

        encoded.append(tag.data(), tag.size());
    }

    if (!first) {
        encoded.push_back(']');
    }

    return encoded;
}

void LogService::updateTagWidthFor(const std::string& encoded_name) {
    const size_t desired = encoded_name.size();
    size_t current = tag_width_->load(std::memory_order_relaxed);

    while (current < desired &&
           !tag_width_->compare_exchange_weak(current, desired, std::memory_order_relaxed, std::memory_order_relaxed)) {
    }
}

} // namespace SimpleSLAM
