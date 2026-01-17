// log_manager.h
#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/pattern_formatter.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include <fmt/format.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <utility>
#include <initializer_list>

namespace SimpleSLAM {

//=============================
// Config
//=============================
struct LogConfig {
  // 开关：不必两种都要
  bool enable_console = true;
  bool enable_file    = true;

  // 文件输出（rotating）
  std::string file_path = "slam.log";
  size_t max_file_size = 50 * 1024 * 1024; // 50MB
  size_t max_files     = 5;

  // async（推荐 true）
  bool async = true;
  size_t async_queue_size = 8192;
  size_t async_threads    = 1;

  // sink 级别：推荐 console 高一点、file 低一点
  spdlog::level::level_enum console_level = spdlog::level::info;
  spdlog::level::level_enum file_level    = spdlog::level::trace;

  // logger 自身 level：建议设为 trace，让 sink 控制输出策略
  spdlog::level::level_enum logger_level  = spdlog::level::trace;

  // 标签层级分隔符：Front|ORB => [Front][ORB]
  char tag_delim = '|';

  // 时间戳后加级别；console 用 ^$ 对 level 上色
  // %P 是我们自定义的 “对齐标签前缀”
  std::string file_pattern    = "[%Y-%m-%d %H:%M:%S.%e][%l]%P %v";
  std::string console_pattern = "[%H:%M:%S.%e][%^%l%$][%t]%P %v";

  // 是否自定义 console 各级别颜色（可选）
  bool custom_console_colors = false;
};

//=============================
// Log handle: object-friendly + pointer-friendly + convertible
//=============================
class Log {
public:
  Log() = default;
  explicit Log(std::shared_ptr<spdlog::logger> lg) : lg_(std::move(lg)) {}

  // 对象也允许 log->info(...)
  spdlog::logger* operator->() const { return lg_.get(); }

  template <typename... Args>
  void trace(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
    lg_->trace(fmt, std::forward<Args>(args)...);
  }
  template <typename... Args>
  void debug(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
    lg_->debug(fmt, std::forward<Args>(args)...);
  }
  template <typename... Args>
  void info(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
    lg_->info(fmt, std::forward<Args>(args)...);
  }
  template <typename... Args>
  void warn(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
    lg_->warn(fmt, std::forward<Args>(args)...);
  }
  template <typename... Args>
  void error(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
    lg_->error(fmt, std::forward<Args>(args)...);
  }
  template <typename... Args>
  void critical(spdlog::format_string_t<Args...> fmt, Args&&... args) const {
    lg_->critical(fmt, std::forward<Args>(args)...);
  }

  // Log -> shared_ptr
  std::shared_ptr<spdlog::logger> shared() const { return lg_; }
  // 可选：拿裸指针
  spdlog::logger* get() const { return lg_.get(); }

  explicit operator bool() const { return (bool)lg_; }

private:
  std::shared_ptr<spdlog::logger> lg_;
};

//=============================
// Custom %P: prints [Front][ORB] with auto alignment
//=============================
class TagPrefixFlag final : public spdlog::custom_flag_formatter {
public:
  TagPrefixFlag(std::shared_ptr<std::atomic<size_t>> width, char delim)
      : width_(std::move(width)), delim_(delim) {}

  void format(const spdlog::details::log_msg& msg,
              const std::tm&,
              spdlog::memory_buf_t& dest) override {
    const std::string_view name_sv{msg.logger_name.data(), msg.logger_name.size()};
    const std::string prefix = BuildPrefix(name_sv, delim_);
    const size_t w = width_->load(std::memory_order_relaxed);
    // 左对齐填充到 w，保证内容列对齐
    fmt::format_to(std::back_inserter(dest), "{:<{}}", prefix, w);
  }

  std::unique_ptr<custom_flag_formatter> clone() const override {
    return std::make_unique<TagPrefixFlag>(width_, delim_);
  }

  static std::string BuildPrefix(std::string_view encoded_name, char delim) {
    std::string out;
    size_t start = 0;
    while (start <= encoded_name.size()) {
      size_t pos = encoded_name.find(delim, start);
      const bool last = (pos == std::string_view::npos);
      std::string_view part = last
          ? encoded_name.substr(start)
          : encoded_name.substr(start, pos - start);

      if (!part.empty()) {
        out.push_back('[');
        out.append(part.begin(), part.end());
        out.push_back(']');
      }
      if (last) break;
      start = pos + 1;
    }
    // 你也可以选择在这里 out.push_back(' ') 来固定分隔
    // 但目前用 pattern 里写 "%P %v" 更灵活
    return out;
  }

private:
  std::shared_ptr<std::atomic<size_t>> width_;
  char delim_;
};

//=============================
// LogManager
//=============================
class LogManager {
public:
  static LogManager& Instance() {
    static LogManager inst;
    return inst;
  }

  // 显式初始化（建议 main() 最开始调用）
  void Init(const LogConfig& cfg) {
    std::lock_guard<std::mutex> lk(mu_);
    if (inited_) return;

    cfg_ = cfg;
    tag_width_ = std::make_shared<std::atomic<size_t>>(0);

    sinks_.clear();

    if (cfg_.enable_console) {
      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(cfg_.console_level);
      if (cfg_.custom_console_colors) {
        // 你可按喜好改颜色
        console_sink->set_color(spdlog::level::trace, console_sink->magenta);
        console_sink->set_color(spdlog::level::debug, console_sink->cyan);
        console_sink->set_color(spdlog::level::info,  console_sink->green);
        console_sink->set_color(spdlog::level::warn,  console_sink->yellow);
        console_sink->set_color(spdlog::level::err,   console_sink->red);
        console_sink->set_color(spdlog::level::critical, console_sink->red_bold);
      }
      console_sink->set_formatter(MakeFormatter(cfg_.console_pattern));
      sinks_.push_back(console_sink);
    }

    if (cfg_.enable_file) {
      auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
          cfg_.file_path, cfg_.max_file_size, cfg_.max_files);
      file_sink->set_level(cfg_.file_level);
      file_sink->set_formatter(MakeFormatter(cfg_.file_pattern));
      sinks_.push_back(file_sink);
    }

    if (sinks_.empty()) {
      throw std::runtime_error("LogManager::Init: no sinks enabled (enable_console/enable_file are both false).");
    }

    if (cfg_.async) {
      // 注意：只初始化一次（我们用 inited_ 保证）
      spdlog::init_thread_pool(cfg_.async_queue_size, cfg_.async_threads);
    }

    inited_ = true;
  }

  // 可选：你不想强制 main() Init，也可以懒初始化默认配置
  void EnsureInit() {
    if (inited_) return;
    Init(LogConfig{});
  }

  // 模块 logger: [Front]
  Log Module(std::string_view module) {
    EnsureInit();
    return Log(GetOrCreate(Encode({module})));
  }

  // 插件 logger: [Front][ORB]
  Log Plugin(std::string_view module, std::string_view plugin) {
    EnsureInit();
    return Log(GetOrCreate(Encode({module, plugin})));
  }

  // 任意层级 tags
  Log Tags(std::initializer_list<std::string_view> tags) {
    EnsureInit();
    return Log(GetOrCreate(Encode(tags)));
  }

  // 如果你直接要 shared_ptr（绕过 Log）
  std::shared_ptr<spdlog::logger> ModulePtr(std::string_view module) {
    EnsureInit();
    return GetOrCreate(Encode({module}));
  }

  // 运行时调整同名 logger 的 level（可选）
  void SetLoggerLevel(std::string_view encoded_name, spdlog::level::level_enum lv) {
    EnsureInit();
    std::lock_guard<std::mutex> lk(mu_);
    auto it = loggers_.find(std::string(encoded_name));
    if (it != loggers_.end()) it->second->set_level(lv);
  }

private:
  LogManager() = default;

  std::unique_ptr<spdlog::formatter> MakeFormatter(const std::string& pattern) {
    auto fmtter = std::make_unique<spdlog::pattern_formatter>();
    fmtter->add_flag<TagPrefixFlag>('P', tag_width_, cfg_.tag_delim);
    fmtter->set_pattern(pattern);
    return fmtter;
  }

  std::string Encode(std::initializer_list<std::string_view> tags) const {
    std::string name;
    bool first = true;
    for (auto t : tags) {
      if (!first) name.push_back(cfg_.tag_delim);
      first = false;
      name.append(t.begin(), t.end());
    }
    return name;
  }

  void UpdateTagWidthFor(const std::string& encoded_name) {
    const std::string prefix = TagPrefixFlag::BuildPrefix(encoded_name, cfg_.tag_delim);
    const size_t need = prefix.size();

    size_t cur = tag_width_->load(std::memory_order_relaxed);
    while (need > cur &&
           !tag_width_->compare_exchange_weak(cur, need,
                                             std::memory_order_release,
                                             std::memory_order_relaxed)) {
      // cur updated by compare_exchange_weak
    }
  }

  std::shared_ptr<spdlog::logger> GetOrCreate(const std::string& encoded_name) {
    std::lock_guard<std::mutex> lk(mu_);

    // 缓存：同名返回同一实例
    auto it = loggers_.find(encoded_name);
    if (it != loggers_.end()) return it->second;

    UpdateTagWidthFor(encoded_name);

    std::shared_ptr<spdlog::logger> logger;
    if (cfg_.async) {
      logger = std::make_shared<spdlog::async_logger>(
          encoded_name, sinks_.begin(), sinks_.end(),
          spdlog::thread_pool(), spdlog::async_overflow_policy::block);
    } else {
      logger = std::make_shared<spdlog::logger>(encoded_name, sinks_.begin(), sinks_.end());
    }

    logger->set_level(cfg_.logger_level);
    logger->flush_on(spdlog::level::warn);

    loggers_.emplace(encoded_name, logger);
    return logger;
  }

private:
  std::mutex mu_;
  bool inited_ = false;
  LogConfig cfg_;

  std::vector<spdlog::sink_ptr> sinks_;
  std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> loggers_;

  std::shared_ptr<std::atomic<size_t>> tag_width_;
};

} // namespace SimpleSLAM
