// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file router.hpp
/// 日志组件：来源树的根、按来源的等级、文字日志的去向，以及一个后台线程。
///
/// 文字日志：调用处只做等级判断、格式化、入队，落盘（终端、run.log、另一个接收端）交给
/// 后台线程，算法线程从不等 I/O。缓冲有界：满了丢最旧的 debug、info 并计数；warn、error
/// 与执行器自己的输出不丢。事件与度量不经缓冲，在调用线程上直接交给诊断，一条不丢。
///
/// 一个系统一个日志组件（`SlamSystem` 自带），不是全局单例：测试里各建各的。

#include <SimpleSLAM/logging/logger.hpp>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace simpleslam::logging {

/// 按来源设等级：键按段前缀匹配（`odometry` 管到 `odometry.plugin.map`），最长的那条生效，
/// 终端与 run.log 都照它。
struct LevelRule {
    std::string key{};
    Level level = Level::kInfo;
};

/// 一条文字日志。
struct Record {
    Level level = Level::kInfo;
    std::int64_t time_ns = 0;  ///< 运行时的时钟：离线是数据时间，在线是墙钟
    std::optional<Frame> frame{};
    std::string source{};  ///< 显示形式
    std::string key{};     ///< 按来源设等级用的键
    /// 执行器自己的输出：终端上只印正文，不带前缀。
    bool plain = false;
    std::source_location where{};
    std::string text{};
    bool to_console = true;
    bool to_file = true;
};

/// 一条事件或度量。
struct Event {
    enum class Kind : std::uint8_t { kEvent, kMetric };
    Kind kind = Kind::kEvent;
    std::string module{};  ///< 来源树的根：模块名
    std::string source{};  ///< 来源的键
    std::int64_t time_ns = 0;
    std::optional<Frame> frame{};
    std::string key{};
    std::string detail{};
    double value = 0.0;
    std::string unit{};
};

struct LogOptions {
    Level console = Level::kInfo;  ///< 终端显示的最低等级
    Level file = Level::kDebug;    ///< run.log 记录的最低等级
    std::vector<LevelRule> rules{};
    std::ostream* out = nullptr;  ///< 终端上的 debug、info；空是 std::cout
    std::ostream* err = nullptr;  ///< 终端上的 warn、error；空是 std::cerr
    /// 后台线程落盘。false 时在调用线程上直接写：单独用通信层、不另开线程时用。
    bool asynchronous = true;
    /// run.log 稍后才接上（`openFile`）：在那之前该进文件的记录先存着，接上时补写。
    bool defer_file = false;
    /// 缓冲里至多存这么多条 debug、info。
    std::size_t capacity = 8192;
    /// 另一个接收端，在后台线程上收到每条记下的记录：测试里收集，以后转发 ROS 的 rosout。
    std::function<void(const Record&)> tap{};
};

class Router : public std::enable_shared_from_this<Router> {
    struct Passkey {};

public:
    [[nodiscard]] static std::shared_ptr<Router> create(LogOptions options = {});
    Router(Passkey, LogOptions options);
    ~Router();
    Router(const Router&) = delete;
    Router& operator=(const Router&) = delete;

    /// 来源树的根：一个模块。同一个名字拿到同一个根，键取名字的最后一段（`/slam/odometry` 是
    /// `odometry`）。插件的槽在装配时把来源挂到它下面。
    [[nodiscard]] Logger root(std::string_view name);
    /// 指定键的根；`plain` 的根自己的记录在终端上只印正文（执行器的进度与结论）。
    [[nodiscard]] Logger root(std::string_view name, std::string key, bool plain);

    /// 记录的时刻从哪里来：运行时的时钟。装配时设，运行中不改。
    void setClock(std::function<std::int64_t()> now);
    /// 事件与度量交给谁：诊断。装配时设，运行中不改。
    void setEventSink(std::function<void(const Event&)> sink);

    /// 接上 run.log：先补写之前存着的记录。打不开抛 `std::runtime_error`。
    void openFile(const std::filesystem::path& path);
    /// 等缓冲里的记录都写完。
    void flush();
    /// 写完、停下后台线程、关文件。之后的记录在调用线程上直接写 stderr。
    void close();
    /// 缓冲满了丢掉的 debug、info 条数。
    [[nodiscard]] std::uint64_t dropped() const noexcept;

    // ── 日志对象用 ──

    /// 某个键的终端与 run.log 等级。
    [[nodiscard]] std::pair<Level, Level> thresholds(std::string_view key) const;
    [[nodiscard]] std::int64_t now() const;
    void submit(Record record);
    void submitEvent(const Event& event) const;

private:
    struct Queued {
        std::uint64_t order = 0;
        Record record;
    };

    void run();
    void deliver(const Record& record);
    void deliverAfterClose(const Record& record) const;

    LogOptions options_;
    std::ostream* out_;
    std::ostream* err_;
    std::function<std::int64_t()> clock_;
    std::function<void(const Event&)> event_sink_;

    std::mutex roots_mutex_;
    std::map<std::string, std::weak_ptr<detail::Node>, std::less<>> roots_;

    // 缓冲：debug、info 一队（有界），warn、error 与执行器的输出一队（不丢），按序号合并
    std::mutex queue_mutex_;
    std::condition_variable wake_;
    std::condition_variable idle_;
    std::deque<Queued> ordinary_;
    std::deque<Queued> important_;
    std::uint64_t next_order_ = 0;
    bool stop_ = false;
    bool writing_ = false;
    std::atomic<bool> closed_{false};
    std::atomic<std::uint64_t> dropped_{0};
    std::thread writer_;

    // 落盘：后台线程写，同步模式下在调用线程上写；接文件与它互斥
    std::mutex delivery_mutex_;
    std::ofstream file_;
    bool file_open_ = false;
    std::deque<std::string> pending_file_;
};

}  // namespace simpleslam::logging
