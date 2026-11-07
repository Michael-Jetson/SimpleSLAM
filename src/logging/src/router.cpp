// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/logging/router.hpp>

#include "node.hpp"

#include <algorithm>
#include <cstdio>
#include <format>
#include <iostream>
#include <stdexcept>

namespace simpleslam::logging {

namespace {

/// 键按段前缀匹配：`odometry` 管到 `odometry.plugin.map`，管不到 `odometry2`。
bool matches(std::string_view rule, std::string_view key) {
    return key == rule
           || (key.size() > rule.size() && key.starts_with(rule) && key[rule.size()] == '.');
}

std::string seconds(std::int64_t ns) {
    return std::format("{:.3f}", static_cast<double>(ns) * 1e-9);
}

std::string frameText(const std::optional<Frame>& frame) {
    return frame ? std::format("#{}", frame->sequence) : std::string("-");
}

std::string_view baseName(std::string_view path) {
    const auto slash = path.find_last_of('/');
    return slash == std::string_view::npos ? path : path.substr(slash + 1);
}

/// 终端：等级、时刻、帧序号、来源、正文。执行器自己的输出只印正文。
std::string consoleLine(const Record& record) {
    if (record.plain) return record.text;
    return std::format("{} {} {} {}  {}", detail::levelLetter(record.level),
                       seconds(record.time_ns), frameText(record.frame), record.source,
                       record.text);
}

/// run.log：等级、时刻、帧序号、来源、正文，执行器自己的输出也带；模块与插件的记录再加代码位置。
std::string fileLine(const Record& record) {
    std::string where;
    if (!record.plain && record.where.line() != 0) {
        where = std::format(" ({}:{})", baseName(record.where.file_name()), record.where.line());
    }
    return std::format("{} {} {} {}{}  {}", detail::levelLetter(record.level),
                       seconds(record.time_ns), frameText(record.frame), record.source, where,
                       record.text);
}

/// warn、error 与执行器自己的输出不丢。
bool important(const Record& record) { return record.plain || record.level >= Level::kWarn; }

}  // namespace

std::shared_ptr<Router> Router::create(LogOptions options) {
    auto router = std::make_shared<Router>(Passkey{}, std::move(options));
    if (router->options_.asynchronous) {
        router->writer_ = std::thread([raw = router.get()] { raw->run(); });
    }
    return router;
}

Router::Router(Passkey /*key*/, LogOptions options)
    : options_(std::move(options)),
      out_(options_.out != nullptr ? options_.out : &std::cout),
      err_(options_.err != nullptr ? options_.err : &std::cerr) {
    options_.capacity = std::max<std::size_t>(options_.capacity, 1);
}

Router::~Router() { close(); }

Logger Router::root(std::string_view name) {
    const auto slash = name.find_last_of('/');
    std::string key(slash == std::string_view::npos ? name : name.substr(slash + 1));
    return root(name, std::move(key), false);
}

Logger Router::root(std::string_view name, std::string key, bool plain) {
    std::lock_guard lock(roots_mutex_);
    if (const auto it = roots_.find(name); it != roots_.end()) {
        if (auto node = it->second.lock()) return detail::Access::make(std::move(node));
    }
    detail::Node::Init init;
    init.key_segment = std::move(key);
    init.display_segment = std::string(name);
    init.router = shared_from_this();
    init.root = true;
    init.plain = plain;
    auto node = std::make_shared<detail::Node>(std::move(init));
    roots_.insert_or_assign(std::string(name), node);
    return detail::Access::make(std::move(node));
}

void Router::setClock(std::function<std::int64_t()> now) { clock_ = std::move(now); }

void Router::setEventSink(std::function<void(const Event&)> sink) {
    event_sink_ = std::move(sink);
}

void Router::openFile(const std::filesystem::path& path) {
    std::lock_guard lock(delivery_mutex_);
    file_.open(path, std::ios::out | std::ios::trunc);
    if (!file_) throw std::runtime_error("打不开日志文件: " + path.string());
    file_open_ = true;
    for (const auto& line : pending_file_) file_ << line << '\n';
    pending_file_.clear();
    file_.flush();
}

void Router::flush() {
    if (!options_.asynchronous) {
        std::lock_guard lock(delivery_mutex_);
        out_->flush();
        err_->flush();
        if (file_open_) file_.flush();
        return;
    }
    std::unique_lock lock(queue_mutex_);
    idle_.wait(lock, [this] {
        return closed_.load() || (ordinary_.empty() && important_.empty() && !writing_);
    });
}

void Router::close() {
    {
        std::lock_guard lock(queue_mutex_);
        if (stop_) return;
        stop_ = true;
    }
    wake_.notify_all();
    if (writer_.joinable()) writer_.join();
    {
        std::lock_guard lock(delivery_mutex_);
        out_->flush();
        err_->flush();
        if (file_open_) {
            file_.flush();
            file_.close();
            file_open_ = false;
        }
        pending_file_.clear();
    }
    closed_.store(true);
    idle_.notify_all();
}

std::uint64_t Router::dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

std::pair<Level, Level> Router::thresholds(std::string_view key) const {
    const LevelRule* best = nullptr;
    for (const auto& rule : options_.rules) {
        if (matches(rule.key, key) && (best == nullptr || rule.key.size() > best->key.size())) {
            best = &rule;
        }
    }
    if (best != nullptr) return {best->level, best->level};
    return {options_.console, options_.file};
}

std::int64_t Router::now() const { return clock_ ? clock_() : 0; }

void Router::submit(Record record) {
    if (!options_.asynchronous) {
        if (closed_.load()) {
            deliverAfterClose(record);
            return;
        }
        std::lock_guard lock(delivery_mutex_);
        deliver(record);
        out_->flush();
        err_->flush();
        return;
    }
    {
        std::unique_lock lock(queue_mutex_);
        if (!stop_) {
            if (important(record)) {
                important_.push_back(Queued{.order = next_order_++, .record = std::move(record)});
            } else {
                if (ordinary_.size() >= options_.capacity) {
                    ordinary_.pop_front();
                    dropped_.fetch_add(1, std::memory_order_relaxed);
                }
                ordinary_.push_back(Queued{.order = next_order_++, .record = std::move(record)});
            }
            lock.unlock();
            wake_.notify_one();
            return;
        }
    }
    deliverAfterClose(record);  // 已经停下：在调用线程上直接写 stderr
}

void Router::submitEvent(const Event& event) const {
    if (!event_sink_) return;
    try {
        event_sink_(event);
    } catch (...) {  // NOLINT(bugprone-empty-catch): 诊断路径不得影响算法
    }
}

void Router::run() {
    std::unique_lock lock(queue_mutex_);
    for (;;) {
        wake_.wait(lock, [this] { return stop_ || !ordinary_.empty() || !important_.empty(); });
        if (ordinary_.empty() && important_.empty()) {
            if (stop_) break;
            continue;
        }
        std::deque<Queued> ordinary;
        std::deque<Queued> important;
        ordinary.swap(ordinary_);
        important.swap(important_);
        writing_ = true;
        lock.unlock();
        {
            // 两队按序号合并，保持记下时的先后
            std::lock_guard delivery(delivery_mutex_);
            auto a = ordinary.begin();
            auto b = important.begin();
            while (a != ordinary.end() || b != important.end()) {
                const bool from_ordinary =
                    b == important.end() || (a != ordinary.end() && a->order < b->order);
                deliver((from_ordinary ? a++ : b++)->record);
            }
            out_->flush();
            err_->flush();
            if (file_open_) file_.flush();
        }
        lock.lock();
        writing_ = false;
        idle_.notify_all();
    }
}

void Router::deliver(const Record& record) {
    if (record.to_console) {
        std::ostream& stream = record.level >= Level::kWarn ? *err_ : *out_;
        stream << consoleLine(record) << '\n';
    }
    if (record.to_file) {
        if (file_open_) {
            file_ << fileLine(record) << '\n';
        } else if (options_.defer_file) {
            pending_file_.push_back(fileLine(record));
            if (pending_file_.size() > options_.capacity) pending_file_.pop_front();
        }
    }
    if (options_.tap) {
        try {
            options_.tap(record);
        } catch (...) {  // NOLINT(bugprone-empty-catch): 接收端的错不影响别的去向
        }
    }
}

void Router::deliverAfterClose(const Record& record) const {
    if (!record.to_console && !record.to_file) return;
    const std::string line = fileLine(record) + '\n';
    std::fwrite(line.data(), 1, line.size(), stderr);
}

}  // namespace simpleslam::logging
