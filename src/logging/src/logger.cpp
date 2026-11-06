// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include "node.hpp"

#include <SimpleSLAM/logging/construction.hpp>
#include <SimpleSLAM/logging/router.hpp>

#include <algorithm>
#include <cstdio>
#include <format>
#include <mutex>
#include <tuple>
#include <vector>

namespace simpleslam::logging {

std::string_view toString(Level level) noexcept {
    switch (level) {
        case Level::kDebug: return "debug";
        case Level::kInfo: return "info";
        case Level::kWarn: return "warn";
        case Level::kError: return "error";
    }
    return "info";
}

std::optional<Level> parseLevel(std::string_view text) noexcept {
    for (const Level level : {Level::kDebug, Level::kInfo, Level::kWarn, Level::kError}) {
        if (text == toString(level)) return level;
    }
    return std::nullopt;
}

namespace detail {

namespace {

void append(std::string& out, std::string_view segment, std::string_view separator) {
    if (segment.empty()) return;
    if (!out.empty()) out += separator;
    out += segment;
}

std::int64_t steadyNow() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

/// 没接上的日志对象：同步写 stderr，整行一次写出，几个线程同时写也不会交错。
void writeDetached(Level level, const std::string& source, std::string_view text) {
    static std::mutex mutex;
    const std::string line =
        std::format("{} {}  {}\n", levelLetter(level), source.empty() ? "-" : source, text);
    std::lock_guard lock(mutex);
    std::fwrite(line.data(), 1, line.size(), stderr);
}

thread_local ConstructionScope* tls_scope = nullptr;

}  // namespace

Node::Node(Init init)
    : parent_(std::move(init.parent)),
      key_segment_(std::move(init.key_segment)),
      display_segment_(std::move(init.display_segment)),
      router_(std::move(init.router)),
      root_(init.root),
      plain_(init.plain),
      muted_(init.muted) {}

NodePtr Node::parent() const {
    std::lock_guard lock(mutex_);
    return parent_;
}

const Resolved* Node::resolved() {
    if (const Resolved* hit = cache_.load(std::memory_order_acquire)) return hit;
    if (muted_) return nullptr;
    std::vector<NodePtr> chain;  // 自己 … 根
    for (NodePtr node = shared_from_this(); node != nullptr; node = node->parent()) {
        chain.push_back(node);
    }
    const Node& top = *chain.back();
    if (!top.root_ || top.router_ == nullptr) return nullptr;

    auto resolved = std::make_unique<Resolved>();
    resolved->router = top.router_.get();
    resolved->root = &top;
    resolved->plain = top.plain_ && chain.size() == 1;
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        std::lock_guard lock((*it)->mutex_);
        append(resolved->source, (*it)->display_segment_, " › ");
        append(resolved->key, (*it)->key_segment_, ".");
    }
    resolved->module = top.display_segment_;
    std::tie(resolved->console, resolved->file) = resolved->router->thresholds(resolved->key);
    resolved->gate = std::min(resolved->console, resolved->file);

    std::lock_guard lock(mutex_);
    if (cache_owner_ == nullptr) {
        cache_owner_ = std::move(resolved);
        cache_.store(cache_owner_.get(), std::memory_order_release);
    }
    return cache_owner_.get();
}

void Node::adopt(const NodePtr& parent, std::string key_segment, std::string_view display) {
    if (root_ || muted_ || parent == nullptr || parent.get() == this) return;
    std::lock_guard lock(mutex_);
    if (parent_ != nullptr) return;
    parent_ = parent;
    key_segment_ = std::move(key_segment);
    if (display_segment_.empty()) display_segment_ = std::string(display);
}

void Node::setFrame(const Frame& frame) noexcept {
    if (!root_) return;
    sequence_.store(frame.sequence, std::memory_order_relaxed);
    data_time_.store(frame.data_time_ns, std::memory_order_relaxed);
    session_.store(frame.session, std::memory_order_relaxed);
    revision_.store(frame.parameter_revision, std::memory_order_relaxed);
    has_frame_.store(true, std::memory_order_release);
}

std::optional<Frame> Node::frame() const noexcept {
    if (!has_frame_.load(std::memory_order_acquire)) return std::nullopt;
    return Frame{.sequence = sequence_.load(std::memory_order_relaxed),
                 .data_time_ns = data_time_.load(std::memory_order_relaxed),
                 .session = session_.load(std::memory_order_relaxed),
                 .parameter_revision = revision_.load(std::memory_order_relaxed)};
}

std::optional<std::uint64_t> Node::admit(const std::source_location& site,
                                         std::chrono::nanoseconds period, std::int64_t now) {
    std::lock_guard lock(throttle_mutex_);
    const auto [it, first] =
        throttles_.try_emplace({site.file_name(), site.line()}, Throttle{.last = now, .merged = 0});
    if (first) return 0;
    if (now - it->second.last < period.count()) {
        ++it->second.merged;
        return std::nullopt;
    }
    const std::uint64_t merged = it->second.merged;
    it->second = Throttle{.last = now, .merged = 0};
    return merged;
}

std::string Node::sourceText() const {
    std::vector<NodePtr> hold;  // 走到头之前别让父节点被释放
    for (NodePtr node = std::const_pointer_cast<Node>(shared_from_this()); node != nullptr;
         node = node->parent()) {
        hold.push_back(node);
    }
    std::string out;
    for (auto it = hold.rbegin(); it != hold.rend(); ++it) {
        std::lock_guard lock((*it)->mutex_);
        append(out, (*it)->display_segment_, " › ");
    }
    return out;
}

std::string Node::keyText() const {
    std::vector<NodePtr> hold;
    for (NodePtr node = std::const_pointer_cast<Node>(shared_from_this()); node != nullptr;
         node = node->parent()) {
        hold.push_back(node);
    }
    std::string out;
    for (auto it = hold.rbegin(); it != hold.rend(); ++it) {
        std::lock_guard lock((*it)->mutex_);
        append(out, (*it)->key_segment_, ".");
    }
    return out;
}

NodePtr componentNode(const NodePtr& parent, std::string_view field, std::string_view type) {
    Node::Init init;
    init.parent = parent;
    init.muted = parent != nullptr && parent->muted();
    init.key_segment = std::string(field.empty() ? type : field);
    const bool top_level = parent == nullptr || parent->isRoot();
    init.display_segment =
        top_level || field.empty() ? std::string(type) : std::format("{}({})", field, type);
    return std::make_shared<Node>(std::move(init));
}

NodePtr nodeOf(const Logger& logger) noexcept { return Access::node(logger); }

ConstructionScope::ConstructionScope(NodePtr node) noexcept
    : node_(std::move(node)), outer_(tls_scope) {
    tls_scope = this;
}

ConstructionScope::~ConstructionScope() { tls_scope = outer_; }

NodePtr ConstructionScope::current() noexcept {
    return tls_scope != nullptr ? tls_scope->node_ : nullptr;
}

NodePtr ConstructionScope::take() {
    if (tls_scope == nullptr) return std::make_shared<Node>(Node::Init{});
    if (!tls_scope->taken_) {
        tls_scope->taken_ = true;
        return tls_scope->node_;
    }
    Node::Init init;
    init.parent = tls_scope->node_;
    init.muted = tls_scope->node_->muted();
    return std::make_shared<Node>(std::move(init));
}

namespace {
NodePtr mutedNode() {
    Node::Init init;
    init.muted = true;
    return std::make_shared<Node>(std::move(init));
}
}  // namespace

MutedConstruction::MutedConstruction() : scope_(mutedNode()) {}

}  // namespace detail

namespace {

void sendEvent(const detail::NodePtr& node, Event::Kind kind, std::string_view key,
               std::string_view detail, double value, std::string_view unit) {
    try {
        if (node->muted()) return;
        const detail::Resolved* resolved = node->resolved();
        if (resolved == nullptr) return;  // 没接上：没有诊断可交
        Event event;
        event.kind = kind;
        event.module = resolved->module;
        event.source = resolved->key;
        event.time_ns = resolved->router->now();
        event.frame = resolved->root->frame();
        event.key = std::string(key);
        event.detail = std::string(detail);
        event.value = value;
        event.unit = std::string(unit);
        resolved->router->submitEvent(event);
    } catch (...) {  // NOLINT(bugprone-empty-catch): 日志路径不得影响算法
    }
}

}  // namespace

Logger::Logger() : node_(std::make_shared<detail::Node>(detail::Node::Init{})) {}

Logger::Logger(detail::NodePtr node) noexcept : node_(std::move(node)) {}

bool Logger::enabled(Level level) const {
    if (node_->muted()) return false;
    if (const detail::Resolved* resolved = node_->resolved()) return level >= resolved->gate;
    return level >= Level::kInfo;
}

void Logger::write(Level level, const std::source_location& where, std::string text) const {
    try {
        if (node_->muted()) return;
        const detail::Resolved* resolved = node_->resolved();
        if (resolved == nullptr) {
            detail::writeDetached(level, node_->sourceText(), text);
            return;
        }
        Record record;
        record.level = level;
        record.time_ns = resolved->router->now();
        record.frame = resolved->root->frame();
        record.source = resolved->source;
        record.key = resolved->key;
        record.plain = resolved->plain;
        record.where = where;
        record.text = std::move(text);
        record.to_console = level >= resolved->console;
        record.to_file = level >= resolved->file;
        resolved->router->submit(std::move(record));
    } catch (...) {  // NOLINT(bugprone-empty-catch): 日志路径不得影响算法
    }
}

Throttled Logger::every(std::chrono::nanoseconds period,
                        std::source_location where) const noexcept {
    return Throttled(*this, period, where);
}

void Logger::event(std::string_view key, std::string_view detail) const {
    sendEvent(node_, Event::Kind::kEvent, key, detail, 0.0, {});
}

void Logger::metric(std::string_view key, double value, std::string_view unit) const {
    sendEvent(node_, Event::Kind::kMetric, key, {}, value, unit);
}

Timer Logger::timer(std::string_view step) const {
    return Timer(*this, std::string(step) + ".duration");
}

void Logger::setFrame(const Frame& frame) const noexcept { node_->setFrame(frame); }

std::string Logger::source() const {
    if (const detail::Resolved* resolved = node_->resolved()) return resolved->source;
    return node_->sourceText();
}

std::string Logger::key() const {
    if (const detail::Resolved* resolved = node_->resolved()) return resolved->key;
    return node_->keyText();
}

std::optional<std::uint64_t> Throttled::admit() const {
    const detail::NodePtr& node = detail::Access::node(logger_);
    const detail::Resolved* resolved = node->resolved();
    const std::int64_t now = resolved != nullptr ? resolved->router->now() : detail::steadyNow();
    return node->admit(site_, period_, now);
}

Timer::Timer(Logger logger, std::string key) noexcept
    : logger_(std::move(logger)), key_(std::move(key)), start_(std::chrono::steady_clock::now()) {}

Timer::Timer(Timer&& other) noexcept
    : logger_(std::move(other.logger_)), key_(std::move(other.key_)), start_(other.start_),
      active_(other.active_) {
    other.active_ = false;
}

Timer::~Timer() {
    if (!active_) return;
    const std::chrono::duration<double, std::milli> elapsed =
        std::chrono::steady_clock::now() - start_;
    logger_.metric(key_, elapsed.count(), "ms");
}

Loggable::Loggable() : log_(detail::Access::make(detail::ConstructionScope::take())) {}

void adopt(const Loggable& component, const Logger& parent, std::string_view field,
           std::string_view display) {
    detail::Access::node(detail::Access::log(component))
        ->adopt(detail::Access::node(parent), std::string(field), display);
}

}  // namespace simpleslam::logging
