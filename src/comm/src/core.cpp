// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/comm/core.hpp>

#include <cstdio>
#include <utility>

namespace simpleslam::comm {

RuntimeCore::RuntimeCore(std::unique_ptr<IClock> clock, ErrorReporter reporter, std::size_t backlog_warn,
                         ExceptionPolicy default_policy)
    : clock_(std::move(clock)),
      reporter_(std::move(reporter)),
      backlog_warn_(backlog_warn),
      default_policy_(default_policy) {}

std::shared_ptr<Topic> RuntimeCore::topicOf(const std::string& resolved, std::type_index type) {
    if (!isValidResolvedName(resolved)) throw WiringError("话题名非法: " + resolved);
    std::lock_guard lock(mutex_);
    auto it = topics_.find(resolved);
    if (it == topics_.end()) {
        auto created = std::make_shared<Topic>(resolved, type, hub_, reporter_, backlog_warn_);
        topics_.emplace(resolved, created);
        return created;
    }
    if (it->second->messageType() != type) {
        throw WiringError("话题 " + resolved + " 的消息类型与既有登记不符");
    }
    return it->second;
}

std::shared_ptr<ServiceEndpoint> RuntimeCore::addService(const std::string& resolved, std::type_index request,
                                                         std::type_index response, MailboxPtr owner,
                                                         ServiceEndpoint::Handler handler) {
    if (!isValidResolvedName(resolved)) throw WiringError("服务名非法: " + resolved);
    auto service = std::make_shared<ServiceEndpoint>(resolved, request, response, std::move(owner),
                                                     std::move(handler), reporter_);
    std::lock_guard lock(mutex_);
    auto [it, inserted] = services_.emplace(resolved, service);
    if (!inserted) {
        if (!it->second->closed()) throw WiringError("服务重名: " + resolved);
        it->second = service;  // 旧的已经下线，名字可以重新登记
    }
    return service;
}

std::shared_ptr<ServiceEndpoint> RuntimeCore::findService(const std::string& resolved, std::type_index request,
                                                          std::type_index response) {
    std::lock_guard lock(mutex_);
    auto it = services_.find(resolved);
    if (it == services_.end()) return nullptr;
    if (it->second->requestType() != request || it->second->responseType() != response) {
        throw WiringError("服务 " + resolved + " 的请求/响应类型与既有登记不符");
    }
    return it->second;
}

ParamCommitResult RuntimeCore::setParams(const std::vector<ParamChange>& changes) {
    std::lock_guard lock(param_mutex_);  // 提交与分发是一串：快照按版本顺序排进收件箱
    if (hub_->inputsStopped()) {
        return {ParamCommit::kRejectedStopped, params_.snapshot()->revision, "运行时已停止输入", nullptr};
    }
    auto result = params_.commit(changes);
    if (result.ok() && param_broadcast_) param_broadcast_(result.snapshot);
    return result;
}

void RuntimeCore::setParamBroadcast(std::function<void(const ParamSnapshotPtr&)> broadcast) {
    std::lock_guard lock(param_mutex_);
    param_broadcast_ = std::move(broadcast);
}

std::size_t RuntimeCore::publisherCount(const std::string& resolved) const {
    std::lock_guard lock(mutex_);
    auto it = topics_.find(resolved);
    return it == topics_.end() ? 0U : it->second->publisherCount();
}

void RuntimeCore::setLogSink(LogSink sink) {
    std::lock_guard lock(log_mutex_);
    log_sink_ = std::move(sink);
}

void RuntimeCore::log(LogLevel level, std::string_view module, std::string_view text) {
    if (level < log_level_.load(std::memory_order_relaxed)) return;
    const std::int64_t now = clock_->now().count();
    LogSink sink;
    {
        std::lock_guard lock(log_mutex_);
        sink = log_sink_;
    }
    if (sink) {
        sink(level, module, now, text);
        return;
    }
    static constexpr const char* kNames[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    std::fprintf(stderr, "[%s] [%.3f] [%.*s] %.*s\n", kNames[static_cast<int>(level)], static_cast<double>(now) * 1e-9,
                 static_cast<int>(module.size()), module.data(), static_cast<int>(text.size()), text.data());
}

std::vector<std::string> RuntimeCore::danglingTopics() const {
    std::vector<std::string> dangling;
    std::lock_guard lock(mutex_);
    for (const auto& [name, topic] : topics_) {
        if ((topic->subscriberCount() == 0) != (topic->publisherCount() == 0)) dangling.push_back(name);
    }
    return dangling;
}

}  // namespace simpleslam::comm
