// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/comm/core.hpp>

#include <cstdio>
#include <utility>

namespace simpleslam::comm {

namespace {

std::shared_ptr<logging::Router> synchronousRouter() {
    logging::LogOptions options;
    options.asynchronous = false;  // 通信层除了模块线程不另开线程
    return logging::Router::create(std::move(options));
}

}  // namespace

RuntimeCore::RuntimeCore(std::unique_ptr<IClock> clock, ErrorReporter reporter, std::size_t backlog_warn,
                         ExceptionPolicy default_policy,
                         std::shared_ptr<logging::Router> log_router)
    : clock_(std::move(clock)),
      reporter_(std::move(reporter)),
      backlog_warn_(backlog_warn),
      default_policy_(default_policy),
      log_router_(log_router != nullptr ? std::move(log_router) : synchronousRouter()) {
    // 时钟交给日志组件一份：日志组件可能比运行时活得长
    log_router_->setClock([clock = clock_] { return clock->now().count(); });
}

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

ParamCommitResult RuntimeCore::setParams(const std::vector<ParamChange>& changes,
                                         std::string_view writer) {
    std::lock_guard lock(param_mutex_);  // 提交与分发是一串：快照按版本顺序排进收件箱
    if (hub_->inputsStopped()) {
        const auto current = params_.snapshot();
        return {.status = ParamCommit::kRejectedStopped,
                .revision = current->revision,
                .detail = "运行时已停止输入",
                .snapshot = nullptr,
                .previous = current};
    }
    auto result = params_.commit(changes, writer);
    if (!result.ok() || !result.snapshot) return result;
    // 运行中的修改写进日志：任务记录里的 task.yaml 只有初值，变化看 run.log
    const logging::Logger log = log_router_->root("/runtime");
    for (const auto& change : changes) {
        const auto& before = result.previous->values;
        const auto old = before.find(change.name);
        log.info("参数 {}：{} → {}（第 {} 版，{}）", change.name,
                 old == before.end() ? std::string("（无）") : paramText(old->second),
                 paramText(result.snapshot->values.at(change.name)), result.revision, writer);
    }
    if (param_broadcast_) param_broadcast_(result.snapshot);
    return result;
}

void RuntimeCore::setParamBroadcast(std::function<void(const ParamSnapshotPtr&)> broadcast) {
    std::lock_guard lock(param_mutex_);
    param_broadcast_ = std::move(broadcast);
}

}  // namespace simpleslam::comm
