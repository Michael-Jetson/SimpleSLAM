// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file core.hpp
/// 运行时内核：按名字登记话题与服务，并持有时钟、定时器、参数存储与上报出口。
///
/// 单独成一层是为了打断 Module 与 Runtime 的相互依赖：模块只认内核，
/// Runtime 负责装配、驱动与关闭。同一进程可以有多个互不干扰的内核。

#include <SimpleSLAM/comm/clock.hpp>
#include <SimpleSLAM/comm/execution.hpp>
#include <SimpleSLAM/comm/parameters.hpp>
#include <SimpleSLAM/comm/service.hpp>
#include <SimpleSLAM/comm/topic.hpp>
#include <SimpleSLAM/comm/types.hpp>

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace simpleslam::comm {

/// 装配期错误：名字非法、类型不符、服务重名。一律 fail-fast。
class WiringError : public std::runtime_error {
public:
    explicit WiringError(const std::string& what) : std::runtime_error(what) {}
};

/// 关闭话题时的范围。见 RuntimeCore::closeTopics()。
enum class TopicCloseScope : std::uint8_t {
    kIngress,  ///< 只关外部输入的入口话题
    kAll,      ///< 全关
};

/// 内核由 Runtime 共享持有：句柄（ServiceClient 的端点解析）只弱引用它，Runtime 销毁之后
/// 解析安全地返回"没有服务端"，而不是解引用已释放的内核。
class RuntimeCore : public std::enable_shared_from_this<RuntimeCore> {
public:
    RuntimeCore(IoContext& io, IClock& clock, std::shared_ptr<TimerQueue> timers,
                ParameterStore& params, ErrorReporter reporter)
        : io_(&io),
          clock_(&clock),
          timers_(std::move(timers)),
          params_(&params),
          reporter_(std::move(reporter)) {}

    RuntimeCore(const RuntimeCore&) = delete;
    RuntimeCore& operator=(const RuntimeCore&) = delete;

    [[nodiscard]] IoContext& io() noexcept { return *io_; }
    [[nodiscard]] IClock& clock() noexcept { return *clock_; }
    [[nodiscard]] TimerQueue& timers() noexcept { return *timers_; }
    /// 定时器队列的弱引用。交给可能比 Runtime 活得久的句柄（TimerHandle、服务调用端）。
    [[nodiscard]] std::weak_ptr<TimerQueue> timerLink() const noexcept { return timers_; }
    [[nodiscard]] ParameterStore& params() noexcept { return *params_; }
    [[nodiscard]] const ErrorReporter& reporter() const noexcept { return reporter_; }
    /// 停止读取新的外部输入；已接纳工作的派生消息仍可继续投递。
    void stopInputs() noexcept { inputs_stopped_.store(true, std::memory_order_release); }
    [[nodiscard]] bool inputsStopped() const noexcept {
        return inputs_stopped_.load(std::memory_order_acquire);
    }

    /// 按已解析的名字取话题，不存在则创建。类型不符是装配期错误。
    template <class T>
    std::shared_ptr<Topic<T>> topic(const std::string& resolved) {
        if (!isValidResolvedName(resolved)) {
            throw WiringError("话题名非法: " + resolved);
        }
        std::lock_guard lock(mutex_);
        auto it = topics_.find(resolved);
        if (it == topics_.end()) {
            auto created = std::make_shared<Topic<T>>(resolved, reporter_);
            topics_.emplace(resolved, created);
            return created;
        }
        if (it->second->messageType() != std::type_index(typeid(T))) {
            throw WiringError("话题 " + resolved + " 的消息类型与既有登记不符");
        }
        return std::static_pointer_cast<Topic<T>>(it->second);
    }

    /// 登记服务端。同名只允许一个。
    template <class Request, class Response>
    std::shared_ptr<Service<Request, Response>> registerService(
        const std::string& resolved, MailboxPtr owner,
        typename Service<Request, Response>::Handler handler, std::size_t max_in_flight) {
        if (!isValidResolvedName(resolved)) {
            throw WiringError("服务名非法: " + resolved);
        }
        auto service = std::make_shared<Service<Request, Response>>(
            resolved, std::move(owner), std::move(handler), max_in_flight, reporter_);
        std::lock_guard lock(mutex_);
        if (services_.count(resolved) != 0) {
            throw WiringError("服务重名: " + resolved);
        }
        services_.emplace(resolved, service);
        return service;
    }

    /// 查找服务端。不存在返回空；存在但类型不符是装配期错误。
    template <class Request, class Response>
    std::weak_ptr<Service<Request, Response>> findService(const std::string& resolved) {
        std::lock_guard lock(mutex_);
        auto it = services_.find(resolved);
        if (it == services_.end()) return {};
        if (it->second->requestType() != std::type_index(typeid(Request))
            || it->second->responseType() != std::type_index(typeid(Response))) {
            throw WiringError("服务 " + resolved + " 的请求/响应类型与既有登记不符");
        }
        return std::static_pointer_cast<Service<Request, Response>>(it->second);
    }

    /// 只发无订、只订无发的名字。用于装配后的接线诊断，不替代必需依赖的硬校验。
    [[nodiscard]] std::vector<std::string> danglingTopics() const {
        std::vector<std::string> dangling;
        std::lock_guard lock(mutex_);
        for (const auto& [name, topic] : topics_) {
            const bool no_sub = topic->subscriberCount() == 0;
            const bool no_pub = topic->publisherCount() == 0;
            if (no_sub != no_pub) dangling.push_back(name);
        }
        return dangling;
    }

    [[nodiscard]] std::size_t topicCount() const {
        std::lock_guard lock(mutex_);
        return topics_.size();
    }

    /// 关闭全部服务端，拒收新请求。
    void closeServices() {
        std::lock_guard lock(mutex_);
        for (auto& [name, service] : services_) {
            service->close();
        }
    }

    /// 关闭话题的接纳口。已接纳的消息仍会被处理完，这是"停输入"不是"销毁"。
    ///
    /// 分两段是刻意的：先关入口话题让外部数据停止
    /// 进入，派生话题留着，于是已接纳的那些包能把"里程计 → 关键帧 → 图 → 轨迹"
    /// 整条因果链走完；等因果链排空之后再 kAll 关掉其余话题。一次性全关会让
    /// 最终图与轨迹静悄悄地少一截，而精度门禁正是拿它们做结论的。
    void closeTopics(TopicCloseScope scope = TopicCloseScope::kAll) {
        std::lock_guard lock(mutex_);
        for (auto& [name, topic] : topics_) {
            if (scope == TopicCloseScope::kIngress && !topic->ingress()) continue;
            topic->close();
        }
    }

    /// 入口话题的数量。零表示没人声明入口，此时两段关闭退化成一次性全关。
    [[nodiscard]] std::size_t ingressTopicCount() const {
        std::lock_guard lock(mutex_);
        std::size_t count = 0;
        for (const auto& [name, topic] : topics_) {
            if (topic->ingress()) ++count;
        }
        return count;
    }

private:
    IoContext* io_;
    IClock* clock_;
    std::shared_ptr<TimerQueue> timers_;
    ParameterStore* params_;
    ErrorReporter reporter_;
    std::atomic<bool> inputs_stopped_{false};

    mutable std::mutex mutex_;
    std::map<std::string, std::shared_ptr<TopicBase>> topics_;
    std::map<std::string, std::shared_ptr<ServiceBase>> services_;
};

}  // namespace simpleslam::comm
