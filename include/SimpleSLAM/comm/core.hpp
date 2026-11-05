// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file core.hpp
/// 运行时内核：按名字登记话题与服务，持有时钟、定时器、参数服务器和上报出口。
/// 模块只认内核，Runtime 负责装配、驱动与关闭。句柄只弱引用内核。

#include <SimpleSLAM/comm/clock.hpp>
#include <SimpleSLAM/comm/mailbox.hpp>
#include <SimpleSLAM/comm/parameters.hpp>
#include <SimpleSLAM/comm/service.hpp>
#include <SimpleSLAM/comm/topic.hpp>
#include <SimpleSLAM/comm/types.hpp>

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <typeindex>
#include <vector>

namespace simpleslam::comm {

class RuntimeCore : public std::enable_shared_from_this<RuntimeCore> {
public:
    RuntimeCore(std::unique_ptr<IClock> clock, ErrorReporter reporter, std::size_t backlog_warn,
                ExceptionPolicy default_policy);

    [[nodiscard]] const std::shared_ptr<Hub>& hub() const noexcept { return hub_; }
    [[nodiscard]] IClock& clock() noexcept { return *clock_; }
    [[nodiscard]] ParameterStore& params() noexcept { return params_; }
    [[nodiscard]] const ErrorReporter& reporter() const noexcept { return reporter_; }
    [[nodiscard]] ExceptionPolicy defaultPolicy() const noexcept { return default_policy_; }

    /// 提交参数事务，并把本次快照按版本顺序排进每个模块的收件箱。
    ParamCommitResult setParams(const std::vector<ParamChange>& changes);
    void setParamBroadcast(std::function<void(const ParamSnapshotPtr&)> broadcast);

    /// 按名字取话题，不存在则创建。类型不符是装配错误。
    template <class T>
    std::shared_ptr<Topic> topic(const std::string& resolved) {
        return topicOf(resolved, typeid(T));
    }
    std::shared_ptr<Topic> topicOf(const std::string& resolved, std::type_index type);

    std::shared_ptr<ServiceEndpoint> addService(const std::string& resolved, std::type_index request,
                                                std::type_index response, MailboxPtr owner,
                                                ServiceEndpoint::Handler handler);
    /// 不存在返回空；类型不符是装配错误。
    std::shared_ptr<ServiceEndpoint> findService(const std::string& resolved, std::type_index request,
                                                 std::type_index response);

    [[nodiscard]] std::size_t publisherCount(const std::string& resolved) const;
    void setLogSink(LogSink sink);
    void setLogLevel(LogLevel level) noexcept { log_level_.store(level, std::memory_order_relaxed); }
    void log(LogLevel level, std::string_view module, std::string_view text);

    /// 只发无订、只订无发的名字。
    [[nodiscard]] std::vector<std::string> danglingTopics() const;

private:
    std::shared_ptr<Hub> hub_ = std::make_shared<Hub>();
    std::unique_ptr<IClock> clock_;
    ParameterStore params_;
    ErrorReporter reporter_;
    std::size_t backlog_warn_;
    ExceptionPolicy default_policy_;

    std::mutex log_mutex_;
    LogSink log_sink_;
    std::atomic<LogLevel> log_level_{LogLevel::kInfo};

    std::mutex param_mutex_;
    std::function<void(const ParamSnapshotPtr&)> param_broadcast_;

    mutable std::mutex mutex_;
    std::map<std::string, std::shared_ptr<Topic>> topics_;
    std::map<std::string, std::shared_ptr<ServiceEndpoint>> services_;
};

}  // namespace simpleslam::comm
