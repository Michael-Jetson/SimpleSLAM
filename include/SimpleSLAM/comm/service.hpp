// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file service.hpp
/// 服务：一请求一响应，同名只有一个服务端。处理在服务端的收件箱里排队，响应回到调用方的收件箱。
/// 实现类型无关，在 src/comm/src/service.cpp 里；ServiceClient<Req, Res> 只是薄的类型层。
///
/// 受理之后恰好结案一次：响应、超时或停止，先到的那一路赢。服务端任务没执行就被销毁时，
/// 析构以 kStopped 结案，所以无超时的调用也不会永远等下去。

#include <SimpleSLAM/comm/clock.hpp>
#include <SimpleSLAM/comm/mailbox.hpp>
#include <SimpleSLAM/comm/types.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <utility>

namespace simpleslam::comm {

class ServiceEndpoint final : public std::enable_shared_from_this<ServiceEndpoint> {
public:
    using Handler = std::function<AnyMsg(const AnyMsg&)>;
    /// 状态、响应、失败原因。
    using Done = std::function<void(ServiceStatus, AnyMsg, std::string)>;

    ServiceEndpoint(std::string name, std::type_index request, std::type_index response, MailboxPtr owner,
                    Handler handler, ErrorReporter reporter);

    [[nodiscard]] std::type_index requestType() const noexcept { return request_; }
    [[nodiscard]] std::type_index responseType() const noexcept { return response_; }
    void invoke(AnyMsg request, Done done);
    void close() noexcept { closed_.store(true, std::memory_order_release); }
    [[nodiscard]] bool closed() const noexcept { return closed_.load(std::memory_order_acquire); }

private:
    std::atomic<bool> closed_{false};
    std::string name_;
    std::type_index request_;
    std::type_index response_;
    MailboxPtr owner_;
    Handler handler_;
    ErrorReporter reporter_;
};

/// 服务端对象。服务由运行时内核持有，所以默认不需要保存，丢掉它也不会下线；要提前下线时才保存。
class ServiceServer {
public:
    ServiceServer() = default;
    explicit ServiceServer(std::shared_ptr<ServiceEndpoint> impl) : impl_(std::move(impl)) {}

    /// 下线：新调用得到 kNoProvider，已排队没执行的结案为 kStopped。名字可以重新登记。
    void shutdown() const {
        if (impl_) impl_->close();
    }
    [[nodiscard]] bool active() const { return impl_ && !impl_->closed(); }

private:
    std::shared_ptr<ServiceEndpoint> impl_;
};

/// 类型无关的调用端。
class ServiceCaller {
public:
    using Lookup = std::function<std::shared_ptr<ServiceEndpoint>()>;

    ServiceCaller() = default;
    /// caller 为空时只能用阻塞 call()：外部线程调用服务不需要收件箱。
    ServiceCaller(Lookup lookup, MailboxPtr caller, std::shared_ptr<Hub> hub, IClock* clock);

    [[nodiscard]] ServiceStatus asyncCall(AnyMsg request, Duration timeout, ServiceEndpoint::Done done);
    /// timeout 为零表示一直等。
    void call(AnyMsg request, Duration timeout, ServiceStatus& status, AnyMsg& value, std::string& error);

private:
    [[nodiscard]] bool stopped() const;

    Lookup lookup_;
    MailboxPtr caller_;
    std::shared_ptr<Hub> hub_;
    IClock* clock_ = nullptr;
};

template <class Request, class Response>
class ServiceClient {
public:
    using Completion = std::function<void(ServiceResult<Response>)>;

    ServiceClient() = default;
    explicit ServiceClient(ServiceCaller caller) : caller_(std::move(caller)) {}

    /// 返回受理结果。kOk 表示已受理，此后恰好结案一次，完成回调在调用方的线程上执行。
    /// 其余返回值都表示没有受理，不会有回调。
    [[nodiscard]] ServiceStatus asyncCall(Request request, Duration timeout, Completion completion) {
        return caller_.asyncCall(std::make_shared<const Request>(std::move(request)), timeout,
                                 [completion = std::move(completion)](ServiceStatus s, AnyMsg v, std::string e) {
                                     if (completion) completion(result(s, std::move(v), std::move(e)));
                                 });
    }

    /// 阻塞调用。只给运行时之外的线程用，在运行时线程内返回 kWouldDeadlock。
    ServiceResult<Response> call(Request request, Duration timeout) {
        ServiceStatus status = ServiceStatus::kNoProvider;
        AnyMsg value;
        std::string error;
        caller_.call(std::make_shared<const Request>(std::move(request)), timeout, status, value, error);
        return result(status, std::move(value), std::move(error));
    }

private:
    static ServiceResult<Response> result(ServiceStatus status, AnyMsg value, std::string error) {
        return {status, std::static_pointer_cast<const Response>(std::move(value)), std::move(error)};
    }
    ServiceCaller caller_;
};

}  // namespace simpleslam::comm
