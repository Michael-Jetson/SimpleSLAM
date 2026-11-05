// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/comm/service.hpp>

#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <utility>

namespace simpleslam::comm {

namespace {

/// 服务端收下、尚未执行的一次请求。任务没执行就被销毁时，析构以 kStopped 结案。
struct ServiceJob {
    explicit ServiceJob(ServiceEndpoint::Done callback) : done(std::move(callback)) {}
    ServiceJob(const ServiceJob&) = delete;
    ServiceJob& operator=(const ServiceJob&) = delete;
    ~ServiceJob() {
        if (finished) return;
        try {
            done(ServiceStatus::kStopped, nullptr, {});
        } catch (...) {  // NOLINT(bugprone-empty-catch)
        }
    }
    void finish(ServiceStatus status, AnyMsg value, std::string error) {
        finished = true;
        done(status, std::move(value), std::move(error));
    }
    ServiceEndpoint::Done done;
    bool finished = false;
};

/// 一次已受理的异步调用。结案可能来自服务端的线程，也可能来自定时器。
struct PendingCall {
    ServiceEndpoint::Done done;
    MailboxPtr caller;
    TimerHandle timer;
    std::atomic<bool> settled{false};

    void settle(ServiceStatus status, AnyMsg value, std::string error) {
        if (settled.exchange(true, std::memory_order_acq_rel)) return;
        timer.cancel();
        caller->post([callback = std::move(done), status, value = std::move(value), error = std::move(error)] {
            callback(status, value, error);
        });
    }
};

struct BlockingCall {
    std::mutex mutex;
    std::condition_variable cv;
    bool done = false;
    ServiceStatus status = ServiceStatus::kNoProvider;
    AnyMsg value;
    std::string error;
};

}  // namespace

ServiceEndpoint::ServiceEndpoint(std::string name, std::type_index request, std::type_index response,
                                 MailboxPtr owner, Handler handler, ErrorReporter reporter)
    : name_(std::move(name)),
      request_(request),
      response_(response),
      owner_(std::move(owner)),
      handler_(std::move(handler)),
      reporter_(std::move(reporter)) {}

void ServiceEndpoint::invoke(AnyMsg request, Done done) {
    auto job = std::make_shared<ServiceJob>(std::move(done));
    owner_->post([self = shared_from_this(), job, request = std::move(request)] {
        if (self->closed()) return;  // job 析构时以 kStopped 结案
        ServiceStatus status = ServiceStatus::kOk;
        AnyMsg value;
        std::string error;
        try {
            value = self->handler_(request);
        } catch (const std::exception& e) {
            status = ServiceStatus::kHandlerError;
            error = e.what();
        } catch (...) {
            status = ServiceStatus::kHandlerError;
            error = "服务处理函数抛出未知异常";
        }
        if (!error.empty()) detail::reportSafely(self->reporter_, self->name_, error);
        job->finish(status, std::move(value), std::move(error));
    });
}

ServiceCaller::ServiceCaller(Lookup lookup, MailboxPtr caller, std::shared_ptr<Hub> hub, IClock* clock)
    : lookup_(std::move(lookup)), caller_(std::move(caller)), hub_(std::move(hub)), clock_(clock) {}

bool ServiceCaller::stopped() const {
    return !hub_ || hub_->inputsStopped() || (caller_ && caller_->closed());
}

ServiceStatus ServiceCaller::asyncCall(AnyMsg request, Duration timeout, ServiceEndpoint::Done done) {
    if (stopped() || !caller_) return ServiceStatus::kStopped;
    auto service = lookup_ ? lookup_() : nullptr;
    if (!service || service->closed()) return ServiceStatus::kNoProvider;

    auto pending = std::make_shared<PendingCall>();
    pending->done = std::move(done);
    pending->caller = caller_;
    if (timeout.count() > 0) {
        // 句柄先建好再登记：定时器一登记就可能在模块线程上触发，结案时要用这个句柄
        auto alive = std::make_shared<std::atomic<bool>>(true);
        pending->timer = TimerHandle(alive);
        caller_->scheduleTimer(clock_->now() + timeout, Duration{0}, [pending] {
            pending->settle(ServiceStatus::kTimeout, nullptr, {});
        }, std::move(alive));
    }
    service->invoke(std::move(request), [pending](ServiceStatus status, AnyMsg value, std::string error) {
        pending->settle(status, std::move(value), std::move(error));
    });
    return ServiceStatus::kOk;
}

void ServiceCaller::call(AnyMsg request, Duration timeout, ServiceStatus& status, AnyMsg& value,
                         std::string& error) {
    if (inRuntimeThread()) {
        status = ServiceStatus::kWouldDeadlock;
        return;
    }
    if (stopped()) {
        status = ServiceStatus::kStopped;
        return;
    }
    auto service = lookup_ ? lookup_() : nullptr;
    if (!service || service->closed()) {
        status = ServiceStatus::kNoProvider;
        return;
    }
    auto state = std::make_shared<BlockingCall>();
    service->invoke(std::move(request), [state](ServiceStatus s, AnyMsg v, std::string e) {
        {
            std::lock_guard lock(state->mutex);
            state->done = true;
            state->status = s;
            state->value = std::move(v);
            state->error = std::move(e);
        }
        state->cv.notify_all();
    });
    std::unique_lock lock(state->mutex);
    const auto finished = [&] { return state->done; };
    if (timeout.count() > 0) {
        if (!state->cv.wait_for(lock, timeout, finished)) {
            status = ServiceStatus::kTimeout;
            return;
        }
    } else {
        state->cv.wait(lock, finished);
    }
    status = state->status;
    value = std::move(state->value);
    error = std::move(state->error);
}

}  // namespace simpleslam::comm
