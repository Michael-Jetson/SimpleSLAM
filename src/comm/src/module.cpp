// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/comm/module.hpp>

#include <exception>
#include <functional>
#include <utility>

namespace simpleslam::comm {

void Module::attach(RuntimeCore& core, std::string resolved_name) {
    if (!isValidResolvedName(resolved_name)) throw WiringError("模块名必须是合法绝对名: " + resolved_name);
    core_ = &core;
    name_ = std::move(resolved_name);
    mailbox_ = std::make_shared<Mailbox>(name_, core.hub(), core.reporter(), core.defaultPolicy());
    mailbox_->setCallbackScope(this);
    params_ = core.params().snapshot();
    log_ = core.logRouter().root(name_);
}

void Module::enterCallback() noexcept { param_scope_.emplace(&params_, &plugin_writes_); }

void Module::leaveCallback(bool completed) noexcept {
    param_scope_.reset();
    if (plugin_writes_.empty()) return;
    const std::vector<ParamChange> writes = std::exchange(plugin_writes_, {});
    if (!completed) return;  // 回调出错：这次回调里记下的写不提交
    try {
        const auto result = core_->setParams(writes, name_);
        if (!result.ok()) log_.warn("插件写的参数没有生效：{}", result.detail);
    } catch (const std::exception& e) {
        detail::reportSafely(core_->reporter(), name_, e.what());
    } catch (...) {
        detail::reportSafely(core_->reporter(), name_, "提交插件写的参数时抛出未知异常");
    }
}

void Module::initialize() {
    params_ = core_->params().snapshot();
    enterCallback();
    try {
        onInit();
    } catch (...) {
        leaveCallback(false);
        throw;
    }
    leaveCallback(true);
    initialized_ = true;
}

template <class F>
void Module::runHook(F&& hook) noexcept {
    detail::RuntimeThreadMark mark;
    enterCallback();
    bool completed = false;
    try {
        hook();
        completed = true;
    } catch (const std::exception& e) {
        hook_failures_.fetch_add(1, std::memory_order_relaxed);
        detail::reportSafely(core_->reporter(), name_, e.what());
        core_->hub()->recordFailure({name_, e.what(), std::current_exception()}, false);
    } catch (...) {
        hook_failures_.fetch_add(1, std::memory_order_relaxed);
        detail::reportSafely(core_->reporter(), name_, "模块钩子抛出未知异常");
    }
    leaveCallback(completed);
}

void Module::start() {
    if (started_ || !initialized_ || stopped_) return;
    started_ = true;
    runHook([this] { onStart(); });
}

void Module::remap(std::string_view from, std::string_view to) {
    remaps_[resolveName(from, name_)] = resolveName(to, name_);
}

std::string Module::resolveEndpoint(std::string_view name) const {
    std::string resolved = resolveName(name, name_);
    if (auto it = remaps_.find(resolved); it != remaps_.end()) return it->second;
    return resolved;
}

void Module::finish() {
    if (finished_ || stopped_ || !initialized_) return;
    finished_ = true;
    runHook([this] { onFinish(); });
}

void Module::shutdown() {
    if (stopped_) return;
    stopped_ = true;
    for (auto& sub : subscriptions_) sub->close();
    for (const auto& timer : timers_) timer.cancel();
    if (initialized_) runHook([this] { onShutdown(); });
    mailbox_->close();
}

void Module::adoptParams(ParamSnapshotPtr snapshot) {
    params_ = std::move(snapshot);
    onParamsChanged(*params_);
}

ServiceCaller Module::makeCaller(RuntimeCore& core, std::string resolved, std::type_index request,
                                 std::type_index response, MailboxPtr caller) {
    std::weak_ptr<RuntimeCore> weak = core.weak_from_this();
    return ServiceCaller(
        [weak, resolved = std::move(resolved), request, response]() -> std::shared_ptr<ServiceEndpoint> {
            auto alive = weak.lock();
            return alive ? alive->findService(resolved, request, response) : nullptr;
        },
        std::move(caller), core.hub(), &core.clock());
}

TimerHandle Module::createTimer(Duration period, std::function<void()> cb) {
    return scheduleTimer(period, period, std::move(cb));
}

TimerHandle Module::createOneShot(Duration delay, std::function<void()> cb) {
    return scheduleTimer(delay, Duration{0}, std::move(cb));
}

TimerHandle Module::scheduleTimer(Duration delay, Duration period, std::function<void()> cb) {
    auto alive = std::make_shared<std::atomic<bool>>(true);
    auto queued = std::make_shared<std::atomic<bool>>(false);
    auto body = std::make_shared<std::function<void()>>(std::move(cb));
    TimerHandle handle = mailbox_->scheduleTimer(
        core_->clock().now() + delay, period,
        [box = mailbox_, alive, queued, body] {
            if (queued->exchange(true, std::memory_order_acq_rel)) return;
            const bool posted = box->post([alive, queued, body] {
                queued->store(false, std::memory_order_release);
                if (alive->load(std::memory_order_acquire)) (*body)();
            });
            if (!posted) queued->store(false, std::memory_order_release);
        },
        alive);
    timers_.push_back(handle);
    return handle;
}

}  // namespace simpleslam::comm
