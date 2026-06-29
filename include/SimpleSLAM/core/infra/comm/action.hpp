#pragma once

/// @file action.hpp
/// 进程内 Action —— 长耗时、可取消、带进度反馈的目标（类 ROS action）。
///
/// 用于后端长任务：全局 PGO、回环检测、重定位、建图。与服务（同步短调用）不同，
/// Action 在 worker 线程异步执行，期间发反馈、可协作取消（ORB-SLAM3 mbAbortBA 模式）。
///
///   ActionServer：注册 execute(goal, ctx) -> Result，ctx 可发反馈/查取消
///   ActionClient：sendGoal(goal, onFeedback) -> GoalHandle{ cancel(), get(), done() }
///
/// 结果经 std::future 取回，handler 异常经 get() 透传。header-only。

#include <any>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>

namespace simpleslam {

/// 单个目标的共享状态：协作取消标志 + 反馈通道 + 结果 promise。
template <class Feedback, class Result>
struct GoalState {
    std::atomic<bool> cancel{false};
    std::atomic<bool> finished{false};
    std::function<void(const Feedback&)> on_feedback;
    std::promise<Result> result;
};

/// 传给 execute handler 的上下文（在 worker 线程上）：发反馈 + 查取消。
template <class Feedback, class Result>
class ActionContext {
public:
    explicit ActionContext(GoalState<Feedback, Result>* state) : state_(state) {}

    [[nodiscard]] bool isCancelRequested() const { return state_->cancel.load(); }

    void publishFeedback(const Feedback& fb) const {
        if (state_->on_feedback) state_->on_feedback(fb);  // 在 worker 线程触发
    }

private:
    GoalState<Feedback, Result>* state_;
};

/// 客户端持有的目标句柄：取消、查询、取结果。析构=请求取消并 join（jthread）。
template <class Feedback, class Result>
class GoalHandle {
public:
    GoalHandle() = default;
    GoalHandle(std::shared_ptr<GoalState<Feedback, Result>> state, std::jthread worker,
               std::future<Result> future)
        : state_(std::move(state)),
          worker_(std::move(worker)),
          future_(std::move(future)) {}

    ~GoalHandle() {
        if (state_) state_->cancel.store(true);  // 丢句柄=协作取消；worker_(jthread) 随后自动 join
    }

    GoalHandle(GoalHandle&&) noexcept = default;
    GoalHandle& operator=(GoalHandle&&) noexcept = default;
    GoalHandle(const GoalHandle&) = delete;
    GoalHandle& operator=(const GoalHandle&) = delete;

    /// 请求协作取消（handler 需轮询 ctx.isCancelRequested()）。
    void cancel() {
        if (state_) state_->cancel.store(true);
    }

    [[nodiscard]] bool done() const { return state_ && state_->finished.load(); }

    /// 阻塞取结果（handler 异常在此透传）。
    Result get() { return future_.get(); }

    template <class Rep, class Period>
    std::future_status waitFor(const std::chrono::duration<Rep, Period>& d) const {
        return future_.wait_for(d);
    }

    [[nodiscard]] bool valid() const { return future_.valid(); }

private:
    std::shared_ptr<GoalState<Feedback, Result>> state_;
    std::jthread worker_;
    std::future<Result> future_;
};

/// Action 服务端 RAII 句柄。
class ActionServerBase {
public:
    virtual ~ActionServerBase() = default;
};
using ActionServer = std::shared_ptr<ActionServerBase>;

/// 进程内 Action 注册表。可建隔离实例（测试），也有进程级单例。
class ActionRegistry {
public:
    ActionRegistry() = default;
    ActionRegistry(const ActionRegistry&) = delete;
    ActionRegistry& operator=(const ActionRegistry&) = delete;

    /// 注册 Action，返回 RAII 句柄。重复注册同名抛异常。
    template <class Goal, class Feedback, class Result>
    ActionServer advertiseActionImpl(
        std::string_view name,
        std::function<Result(const Goal&, ActionContext<Feedback, Result>&)> execute) {
        std::string key(name);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (actions_.count(key)) {
                throw std::runtime_error(
                    "ActionRegistry: action already advertised: " + key);
            }
            Entry entry;
            entry.goal_type = std::type_index(typeid(Goal));
            entry.feedback_type = std::type_index(typeid(Feedback));
            entry.result_type = std::type_index(typeid(Result));
            entry.execute = std::move(execute);
            actions_.emplace(key, std::move(entry));
        }
        auto* self = this;
        return std::make_shared<Server>([self, key]() { self->remove(key); });
    }

    /// 发送目标：worker 线程异步执行 execute，立即返回 GoalHandle。
    /// 无此 action / 类型不匹配抛异常。on_feedback 在 worker 线程上被调用。
    template <class Goal, class Feedback, class Result>
    GoalHandle<Feedback, Result> sendGoalImpl(
        std::string_view name, const Goal& goal,
        std::function<void(const Feedback&)> on_feedback = {}) {
        std::function<Result(const Goal&, ActionContext<Feedback, Result>&)> execute;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = actions_.find(std::string(name));
            if (it == actions_.end()) {
                throw std::runtime_error(
                    "ActionRegistry: no such action: " + std::string(name));
            }
            if (it->second.goal_type != std::type_index(typeid(Goal)) ||
                it->second.feedback_type != std::type_index(typeid(Feedback)) ||
                it->second.result_type != std::type_index(typeid(Result))) {
                throw std::runtime_error(
                    "ActionRegistry: type mismatch for action: " + std::string(name));
            }
            execute = std::any_cast<
                std::function<Result(const Goal&, ActionContext<Feedback, Result>&)>>(
                it->second.execute);
        }

        auto state = std::make_shared<GoalState<Feedback, Result>>();
        state->on_feedback = std::move(on_feedback);
        std::future<Result> future = state->result.get_future();

        std::jthread worker([execute, goal, state]() {
            ActionContext<Feedback, Result> ctx(state.get());
            try {
                state->result.set_value(execute(goal, ctx));
            } catch (...) {
                state->result.set_exception(std::current_exception());
            }
            state->finished.store(true);
        });

        return GoalHandle<Feedback, Result>(std::move(state), std::move(worker),
                                            std::move(future));
    }

    [[nodiscard]] bool hasActionImpl(std::string_view name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return actions_.count(std::string(name)) > 0;
    }

    static ActionRegistry& instance() {
        static ActionRegistry global;
        return global;
    }

    // ── 全局静态糖（用懒加载单例，免写 instance()）──

    template <class Goal, class Feedback, class Result>
    static ActionServer advertiseAction(
        std::string_view name,
        std::function<Result(const Goal&, ActionContext<Feedback, Result>&)> execute) {
        return instance().advertiseActionImpl<Goal, Feedback, Result>(
            name, std::move(execute));
    }
    template <class Goal, class Feedback, class Result>
    static GoalHandle<Feedback, Result> sendGoal(
        std::string_view name, const Goal& goal,
        std::function<void(const Feedback&)> on_feedback = {}) {
        return instance().sendGoalImpl<Goal, Feedback, Result>(name, goal,
                                                               std::move(on_feedback));
    }
    static bool hasAction(std::string_view name) {
        return instance().hasActionImpl(name);
    }

private:
    struct Entry {
        std::type_index goal_type{typeid(void)};
        std::type_index feedback_type{typeid(void)};
        std::type_index result_type{typeid(void)};
        std::any execute;  ///< std::function<Result(const Goal&, ActionContext&)>
    };

    class Server : public ActionServerBase {
    public:
        explicit Server(std::function<void()> unreg) : unreg_(std::move(unreg)) {}
        ~Server() override { unreg_(); }

    private:
        std::function<void()> unreg_;
    };

    void remove(const std::string& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        actions_.erase(key);
    }

    mutable std::mutex mutex_;
    std::unordered_map<std::string, Entry> actions_;
};

}  // namespace simpleslam
