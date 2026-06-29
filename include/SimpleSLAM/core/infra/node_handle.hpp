#pragma once

/// @file node_handle.hpp
/// 类 ROS1 NodeHandle —— 对 TopicHub + ServiceRegistry 的薄封装，给 ROS1 API 手感。
///
/// 进程内、零序列化。spin/spinOnce = drainAll。可绑定隔离实例（测试）或进程级单例。
/// 这是可选门面层；底层 TopicHub/ServiceRegistry 仍可直接用。

#include <SimpleSLAM/core/infra/service.hpp>
#include <SimpleSLAM/core/infra/topic_hub.hpp>

#include <chrono>
#include <functional>
#include <string_view>
#include <thread>
#include <utility>

namespace simpleslam {

class NodeHandle {
public:
    /// 默认绑定进程级单例；测试可传入隔离实例。
    explicit NodeHandle(TopicHub* hub = &TopicHub::instance(),
                        ServiceRegistry* services = &ServiceRegistry::instance())
        : hub_(hub), services_(services) {}

    // ── 话题 ──

    template <class T>
    Publisher<T> advertise(std::string_view name, QoS qos = QoS::Event) {
        return hub_->createPublisherImpl<T>(name, qos);
    }

    /// 订阅：lambda / 自由函数 / std::function
    template <class T, class Callable>
    SubscriptionHandle subscribe(std::string_view name, Callable&& cb,
                                 SubscribeOptions opts = {}) {
        return hub_->subscribeImpl<T>(name, std::forward<Callable>(cb), opts);
    }

    /// 订阅：成员函数 void(Obj::*)(const T&)
    template <class T, class Obj>
    SubscriptionHandle subscribe(std::string_view name, void (Obj::*method)(const T&),
                                 Obj* obj, SubscribeOptions opts = {}) {
        return hub_->subscribeImpl<T>(name, method, obj, opts);
    }

    /// 订阅：成员函数 void(Obj::*)(MsgPtr<T>)
    template <class T, class Obj>
    SubscriptionHandle subscribe(std::string_view name, void (Obj::*method)(MsgPtr<T>),
                                 Obj* obj, SubscribeOptions opts = {}) {
        return hub_->subscribeImpl<T>(name, method, obj, opts);
    }

    template <class T>
    MsgPtr<T> getLatest(std::string_view name) {
        return hub_->getLatestImpl<T>(name);
    }

    // ── spin（= drainAll）──

    /// 处理一轮 pending（确定性 BFS）。返回处理的消息数。等价 ROS spinOnce。
    size_t spinOnce() { return hub_->drainAll(); }

    /// 循环 spinOnce 直到 keep_running() 返回 false。无 pending 时让出 CPU。
    void spin(const std::function<bool()>& keep_running) {
        while (keep_running()) {
            if (spinOnce() == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    // ── 服务 ──

    template <class Req, class Resp>
    ServiceServer advertiseService(std::string_view name,
                                   std::function<Resp(const Req&)> handler) {
        return services_->advertiseService<Req, Resp>(name, std::move(handler));
    }

    template <class Req, class Resp>
    ServiceClient<Req, Resp> serviceClient(std::string_view name) {
        return ServiceClient<Req, Resp>(std::string(name), services_);
    }

    template <class Req, class Resp>
    Resp callService(std::string_view name, const Req& req) {
        return services_->call<Req, Resp>(name, req);
    }

    [[nodiscard]] TopicHub& topics() { return *hub_; }
    [[nodiscard]] ServiceRegistry& services() { return *services_; }

private:
    TopicHub* hub_;
    ServiceRegistry* services_;
};

}  // namespace simpleslam
