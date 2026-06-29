#pragma once

/// @file service.hpp
/// 进程内 typed request/reply 服务（类 ROS service，但零序列化、零 codegen）。
///
/// 与话题（pub/sub，fire-and-forget、异常隔离）不同：服务是同步请求-应答，
/// handler 的异常**传播给调用者**——调用方需要知道服务是否失败。
///
/// 设计：ServiceRegistry 按名字注册 type-erased handler（std::any + type_index 守类型），
/// call 时锁外直调 handler，零拷贝（进程内活对象）。

#include <any>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>

namespace simpleslam {

/// 服务端 RAII 句柄：持有则服务有效，析构则自动注销（类 SubscriptionHandle）。
class ServiceServerBase {
public:
    virtual ~ServiceServerBase() = default;
};
using ServiceServer = std::shared_ptr<ServiceServerBase>;

/// 进程内服务注册表。可建隔离实例（测试），也有进程级单例 instance()。
class ServiceRegistry {
public:
    ServiceRegistry() = default;

    ServiceRegistry(const ServiceRegistry&) = delete;
    ServiceRegistry& operator=(const ServiceRegistry&) = delete;

    /// 注册服务（实例版，隔离用），返回 RAII 句柄。重复注册同名服务抛异常。
    template <class Req, class Resp>
    ServiceServer advertiseServiceImpl(std::string_view name,
                                       std::function<Resp(const Req&)> handler) {
        std::string key(name);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (services_.count(key)) {
                throw std::runtime_error(
                    "ServiceRegistry: service already advertised: " + key);
            }
            Entry entry;
            entry.req_type = std::type_index(typeid(Req));
            entry.resp_type = std::type_index(typeid(Resp));
            entry.handler = std::move(handler);
            services_.emplace(key, std::move(entry));
        }
        auto* self = this;
        return std::make_shared<Server>([self, key]() { self->remove(key); });
    }

    /// 同步调用服务（实例版）。无此服务 / 类型不匹配抛异常；handler 异常向上传播。
    template <class Req, class Resp>
    Resp callImpl(std::string_view name, const Req& req) {
        std::function<Resp(const Req&)> handler;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = services_.find(std::string(name));
            if (it == services_.end()) {
                throw std::runtime_error(
                    "ServiceRegistry: no such service: " + std::string(name));
            }
            if (it->second.req_type != std::type_index(typeid(Req)) ||
                it->second.resp_type != std::type_index(typeid(Resp))) {
                throw std::runtime_error(
                    "ServiceRegistry: type mismatch for service: " +
                    std::string(name));
            }
            handler =
                std::any_cast<std::function<Resp(const Req&)>>(it->second.handler);
        }
        // 锁外调用：长 handler 不占注册表锁，re-entrant 调用不死锁；异常透传给调用者。
        return handler(req);
    }

    [[nodiscard]] bool hasServiceImpl(std::string_view name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return services_.count(std::string(name)) > 0;
    }

    /// 进程级单例（懒加载：首次访问自动创建，无需 init）。
    static ServiceRegistry& instance() {
        static ServiceRegistry global;
        return global;
    }

    // ── 全局静态糖（用懒加载单例，免写 instance()）──

    template <class Req, class Resp>
    static ServiceServer advertiseService(std::string_view name,
                                          std::function<Resp(const Req&)> handler) {
        return instance().advertiseServiceImpl<Req, Resp>(name, std::move(handler));
    }
    template <class Req, class Resp>
    static Resp call(std::string_view name, const Req& req) {
        return instance().callImpl<Req, Resp>(name, req);
    }
    static bool hasService(std::string_view name) {
        return instance().hasServiceImpl(name);
    }

private:
    struct Entry {
        std::type_index req_type{typeid(void)};
        std::type_index resp_type{typeid(void)};
        std::any handler;  ///< std::function<Resp(const Req&)>
    };

    class Server : public ServiceServerBase {
    public:
        explicit Server(std::function<void()> unreg) : unreg_(std::move(unreg)) {}
        ~Server() override { unreg_(); }

    private:
        std::function<void()> unreg_;
    };

    void remove(const std::string& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        services_.erase(key);
    }

    mutable std::mutex mutex_;
    std::unordered_map<std::string, Entry> services_;
};

/// typed 服务客户端：绑定名字 + 注册表，.call(req) → Resp。
template <class Req, class Resp>
class ServiceClient {
public:
    explicit ServiceClient(std::string name,
                           ServiceRegistry* reg = &ServiceRegistry::instance())
        : name_(std::move(name)), reg_(reg) {}

    Resp call(const Req& req) { return reg_->template callImpl<Req, Resp>(name_, req); }
    [[nodiscard]] bool exists() const { return reg_->hasServiceImpl(name_); }
    [[nodiscard]] const std::string& name() const { return name_; }

private:
    std::string name_;
    ServiceRegistry* reg_;
};

}  // namespace simpleslam
