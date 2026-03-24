#pragma once

#include <memory>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include <simpleslam/core/IService.h>
#include <simpleslam/core/ServiceNames.h>
#include <simpleslam/infrastructure/LogService.h>
#include <simpleslam/runtime/Permission.h>
#include <simpleslam/runtime/ResourceHub.h>

namespace SimpleSLAM {

class SystemContext {
public:
    template<typename T>
    void registerService(std::string name, std::shared_ptr<T> service) {
        static_assert(std::is_base_of_v<IService, T>, "T must inherit IService");
        if (!service) {
            throw std::invalid_argument("Cannot register null service: " + name);
        }

        std::unique_lock lock(mutex_);
        if (services_.count(name)) {
            throw std::runtime_error(
                "Service already registered (re-registration would invalidate existing references): " + name);
        }
        services_.emplace(
            std::move(name),
            std::static_pointer_cast<IService>(std::move(service)));
    }

    template<typename T>
    [[nodiscard]] std::shared_ptr<T> getService(const std::string& name) const {
        std::shared_lock lock(mutex_);
        auto it = services_.find(name);
        if (it == services_.end()) {
            throw std::runtime_error("Service not found: " + name);
        }

        auto result = std::dynamic_pointer_cast<T>(it->second);
        if (!result) {
            throw std::runtime_error("Service type mismatch: " + name);
        }
        return result;
    }

    [[nodiscard]] Log getLogger(const std::string& tag) const {
        return getService<LogService>(service_names::LOG)->getLogger(tag);
    }

    template<typename T, Permission P>
    [[nodiscard]] ResourceHandle<T, P> getResource(const std::string& name) const {
        return getService<ResourceHub>(service_names::RESOURCES)->get<T, P>(name);
    }

    // Check if a service exists (for optional dependencies)
    template<typename T>
    [[nodiscard]] bool hasService(const std::string& name) const {
        std::shared_lock lock(mutex_);
        auto it = services_.find(name);
        if (it == services_.end()) return false;
        return std::dynamic_pointer_cast<T>(it->second) != nullptr;
    }

    // Get optional service (returns nullptr if not found, no exception)
    template<typename T>
    [[nodiscard]] std::shared_ptr<T> tryGetService(const std::string& name) const {
        std::shared_lock lock(mutex_);
        auto it = services_.find(name);
        if (it == services_.end()) return nullptr;
        return std::dynamic_pointer_cast<T>(it->second);
    }

private:
    mutable std::shared_mutex mutex_;
    std::unordered_map<std::string, std::shared_ptr<IService>> services_;
};

} // namespace SimpleSLAM
