#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>

#include <simpleslam/core/IService.h>
#include <simpleslam/runtime/ResourceHandle.h>

namespace SimpleSLAM {

class ResourceHub : public IService {
public:
    template<typename T>
    void registerResource(std::string name, std::shared_ptr<T> resource) {
        if (!resource) {
            throw std::invalid_argument("Cannot register null resource: " + name);
        }

        std::lock_guard lock(registry_mutex_);
        if (resources_.count(name)) {
            throw std::runtime_error(
                "Resource already registered (re-registration would invalidate existing handles): " + name);
        }
        resources_.emplace(
            std::move(name),
            ResourceEntry{
                std::static_pointer_cast<void>(std::move(resource)),
                std::type_index(typeid(T)),
                std::make_unique<std::shared_mutex>()
            });
    }

    template<typename T, Permission P>
    [[nodiscard]] ResourceHandle<T, P> get(const std::string& name) const {
        std::lock_guard lock(registry_mutex_);
        auto it = resources_.find(name);
        if (it == resources_.end()) {
            throw std::runtime_error("Resource not found: " + name);
        }
        // Note: typeid checks exact type only, does not support base-class lookup.
        // Phase 4: replace with dynamic_pointer_cast when capability interfaces
        // (IMapQuery/IMapMutation) are introduced.
        if (it->second.type != std::type_index(typeid(T))) {
            throw std::runtime_error("Resource type mismatch: " + name);
        }

        return ResourceHandle<T, P>(
            std::static_pointer_cast<T>(it->second.resource),
            it->second.mutex.get());
    }

private:
    struct ResourceEntry {
        std::shared_ptr<void> resource;
        std::type_index type{typeid(void)};
        std::unique_ptr<std::shared_mutex> mutex = std::make_unique<std::shared_mutex>();
    };

    mutable std::mutex registry_mutex_;
    std::unordered_map<std::string, ResourceEntry> resources_;
};

} // namespace SimpleSLAM
