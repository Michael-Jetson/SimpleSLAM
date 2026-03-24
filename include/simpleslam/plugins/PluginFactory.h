#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <simpleslam/config/Params.h>

namespace SimpleSLAM {

template<typename PluginInterface>
class PluginFactory {
public:
    using Creator = std::function<std::unique_ptr<PluginInterface>(Params params)>;

    PluginFactory(const PluginFactory&) = delete;
    PluginFactory& operator=(const PluginFactory&) = delete;

    static PluginFactory& instance() {
        static PluginFactory factory;
        return factory;
    }

    // Note: No mutex protection — registerPlugin() is called during static
    // initialization (single-threaded). Phase 3+ dynamic plugin loading (dlopen)
    // will need a mutex or std::call_once around creators_ access.
    void registerPlugin(const std::string& name, Creator creator) {
        if (creators_.count(name)) {
            throw std::runtime_error("Plugin already registered (duplicate key): " + name);
        }
        creators_[name] = std::move(creator);
    }

    [[nodiscard]] std::unique_ptr<PluginInterface> create(const std::string& name,
                                                          Params params) const {
        auto it = creators_.find(name);
        if (it == creators_.end()) {
            throw std::runtime_error("Plugin not found: " + name);
        }
        return it->second(std::move(params));
    }

    [[nodiscard]] std::vector<std::string> listPlugins() const {
        std::vector<std::string> names;
        names.reserve(creators_.size());
        for (const auto& [name, creator] : creators_) {
            (void)creator;
            names.push_back(name);
        }
        return names;
    }

private:
    PluginFactory() = default;
    std::unordered_map<std::string, Creator> creators_;
};

template<typename PluginInterface, typename PluginImpl>
class PluginRegistrar {
public:
    PluginRegistrar(PluginFactory<PluginInterface>& factory, const std::string& name) {
        factory.registerPlugin(name, [](Params params) {
            return std::make_unique<PluginImpl>(std::move(params));
        });
    }
};

} // namespace SimpleSLAM

#define SIMPLESLAM_DETAIL_CONCAT_INNER(a, b) a##b
#define SIMPLESLAM_DETAIL_CONCAT(a, b) SIMPLESLAM_DETAIL_CONCAT_INNER(a, b)

#define REGISTER_PLUGIN(Interface, Impl, Name)                                              \
    static ::SimpleSLAM::PluginRegistrar<Interface, Impl>                                   \
        SIMPLESLAM_DETAIL_CONCAT(simpleslam_plugin_registrar_, __LINE__)(                   \
            ::SimpleSLAM::PluginFactory<Interface>::instance(),                             \
            Name)
