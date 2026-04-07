#pragma once

#include <string>
#include <type_traits>
#include <unordered_map>

#include <simpleslam/config/Node.h>

namespace SimpleSLAM {

using Params = std::unordered_map<std::string, Node>;

namespace detail {
template <typename T>
inline constexpr bool always_false_v = false;
} // namespace detail

template <typename T>
T getParam(const Params& params, const std::string& key, T default_value) {
    auto it = params.find(key);
    if (it == params.end()) {
        return default_value;
    }

    using ValueType = std::decay_t<T>;
    if constexpr (std::is_same_v<ValueType, bool>) {
        return it->second.asBool();
    } else if constexpr (std::is_same_v<ValueType, int>) {
        return it->second.asInt();
    } else if constexpr (std::is_same_v<ValueType, double>) {
        return it->second.asDouble();
    } else if constexpr (std::is_same_v<ValueType, std::string>) {
        return it->second.asString();
    } else {
        static_assert(detail::always_false_v<ValueType>, "Unsupported getParam type");
    }
}

} // namespace SimpleSLAM
