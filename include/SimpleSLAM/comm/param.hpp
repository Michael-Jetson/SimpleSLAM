// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file param.hpp
/// 参数的声明：用到参数的模块或插件在类里把它写成常量，写明名字、类型、默认值与说明，
/// 数值可以给上下限，还要说明运行中改不改。读写都拿这个常量，名字写错就编译不过。
///
/// ```cpp
/// static constexpr comm::Param<double> kGravity{
///     .name = "gravity", .fallback = 9.81, .doc = "重力加速度（m/s²）", .min = 9.0, .max = 10.5};
/// static constexpr std::array kParams{comm::reads(kGravity)};   // 这个类用到哪些参数
/// ...
/// const double g = param(kGravity);
/// ```
///
/// 几个类声明同一个名字就是共享：类型必须相同。只依赖标准库，配置层的目录登记实现时读它。

#include <concepts>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>

namespace simpleslam::comm {

/// 参数的值：四种标量。
using ParamValue = std::variant<bool, std::int64_t, double, std::string>;

/// 下限写它表示"必须为正"：最小的正规正数。
inline constexpr double kPositive = std::numeric_limits<double>::min();

enum class ParamType : std::uint8_t { kBool, kInt, kReal, kText };

[[nodiscard]] constexpr std::string_view toString(ParamType type) noexcept {
    switch (type) {
        case ParamType::kBool: return "bool";
        case ParamType::kInt: return "int";
        case ParamType::kReal: return "double";
        case ParamType::kText: return "string";
    }
    return "double";
}

/// 一个值的类型：与 `ParamValue` 的下标一一对应。
[[nodiscard]] inline ParamType typeOf(const ParamValue& value) noexcept {
    return static_cast<ParamType>(value.index());
}

template <class T>
concept ParamScalar = std::same_as<T, bool> || std::same_as<T, std::int64_t>
                      || std::same_as<T, double> || std::same_as<T, std::string>;

template <ParamScalar T>
[[nodiscard]] constexpr ParamType paramTypeOf() noexcept {
    if constexpr (std::is_same_v<T, bool>) {
        return ParamType::kBool;
    } else if constexpr (std::is_same_v<T, std::int64_t>) {
        return ParamType::kInt;
    } else if constexpr (std::is_same_v<T, double>) {
        return ParamType::kReal;
    } else {
        return ParamType::kText;
    }
}

/// 一个参数的声明。写成 `static constexpr` 常量，字符串参数的默认值是 `std::string_view`。
template <ParamScalar T>
struct Param {
    using Value = T;
    using Stored = std::conditional_t<std::is_same_v<T, std::string>, std::string_view, T>;

    /// 名字：字母、数字、`_`，用 `.` 分段，如 `graph.odometry_translation_weight`。
    std::string_view name{};
    /// 默认值：任务里没写值时取它。
    Stored fallback{};
    std::string_view doc{};
    /// 数值参数的下限与上限，都含端点；写入与装配时都核对。
    std::optional<double> min{};
    std::optional<double> max{};
    /// 运行中可以改。为 false 的参数开始运行之后一律拒绝写入。
    bool runtime = false;
};

/// 一个组件怎样用一个参数：读还是写，连同声明本身。可以写进 `constexpr` 数组。
struct ParamUse {
    std::string_view name{};
    ParamType type = ParamType::kReal;
    bool bool_fallback = false;
    std::int64_t int_fallback = 0;
    double real_fallback = 0.0;
    std::string_view text_fallback{};
    std::string_view doc{};
    std::optional<double> min{};
    std::optional<double> max{};
    bool runtime = false;
    /// 这个组件写它。运行中会变的参数至多一个组件写；没有组件写的，只有外部能改。
    bool writes = false;

    [[nodiscard]] ParamValue fallback() const {
        switch (type) {
            case ParamType::kBool: return bool_fallback;
            case ParamType::kInt: return int_fallback;
            case ParamType::kReal: return real_fallback;
            case ParamType::kText: return std::string(text_fallback);
        }
        return real_fallback;
    }
};

namespace detail {

template <ParamScalar T>
[[nodiscard]] constexpr ParamUse useOf(const Param<T>& param, bool writes) noexcept {
    ParamUse use{.name = param.name,
                 .type = paramTypeOf<T>(),
                 .doc = param.doc,
                 .min = param.min,
                 .max = param.max,
                 .runtime = param.runtime,
                 .writes = writes};
    if constexpr (std::is_same_v<T, bool>) {
        use.bool_fallback = param.fallback;
    } else if constexpr (std::is_same_v<T, std::int64_t>) {
        use.int_fallback = param.fallback;
    } else if constexpr (std::is_same_v<T, double>) {
        use.real_fallback = param.fallback;
    } else {
        use.text_fallback = param.fallback;
    }
    return use;
}

}  // namespace detail

/// 这个组件读这个参数。
template <ParamScalar T>
[[nodiscard]] constexpr ParamUse reads(const Param<T>& param) noexcept {
    return detail::useOf(param, false);
}

/// 这个组件写这个参数（也读）。
template <ParamScalar T>
[[nodiscard]] constexpr ParamUse writes(const Param<T>& param) noexcept {
    return detail::useOf(param, true);
}

/// 用到参数的类：带一个 `kParams`，列出它读写哪些参数。
template <class T>
concept DeclaresParams = requires {
    { T::kParams.size() } -> std::convertible_to<std::size_t>;
    { T::kParams[0] } -> std::convertible_to<const ParamUse&>;
};

}  // namespace simpleslam::comm
