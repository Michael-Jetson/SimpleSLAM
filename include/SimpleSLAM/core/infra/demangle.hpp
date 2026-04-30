#pragma once

/// @file demangle.hpp
/// 类型名反修饰工具——将编译器修饰名转为可读类名
///
/// GCC/Clang 使用 abi::__cxa_demangle，MSVC 的 typeid 本身不修饰。
/// 去除命名空间前缀，只保留类名（如 "simpleslam::ScanContext" → "ScanContext"）。

#include <cstdlib>
#include <string>
#include <typeinfo>

#if defined(__GNUC__) || defined(__clang__)
#include <cxxabi.h>
#endif

namespace simpleslam::detail {

template <typename T>
std::string demangle() {
#if defined(__GNUC__) || defined(__clang__)
    int status = 0;
    char* raw = abi::__cxa_demangle(typeid(T).name(), nullptr, nullptr, &status);
    std::string result = (status == 0) ? raw : typeid(T).name();
    std::free(raw);
    if (auto pos = result.rfind("::"); pos != std::string::npos) {
        result = result.substr(pos + 2);
    }
    return result;
#else
    return typeid(T).name();
#endif
}

}  // namespace simpleslam::detail
