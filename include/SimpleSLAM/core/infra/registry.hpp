#pragma once

/// @file registry.hpp
/// 插件/模块注册表——静态初始化自动注册，名称驱动创建
///
/// 提供两种注册宏，一行代码即可完成注册：
///   - SIMPLESLAM_REGISTER_PLUGIN：注册类型擦除插件（如回环检测器、位姿图优化器）
///   - SIMPLESLAM_REGISTER_MODULE：注册继承模块（如里程计、后端服务、数据源）
///
/// 创建实例时按名称查找，可选传入 YAML 配置：
///   auto det = Registry<AnyLoopDetector>::create("scan_context", yaml_node);
///   auto det = Registry<AnyLoopDetector>::create("scan_context");  // 无参创建
///
/// 工作原理：
///   宏在文件作用域展开为 static const bool 变量，触发 C++ 静态初始化。
///   只要编译单元被链接进最终二进制，注册自动完成——无需手动调用。
///   内部使用 Meyer's Singleton（函数内 static），保证跨翻译单元的初始化顺序安全。
///
/// 跨项目使用：
///   在自己的算法头文件/源文件中使用注册宏，然后链接到使用 SimpleSLAM 的项目即可。
///   注意：静态库 (.a) 中未被引用的翻译单元可能被链接器丢弃。
///   解决方案：CMake OBJECT 库 / --whole-archive / 显式引用 dummy 符号。
///
/// 冲突检测：
///   - 同名重复注册 → stderr 输出错误信息 + abort()（这是编程错误）
///   - 同一类型以不同名注册 → stderr 警告（别名行为，非致命）

#include <SimpleSLAM/core/infra/demangle.hpp>

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace simpleslam {

/// 注册表条目元数据，用于 registered() 内省查询
struct RegistryEntry {
    std::string name;            ///< 注册名称（如 "scan_context"）
    std::type_index concrete_type;  ///< 具体实现类的 type_index
};

/// 类型安全的工厂注册表（模板类）
///
/// 每个模板参数 T 对应一个独立的全局注册表实例。
/// 不同类别的注册表互不干扰：
///   - Registry<AnyLoopDetector>     — 回环检测插件注册表
///   - Registry<AnyPoseGraphOptimizer> — 位姿图优化插件注册表
///   - Registry<unique_ptr<OdometryBase>> — 里程计模块注册表
///   - Registry<unique_ptr<ServiceBase>>  — 后端服务模块注册表
///
/// 线程安全：内部使用 mutex 保护，可在 main() 之后安全调用。
template <typename T>
class Registry final {
public:
    /// 工厂函数签名：接收 YAML 配置节点，返回 T 实例
    using FactoryFn = std::function<T(const YAML::Node&)>;

    // ── 创建 ──

    /// 按名称+配置创建实例
    ///
    /// @param name   注册时使用的名称（如 "scan_context"）
    /// @param config YAML 配置节点，传递给具体类型的构造函数
    /// @throws std::runtime_error 名称未注册时抛出，错误信息包含已注册名称列表
    ///
    /// 示例：
    ///   auto det = Registry<AnyLoopDetector>::create("scan_context", cfg.node("loop"));
    static T create(const std::string& name, const YAML::Node& config) {
        FactoryFn factory;
        {
            // 持锁查找，复制 factory 后立即释放——factory 调用可能耗时（读文件等）
            auto& d = data();
            std::lock_guard lock(d.mutex);
            auto it = d.factories.find(name);
            if (it == d.factories.end()) {
                std::string available;
                for (const auto& [n, _] : d.factories) {
                    if (!available.empty()) available += ", ";
                    available += n;
                }
                throw std::runtime_error(
                    "Registry::create: unknown name '" + name +
                    "'. Available: [" + available + "]");
            }
            factory = it->second;
        }
        // 锁已释放，安全调用 factory
        auto obj = factory(config);
        if constexpr (requires(T& t) { t.setName(std::string{}); }) {
            obj.setName(name);
        }
        return obj;
    }

    /// 按名称创建实例（无配置，使用默认参数）
    ///
    /// 等价于 create(name, YAML::Node{})。
    /// 适用于不需要配置的简单类型，或类型有默认构造函数的场景。
    ///
    /// 示例：
    ///   auto det = Registry<AnyLoopDetector>::create("simple_detector");
    static T create(const std::string& name) {
        return create(name, YAML::Node{});
    }

    // ── 注册（宏内部调用，非用户直接使用）──

    /// 注册工厂函数
    ///
    /// @param name          注册名称（全局唯一）
    /// @param factory       工厂函数（接收 YAML::Node，返回 T）
    /// @param concrete_type 具体实现类的 type_index（用于别名检测）
    /// @return true（用于初始化 static const bool 变量）
    ///
    /// 冲突检测规则：
    ///   - 同名重复注册 → 致命错误，cerr + abort()
    ///   - 同类型不同名 → 非致命警告，cerr 输出提示
    static bool registerFactory(const std::string& name,
                                FactoryFn factory,
                                std::type_index concrete_type) {
        auto& d = data();
        std::lock_guard lock(d.mutex);

        // 同名检测——编程错误，必须终止
        if (d.factories.contains(name)) {
            std::cerr << "[SimpleSLAM] FATAL: Registry duplicate name '"
                      << name << "' — aborting\n";
            std::abort();
        }

        // 同类型检测——别名行为，仅警告
        for (const auto& [existing_name, existing_type] : d.type_map) {
            if (existing_type == concrete_type) {
                std::cerr << "[SimpleSLAM] WARNING: Registry type already "
                             "registered as '"
                          << existing_name << "', adding alias '" << name
                          << "'\n";
            }
        }

        d.factories.emplace(name, std::move(factory));
        d.type_map.emplace(name, concrete_type);
        return true;
    }

    // ── 查询 ──

    /// 检查名称是否已注册
    static bool has(const std::string& name) {
        auto& d = data();
        std::lock_guard lock(d.mutex);
        return d.factories.contains(name);
    }

    /// 列出全部已注册条目（名称 + 具体类型的 type_index）
    ///
    /// 可用于调试、诊断、打印可用插件列表等场景。
    static std::vector<RegistryEntry> registered() {
        auto& d = data();
        std::lock_guard lock(d.mutex);
        std::vector<RegistryEntry> entries;
        entries.reserve(d.type_map.size());
        for (const auto& [name, type] : d.type_map) {
            entries.push_back({name, type});
        }
        return entries;
    }

    /// 列出全部已注册名称（纯字符串列表）
    static std::vector<std::string> registeredNames() {
        auto& d = data();
        std::lock_guard lock(d.mutex);
        std::vector<std::string> names;
        names.reserve(d.factories.size());
        for (const auto& [name, _] : d.factories) {
            names.push_back(name);
        }
        return names;
    }

private:
    /// 注册表内部数据——持有工厂函数映射和类型映射
    struct RegistryData {
        std::mutex mutex;  ///< 保护并发访问
        std::unordered_map<std::string, FactoryFn> factories;  ///< 名称 → 工厂函数
        std::unordered_map<std::string, std::type_index> type_map;  ///< 名称 → 具体类型
    };

    /// Meyer's Singleton——保证跨翻译单元的初始化顺序安全
    /// C++11 标准保证函数内 static 变量的线程安全初始化
    static RegistryData& data() {
        static RegistryData instance;
        return instance;
    }
};

// ═══════════════════════════════════════════════════════════════════
// 构造函数适配辅助（detail 命名空间，宏内部使用）
// ═══════════════════════════════════════════════════════════════════

namespace detail {

/// 为类型擦除插件生成工厂函数
///
/// 自动检测 Concrete 类型的构造函数签名：
///   - 如果 Concrete 可从 YAML::Node 构造 → 传入配置
///   - 否则 → 使用默认构造函数（忽略配置）
///
/// 返回的 lambda 签名为 AnyType(const YAML::Node&)，
/// 内部将 Concrete 实例传给 AnyType 的类型擦除构造函数。
template <typename AnyType, typename Concrete>
auto makePluginFactory() {
    return [](const YAML::Node& config) -> AnyType {
        if constexpr (std::is_constructible_v<Concrete, const YAML::Node&>) {
            return AnyType(Concrete(config));
        } else {
            (void)config;
            return AnyType(Concrete{});
        }
    };
}

/// 为继承模块生成工厂函数（返回 unique_ptr<Base>）
///
/// 自动检测 Derived 类型的构造函数签名：
///   - 如果 Derived 可从 YAML::Node 构造 → 传入配置
///   - 否则 → 使用默认构造函数（忽略配置）
///
/// 返回的 lambda 签名为 unique_ptr<Base>(const YAML::Node&)。
template <typename Base, typename Derived>
auto makeModuleFactory() {
    return [](const YAML::Node& config) -> std::unique_ptr<Base> {
        if constexpr (std::is_constructible_v<Derived, const YAML::Node&>) {
            return std::make_unique<Derived>(config);
        } else {
            (void)config;
            return std::make_unique<Derived>();
        }
    };
}

}  // namespace detail

// ═══════════════════════════════════════════════════════════════════
// 注册宏
// ═══════════════════════════════════════════════════════════════════

/// 辅助宏：二级展开，确保 __COUNTER__ 在拼接前被求值
#define SIMPLESLAM_CONCAT_IMPL(a, b) a##b
#define SIMPLESLAM_CONCAT(a, b) SIMPLESLAM_CONCAT_IMPL(a, b)

/// 注册类型擦除插件
///
/// 将 ConcreteType 注册到 Registry<AnyType> 中，关联名称 Name。
/// ConcreteType 必须满足 AnyType 对应的 concept（如 LoopDetector）。
/// ConcreteType 可以有 YAML::Node 构造函数（接收配置），也可以只有默认构造函数。
///
/// 用法：
///   SIMPLESLAM_REGISTER_PLUGIN(AnyLoopDetector, "scan_context", ScanContextDetector);
///
/// 展开后等价于：
///   static const bool simpleslam_reg_N =
///       Registry<AnyLoopDetector>::registerFactory("scan_context", factory, typeid(...));
///
/// @param AnyType      类型擦除包装类（如 AnyLoopDetector）
/// @param Name         注册名称字符串（如 "scan_context"）
/// @param ConcreteType 具体实现类（如 ScanContextDetector）
#define SIMPLESLAM_REGISTER_PLUGIN(AnyType, Name, ConcreteType)              \
    static const bool SIMPLESLAM_CONCAT(simpleslam_reg_, __COUNTER__) =      \
        ::simpleslam::Registry<::simpleslam::AnyType>::registerFactory(      \
            Name,                                                            \
            ::simpleslam::detail::makePluginFactory<                          \
                ::simpleslam::AnyType, ConcreteType>(),                      \
            std::type_index(typeid(ConcreteType)))

/// 注册继承模块
///
/// 将 DerivedType 注册到 Registry<unique_ptr<BaseType>> 中，关联名称 Name。
/// DerivedType 必须继承自 BaseType。
/// DerivedType 可以有 YAML::Node 构造函数，也可以只有默认构造函数。
///
/// 用法：
///   SIMPLESLAM_REGISTER_MODULE(OdometryBase, "lo_icp", LoIcpOdometry);
///
/// 创建时：
///   auto odom = Registry<unique_ptr<OdometryBase>>::create("lo_icp", yaml_node);
///   auto odom = Registry<unique_ptr<OdometryBase>>::create("lo_icp");  // 无参
///
/// @param BaseType    基类（如 OdometryBase、ServiceBase、ISensorSource）
/// @param Name        注册名称字符串
/// @param DerivedType 派生类
#define SIMPLESLAM_REGISTER_MODULE(BaseType, Name, DerivedType)              \
    static const bool SIMPLESLAM_CONCAT(simpleslam_reg_, __COUNTER__) =      \
        ::simpleslam::Registry<std::unique_ptr<::simpleslam::BaseType>>      \
            ::registerFactory(                                               \
                Name,                                                        \
                ::simpleslam::detail::makeModuleFactory<                      \
                    ::simpleslam::BaseType, DerivedType>(),                   \
                std::type_index(typeid(DerivedType)))

}  // namespace simpleslam
