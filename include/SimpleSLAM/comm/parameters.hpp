// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file parameters.hpp
/// 参数服务器：一个进程一份。参数由用到它的组件声明（`param.hpp`），装配时连同初值一起定下；
/// 读的是不可变的快照，写的是事务：名字、类型、范围、谁能写、运行中改不改都核对，全部通过
/// 才生效，版本由提交生成。
///
/// 模块在自己的收件箱里换快照，所以一次回调之内值不变，换代与消息同一条 FIFO。插件在所属模块
/// 的回调里读写：读的是这个模块当前的快照，写先记下，回调结束后由模块合成一个事务提交。

#include <SimpleSLAM/comm/param.hpp>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace simpleslam::comm {

/// 某个版本的全部参数。发布后不可变。
struct ParamSnapshot {
    std::uint64_t revision = 0;
    std::map<std::string, ParamValue, std::less<>> values;

    template <ParamScalar T>
    [[nodiscard]] std::optional<T> get(std::string_view name) const {
        const auto it = values.find(name);
        if (it == values.end()) return std::nullopt;
        if (const T* p = std::get_if<T>(&it->second)) return *p;
        return std::nullopt;
    }
    [[nodiscard]] bool has(std::string_view name) const {
        return values.find(name) != values.end();
    }
};

using ParamSnapshotPtr = std::shared_ptr<const ParamSnapshot>;

class ParameterError : public std::runtime_error {
public:
    explicit ParameterError(const std::string& what) : std::runtime_error(what) {}
};

/// 一个参数的定义：声明，加上谁能写。装配时定下，运行中不变。
struct ParamDefinition {
    std::string name;
    ParamType type = ParamType::kReal;
    ParamValue fallback;
    std::string doc;
    std::optional<double> min;
    std::optional<double> max;
    bool runtime = false;
    /// 声明了写它的模块。空表示没有组件写它，只有外部能改。
    std::vector<std::string> writers;
};

/// 外部写入的写者名：操作员，以后的 ROS 适配模块。
inline constexpr std::string_view kExternalWriter = "外部";

struct ParamChange {
    std::string name;
    ParamValue value;
};

enum class ParamCommit : std::uint8_t {
    kStored,
    kUnknownName,     ///< 没有这个参数
    kRejectedType,    ///< 类型不符
    kRejectedValue,   ///< 超出声明的范围
    kRejectedStatic,  ///< 声明为运行中不改，已经开始运行
    kRejectedWriter,  ///< 写者没有声明写它
    kRejectedStopped, ///< 运行时已停止输入
};

struct ParamCommitResult {
    ParamCommit status = ParamCommit::kStored;
    std::uint64_t revision = 0;  ///< 提交后的版本；被拒时是当前版本
    std::string detail;
    ParamSnapshotPtr snapshot;   ///< 本次提交产生的那一份；被拒时为空
    ParamSnapshotPtr previous;   ///< 提交之前的那一份

    [[nodiscard]] bool ok() const noexcept { return status == ParamCommit::kStored; }
};

/// 值的文本：日志与报错用，往返无损。
[[nodiscard]] std::string paramText(const ParamValue& value);

/// 核对一个值能不能给这个参数：类型（整数可以给实数参数，换成实数）与范围。不合格返回说明。
[[nodiscard]] std::optional<std::string> checkParamValue(const ParamDefinition& definition,
                                                         ParamValue& value);

/// 一个组件用到的参数：它是谁（报错用，如 `system.modules.odometry.plugin(my_lio)`）、属于哪个
/// 模块（写的话就是写者）、用了哪些。
struct ParamComponent {
    std::string component;
    std::string module;
    std::vector<ParamUse> uses;
};

/// 收齐组件的声明，连同配置给的值，得到参数的定义与全部初值。核对：
/// - 同一个名字的类型必须相同；默认值不同的，配置里必须写明值；
/// - 运行中会变的参数至多一个模块写；运行中不改的参数不能声明写；
/// - 配置里的每个名字都要有组件声明过，类型与范围都合格。
/// 不合格抛 `ParameterError`，写明是哪个参数、哪几个组件。没写值的取默认值。
struct ResolvedParams {
    std::vector<ParamDefinition> definitions;
    std::map<std::string, ParamValue> values;
};
[[nodiscard]] ResolvedParams resolveParams(const std::vector<ParamComponent>& components,
                                           std::map<std::string, ParamValue> values);

class ParameterStore {
public:
    /// 装配时定下全部参数：定义与初值。初值按定义核对，没给的取默认值。只能定一次。
    void define(std::vector<ParamDefinition> definitions,
                std::map<std::string, ParamValue> values = {});
    [[nodiscard]] bool defined() const;

    /// 开始运行时调：声明为运行中不改的参数从此拒绝写入。
    void freeze();

    [[nodiscard]] ParamSnapshotPtr snapshot() const {
        std::lock_guard lock(mutex_);
        return snapshot_;
    }

    /// 全部参数的定义，按名字排列。
    [[nodiscard]] std::vector<ParamDefinition> definitions() const;

    /// 事务提交：全部通过才生效，任一不合格整批拒绝、版本不变。`writer` 是提交的模块名，
    /// 外部写入用 `kExternalWriter`。
    ParamCommitResult commit(const std::vector<ParamChange>& changes, std::string_view writer);

private:
    mutable std::mutex mutex_;
    std::map<std::string, ParamDefinition, std::less<>> definitions_;
    ParamSnapshotPtr snapshot_ = std::make_shared<const ParamSnapshot>();
    bool defined_ = false;
    bool frozen_ = false;
};

// ── 插件读写参数 ──

/// 一次回调的参数作用域：读哪一份快照、写记到哪里。模块进回调时设上，出来时还原，所以插件
/// 读到的是所属模块这次回调的快照；模块之外（测试里）直接构造一个，给插件一份快照。
/// 作用域按线程，嵌套时内层优先。
class ParamScope {
public:
    /// 读 `*source` 指向的那一份：模块的快照在换代时就地换掉，作用域跟着它。
    ParamScope(const ParamSnapshotPtr* source, std::vector<ParamChange>* writes) noexcept;
    /// 读一份给定的快照；`writes` 为空时不能写。
    explicit ParamScope(ParamSnapshotPtr snapshot,
                        std::vector<ParamChange>* writes = nullptr) noexcept;
    ~ParamScope();
    ParamScope(const ParamScope&) = delete;
    ParamScope& operator=(const ParamScope&) = delete;

    /// 当前线程上最内层的作用域；不在任何回调里时为空。
    [[nodiscard]] static const ParamScope* current() noexcept;

    [[nodiscard]] const ParamSnapshot& snapshot() const noexcept;
    /// 记下一个写；这个作用域不能写时抛 `std::logic_error`。
    void write(ParamChange change) const;

private:
    ParamSnapshotPtr owned_;
    const ParamSnapshotPtr* source_;
    std::vector<ParamChange>* writes_;
    const ParamScope* previous_;
};

/// 读一个参数：所属模块这次回调的快照里的值。不在回调里（构造函数里、插件自己开的线程里）
/// 抛 `std::logic_error`，不悄悄给默认值；快照里没有它也抛：用到它的类要在 `kParams` 里声明。
template <ParamScalar T>
[[nodiscard]] T readParam(const Param<T>& param) {
    const ParamScope* scope = ParamScope::current();
    if (scope == nullptr) {
        throw std::logic_error("参数 " + std::string(param.name)
                               + " 只能在所属模块的回调里读；模块之外（测试里）用 comm::ParamScope "
                                 "给一份快照");
    }
    if (const auto value = scope->snapshot().template get<T>(param.name)) return *value;
    throw std::logic_error("参数 " + std::string(param.name) + " 没有定义：用到它的类要在 kParams "
                           "里声明它（类型 " + std::string(toString(paramTypeOf<T>())) + "）");
}

/// 写一个参数：先记下，所属模块在这次回调结束后提交，从下一次回调起生效。
template <ParamScalar T>
void writeParam(const Param<T>& param, T value) {
    const ParamScope* scope = ParamScope::current();
    if (scope == nullptr) {
        throw std::logic_error("参数 " + std::string(param.name) + " 只能在所属模块的回调里写");
    }
    scope->write(ParamChange{std::string(param.name), ParamValue{std::move(value)}});
}

/// 插件接口的基类部件：在插件里直接 `param(k)` 读、`setParam(k, v)` 写，与 `log_` 一样。
class ParamUser {
protected:
    template <ParamScalar T>
    [[nodiscard]] static T param(const Param<T>& p) {
        return readParam(p);
    }
    template <ParamScalar T>
    static void setParam(const Param<T>& p, std::type_identity_t<T> value) {
        writeParam(p, std::move(value));
    }
};

}  // namespace simpleslam::comm
