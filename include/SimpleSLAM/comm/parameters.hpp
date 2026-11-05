// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file parameters.hpp
/// 参数：类型化真源 + 版本化不可变快照 + 事务提交。
///
/// 三条规则：
/// 1. 读的是快照——一次求解中途不会混进新值；
/// 2. 写的是事务——多个互相约束的参数一起校验提交，可带 expected_revision 防覆盖旧版本；
/// 3. 声明不是写入——declare() 只补缺失的默认值，绝不覆盖配置已给定的值。
///    声明若能覆盖，模块一声明就把装配期预置的配置值改回代码默认值，
///    配置与运行期参数之间就有了第二套真源。
/// 这里只到 Stored（存储已提交新版本）。Queued 与 Applied 分别由提交结果与
/// Runtime::paramsApplied() 表达，三者的区分见 types.hpp 的 ParamCommit。

#include <SimpleSLAM/comm/types.hpp>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace simpleslam::comm {

/// 参数值。类型集合刻意小：复杂结构属于配置聚合体，不进运行期可调参数。
using ParamValue = std::variant<bool, std::int64_t, double, std::string>;

/// 某个版本的全部参数。发布后不可变。
struct ParamSnapshot {
    std::uint64_t revision = 0;
    std::map<std::string, ParamValue> values;  ///< 有序容器：遍历顺序确定

    template <class T>
    [[nodiscard]] std::optional<T> get(const std::string& name) const {
        auto it = values.find(name);
        if (it == values.end()) return std::nullopt;
        if (const T* p = std::get_if<T>(&it->second)) return *p;
        return std::nullopt;
    }

    [[nodiscard]] bool has(const std::string& name) const { return values.count(name) != 0; }
};

using ParamSnapshotPtr = std::shared_ptr<const ParamSnapshot>;

/// 声明的结果。声明是幂等的：同名同类型重复声明不是错误，也不改动现值。
enum class ParamDeclare : std::uint8_t {
    kDeclared,       ///< 之前没有，已按默认值建立
    kAlreadyPresent, ///< 之前已有（配置或更早的声明给定），保持原值
};

/// 参数使用错误：类型冲突。装配期问题一律 fail-fast，不退化成默认值。
class ParameterError : public std::runtime_error {
public:
    explicit ParameterError(const std::string& what) : std::runtime_error(what) {}
};

struct ParamChange {
    std::string name;
    ParamValue value;
};

struct ParamCommitResult {
    ParamCommit status = ParamCommit::kStored;
    std::uint64_t revision = 0;  ///< 提交后的版本；被拒时是当前版本
    std::string detail;
    /// **本次提交产生的那一份**快照，不是"提交之后再去取的最新快照"。
    ///
    /// 后者是两次操作：并发提交可以挤在中间，于是返回的版本号与随后分发给模块的
    /// 快照对不上。被拒时为空。
    ParamSnapshotPtr snapshot;
    /// 新快照没能排进收件箱的模块名。收件箱已关或控制类工作满载都会落在这里——
    /// 参数已经进了真源，但这些模块还在用旧的一代，必须可见而不是静默。
    std::vector<std::string> undelivered;

    [[nodiscard]] bool ok() const noexcept { return status == ParamCommit::kStored; }
    /// 已提交，且新快照已经排进了每个模块的收件箱。
    ///
    /// 这只是 **Queued**：模块还没被驱动时，它们手里仍是旧的一代。要问"生效了没有"，
    /// 看 Runtime::paramsApplied(revision)。
    [[nodiscard]] bool fullyQueued() const noexcept { return ok() && undelivered.empty(); }
};

/// 参数更新通知的消息体。消费者据 revision 去取快照，不把值塞进通知。
struct ParamUpdate {
    std::uint64_t revision = 0;
};

/// 参数存储。唯一写者，提交在锁内完成，读取走无锁的快照指针拷贝。
class ParameterStore {
public:
    /// 跨字段校验器：返回 false 表示拒绝，detail 写明原因。
    using Validator = std::function<bool(const ParamSnapshot&, std::string&)>;

    ParameterStore() : snapshot_(std::make_shared<const ParamSnapshot>()) {}

    /// 声明一个参数及其默认值。声明即 schema：未声明的名字不接受写入。
    ///
    /// 只补缺失值。已存在同名参数时保持其现值——装配期由配置预置的值优先于代码默认值。
    /// 已存在但类型不符是装配期错误：代码与配置对同一个名字的理解不一致，必须当场暴露。
    template <class T>
    ParamDeclare declare(std::string name, T default_value) {
        std::lock_guard lock(mutex_);
        auto it = snapshot_->values.find(name);
        if (it != snapshot_->values.end()) {
            if (!std::holds_alternative<T>(it->second)) {
                throw ParameterError("参数 " + name + " 的声明类型与已有值的类型不符");
            }
            return ParamDeclare::kAlreadyPresent;
        }
        auto next = std::make_shared<ParamSnapshot>(*snapshot_);
        next->values.emplace(std::move(name), ParamValue{std::move(default_value)});
        next->revision = snapshot_->revision;  // 声明不推进版本
        snapshot_ = std::move(next);
        return ParamDeclare::kDeclared;
    }

    /// 装配期预置一个值：名字可以尚未声明，类型由此固定。配置加载走这条路。
    ///
    /// 与 commit() 的区别：commit 是运行期事务，只接受已声明的名字；
    /// 这里是"先于代码的配置"，用于把 YAML/命令行的值放进真源，等模块来声明。
    void preset(std::string name, ParamValue value) {
        std::lock_guard lock(mutex_);
        auto next = std::make_shared<ParamSnapshot>(*snapshot_);
        next->values.insert_or_assign(std::move(name), std::move(value));
        next->revision = snapshot_->revision;
        snapshot_ = std::move(next);
    }

    [[nodiscard]] ParamSnapshotPtr snapshot() const {
        std::lock_guard lock(mutex_);
        return snapshot_;
    }

    [[nodiscard]] std::uint64_t revision() const {
        std::lock_guard lock(mutex_);
        return snapshot_->revision;
    }

    void setValidator(Validator v) {
        std::lock_guard lock(mutex_);
        validator_ = std::move(v);
    }

    /// 事务提交：全部通过才生效，任一失败则整批拒绝、版本不变。
    ParamCommitResult commit(const std::vector<ParamChange>& changes,
                             std::optional<std::uint64_t> expected_revision = std::nullopt) {
        std::lock_guard lock(mutex_);
        if (expected_revision && *expected_revision != snapshot_->revision) {
            return {ParamCommit::kRejectedStale, snapshot_->revision,
                    "expected_revision 与当前版本不符", nullptr, {}};
        }

        auto next = std::make_shared<ParamSnapshot>(*snapshot_);
        for (const auto& change : changes) {
            auto it = next->values.find(change.name);
            if (it == next->values.end()) {
                return {ParamCommit::kUnknownName, snapshot_->revision, change.name, nullptr, {}};
            }
            if (it->second.index() != change.value.index()) {
                return {ParamCommit::kRejectedType, snapshot_->revision, change.name, nullptr, {}};
            }
            it->second = change.value;
        }

        if (validator_) {
            std::string detail;
            if (!validator_(*next, detail)) {
                return {ParamCommit::kRejectedValue, snapshot_->revision, std::move(detail),
                        nullptr, {}};
            }
        }

        next->revision = snapshot_->revision + 1;
        const std::uint64_t committed = next->revision;
        ParamSnapshotPtr published = next;  // 本次提交的那一份，原样带回
        snapshot_ = std::move(next);
        return {ParamCommit::kStored, committed, {}, std::move(published), {}};
    }

private:
    mutable std::mutex mutex_;
    ParamSnapshotPtr snapshot_;
    Validator validator_;
};

}  // namespace simpleslam::comm
