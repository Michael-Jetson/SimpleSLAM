// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file parameters.hpp
/// 参数服务器：一个进程一份。读快照，写事务，声明只补缺失值。
///
/// 模块在自己的收件箱里换快照，所以一次回调内快照不变，换代与消息同一条 FIFO。

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

using ParamValue = std::variant<bool, std::int64_t, double, std::string>;

/// 某个版本的全部参数。发布后不可变。
struct ParamSnapshot {
    std::uint64_t revision = 0;
    std::map<std::string, ParamValue> values;

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

class ParameterError : public std::runtime_error {
public:
    explicit ParameterError(const std::string& what) : std::runtime_error(what) {}
};

struct ParamChange {
    std::string name;
    ParamValue value;
};

enum class ParamCommit : std::uint8_t {
    kStored,
    kRejectedType,
    kRejectedValue,
    kUnknownName,
    kRejectedStopped,
};

struct ParamCommitResult {
    ParamCommit status = ParamCommit::kStored;
    std::uint64_t revision = 0;  ///< 提交后的版本；被拒时是当前版本
    std::string detail;
    ParamSnapshotPtr snapshot;   ///< 本次提交产生的那一份；被拒时为空

    [[nodiscard]] bool ok() const noexcept { return status == ParamCommit::kStored; }
};

class ParameterStore {
public:
    /// 跨字段校验：返回 false 表示拒绝，detail 写明原因。
    using Validator = std::function<bool(const ParamSnapshot&, std::string&)>;

    /// 声明即 schema：未声明的名字不接受写入。已存在时保持现值，类型不符是装配期错误。
    template <class T>
    void declare(std::string name, T default_value) {
        std::lock_guard lock(mutex_);
        auto it = snapshot_->values.find(name);
        if (it != snapshot_->values.end()) {
            if (!std::holds_alternative<T>(it->second)) {
                throw ParameterError("参数 " + name + " 的声明类型与已有值的类型不符");
            }
            return;
        }
        auto next = std::make_shared<ParamSnapshot>(*snapshot_);
        next->values.emplace(std::move(name), ParamValue{std::move(default_value)});
        snapshot_ = std::move(next);
    }

    /// 装配期预置一个值（YAML、命令行），先于代码的声明。
    void preset(std::string name, ParamValue value);

    [[nodiscard]] ParamSnapshotPtr snapshot() const {
        std::lock_guard lock(mutex_);
        return snapshot_;
    }

    void setValidator(Validator v) {
        std::lock_guard lock(mutex_);
        validator_ = std::move(v);
    }

    /// 事务提交：全部通过才生效，任一失败则整批拒绝、版本不变。
    ParamCommitResult commit(const std::vector<ParamChange>& changes);

private:
    mutable std::mutex mutex_;
    ParamSnapshotPtr snapshot_ = std::make_shared<const ParamSnapshot>();
    Validator validator_;
};

}  // namespace simpleslam::comm
