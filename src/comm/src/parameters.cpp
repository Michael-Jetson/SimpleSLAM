// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/comm/parameters.hpp>

#include <algorithm>
#include <cmath>
#include <format>
#include <utility>

namespace simpleslam::comm {

namespace {

thread_local const ParamScope* t_current_scope = nullptr;

std::string join(const std::vector<std::string>& items) {
    std::string out;
    for (const auto& item : items) out += (out.empty() ? "" : "、") + item;
    return out;
}

/// 整数可以给实数参数：换成实数。类型对不上返回 false。
bool coerce(const ParamDefinition& definition, ParamValue& value) {
    if (definition.type == ParamType::kReal && std::holds_alternative<std::int64_t>(value)) {
        value = static_cast<double>(std::get<std::int64_t>(value));
    }
    return typeOf(value) == definition.type;
}

std::string typeProblem(const ParamDefinition& definition, const ParamValue& value) {
    return std::format("参数 {} 要 {}，得到 {}（{}）", definition.name, toString(definition.type),
                       toString(typeOf(value)), paramText(value));
}

/// 数值的范围：有限、不越过声明的上下限。
std::optional<std::string> rangeProblem(const ParamDefinition& definition,
                                        const ParamValue& value) {
    double number = 0.0;
    if (const auto* real = std::get_if<double>(&value)) {
        if (!std::isfinite(*real)) {
            return std::format("参数 {} 必须是有限数，得到 {}", definition.name, *real);
        }
        number = *real;
    } else if (const auto* integer = std::get_if<std::int64_t>(&value)) {
        number = static_cast<double>(*integer);
    } else {
        return std::nullopt;
    }
    if (definition.min && number < *definition.min) {
        if (*definition.min == kPositive) {
            return std::format("参数 {} 必须为正，得到 {}", definition.name, paramText(value));
        }
        return std::format("参数 {} 不能小于 {}，得到 {}", definition.name, *definition.min,
                           paramText(value));
    }
    if (definition.max && number > *definition.max) {
        return std::format("参数 {} 不能大于 {}，得到 {}", definition.name, *definition.max,
                           paramText(value));
    }
    return std::nullopt;
}

}  // namespace

std::string paramText(const ParamValue& value) {
    return std::visit(
        [](const auto& v) -> std::string {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, bool>) {
                return v ? "true" : "false";
            } else if constexpr (std::is_same_v<T, std::string>) {
                return "\"" + v + "\"";
            } else {
                return std::format("{}", v);
            }
        },
        value);
}

std::optional<std::string> checkParamValue(const ParamDefinition& definition, ParamValue& value) {
    if (!coerce(definition, value)) return typeProblem(definition, value);
    return rangeProblem(definition, value);
}

ResolvedParams resolveParams(const std::vector<ParamComponent>& components,
                             std::map<std::string, ParamValue> values) {
    struct Entry {
        ParamDefinition definition;
        std::vector<std::string> declared_by;
        std::vector<std::string> written_by;
        bool fallback_differs = false;
    };
    std::map<std::string, Entry, std::less<>> entries;
    for (const auto& component : components) {
        for (const auto& use : component.uses) {
            const std::string name(use.name);
            auto [it, inserted] = entries.try_emplace(name);
            Entry& entry = it->second;
            auto& definition = entry.definition;
            if (inserted) {
                definition = ParamDefinition{.name = name,
                                             .type = use.type,
                                             .fallback = use.fallback(),
                                             .doc = std::string(use.doc),
                                             .min = use.min,
                                             .max = use.max,
                                             .runtime = use.runtime,
                                             .writers = {}};
            } else {
                if (definition.type != use.type) {
                    throw ParameterError(std::format(
                        "参数 {} 的类型不一致：{} 声明成 {}，{} 声明成 {}", name,
                        join(entry.declared_by), toString(definition.type), component.component,
                        toString(use.type)));
                }
                if (definition.runtime != use.runtime) {
                    throw ParameterError(std::format("参数 {} 运行中改不改，{} 与 {} 声明得不一样",
                                                     name, join(entry.declared_by),
                                                     component.component));
                }
                if (definition.fallback != use.fallback()) entry.fallback_differs = true;
                // 几个组件各有范围：值要同时满足它们，取交集
                if (use.min) definition.min = std::max(definition.min.value_or(*use.min), *use.min);
                if (use.max) definition.max = std::min(definition.max.value_or(*use.max), *use.max);
            }
            entry.declared_by.push_back(component.component);
            if (use.writes) {
                if (!use.runtime) {
                    throw ParameterError(std::format("参数 {} 声明为运行中不改，{} 不能声明写它",
                                                     name, component.component));
                }
                entry.written_by.push_back(component.component);
                definition.writers.push_back(component.module);
            }
        }
    }
    for (const auto& [name, entry] : entries) {
        if (entry.written_by.size() > 1) {
            throw ParameterError(std::format(
                "参数 {} 有 {} 个写者（{}）：运行中会变的参数至多一个组件写", name,
                entry.written_by.size(), join(entry.written_by)));
        }
    }
    for (auto& [name, value] : values) {
        const auto it = entries.find(name);
        if (it == entries.end()) {
            if (entries.empty()) {
                throw ParameterError(
                    std::format("未知参数 {}：这次选中的组件都没有声明参数", name));
            }
            std::vector<std::string> known;
            for (const auto& [declared, entry] : entries) known.push_back(declared);
            throw ParameterError(std::format("未知参数 {}：选中的组件都没有声明它；声明过的有 {}",
                                             name, join(known)));
        }
        if (auto problem = checkParamValue(it->second.definition, value)) {
            throw ParameterError(*problem);
        }
    }
    ResolvedParams out;
    for (auto& [name, entry] : entries) {
        if (!values.contains(name)) {
            if (entry.fallback_differs) {
                throw ParameterError(std::format(
                    "参数 {} 的默认值在 {} 里声明得不一样：要在 system.params 里写明它的值", name,
                    join(entry.declared_by)));
            }
            ParamValue fallback = entry.definition.fallback;
            if (auto problem = checkParamValue(entry.definition, fallback)) {
                throw ParameterError("声明的默认值不合格：" + *problem);
            }
            values.emplace(name, std::move(fallback));
        }
        out.definitions.push_back(std::move(entry.definition));
    }
    out.values = std::move(values);
    return out;
}

void ParameterStore::define(std::vector<ParamDefinition> definitions,
                            std::map<std::string, ParamValue> values) {
    std::lock_guard lock(mutex_);
    if (defined_) throw ParameterError("参数只能定义一次：装配时定下，运行中不变");
    std::map<std::string, ParamDefinition, std::less<>> table;
    auto next = std::make_shared<ParamSnapshot>();
    for (auto& definition : definitions) {
        if (table.contains(definition.name)) {
            throw ParameterError("参数 " + definition.name + " 定义了两次");
        }
        ParamValue value = definition.fallback;
        if (const auto it = values.find(definition.name); it != values.end()) {
            value = std::move(it->second);
            values.erase(it);
        }
        if (auto problem = checkParamValue(definition, value)) throw ParameterError(*problem);
        next->values.emplace(definition.name, std::move(value));
        std::string name = definition.name;
        table.emplace(std::move(name), std::move(definition));
    }
    if (!values.empty()) {
        throw ParameterError("未知参数 " + values.begin()->first + "：没有定义它");
    }
    definitions_ = std::move(table);
    snapshot_ = std::move(next);
    defined_ = true;
}

bool ParameterStore::defined() const {
    std::lock_guard lock(mutex_);
    return defined_;
}

void ParameterStore::freeze() {
    std::lock_guard lock(mutex_);
    frozen_ = true;
}

std::vector<ParamDefinition> ParameterStore::definitions() const {
    std::lock_guard lock(mutex_);
    std::vector<ParamDefinition> out;
    out.reserve(definitions_.size());
    for (const auto& [name, definition] : definitions_) out.push_back(definition);
    return out;
}

ParamCommitResult ParameterStore::commit(const std::vector<ParamChange>& changes,
                                         std::string_view writer) {
    // 核对只用定义里的数据，不跑用户代码，所以整个提交在锁里做完
    std::lock_guard lock(mutex_);
    const ParamSnapshotPtr base = snapshot_;
    const auto reject = [&](ParamCommit status, std::string detail) {
        return ParamCommitResult{.status = status,
                                 .revision = base->revision,
                                 .detail = std::move(detail),
                                 .snapshot = nullptr,
                                 .previous = base};
    };
    if (changes.empty()) {
        return {.status = ParamCommit::kStored,
                .revision = base->revision,
                .detail = {},
                .snapshot = nullptr,
                .previous = base};
    }
    auto next = std::make_shared<ParamSnapshot>(*base);
    for (const auto& change : changes) {
        const auto it = definitions_.find(change.name);
        if (it == definitions_.end()) {
            return reject(ParamCommit::kUnknownName, "没有参数 " + change.name);
        }
        const ParamDefinition& definition = it->second;
        const auto& writers = definition.writers;
        const bool allowed = writers.empty() ? writer == kExternalWriter
                                             : std::ranges::find(writers, writer) != writers.end();
        if (!allowed) {
            return reject(ParamCommit::kRejectedWriter,
                          writers.empty()
                              ? std::format("参数 {} 没有组件写它，只有外部能改；{} 没有声明写它",
                                            change.name, writer)
                              : std::format("参数 {} 由 {} 写，{} 没有声明写它", change.name,
                                            join(writers), writer));
        }
        if (frozen_ && !definition.runtime) {
            return reject(ParamCommit::kRejectedStatic,
                          "参数 " + change.name + " 声明为运行中不改，已经开始运行");
        }
        ParamValue value = change.value;
        if (!coerce(definition, value)) {
            return reject(ParamCommit::kRejectedType, typeProblem(definition, value));
        }
        if (auto problem = rangeProblem(definition, value)) {
            return reject(ParamCommit::kRejectedValue, std::move(*problem));
        }
        next->values.insert_or_assign(change.name, std::move(value));
    }
    next->revision = base->revision + 1;
    ParamSnapshotPtr published = next;
    snapshot_ = std::move(next);
    return {.status = ParamCommit::kStored,
            .revision = published->revision,
            .detail = {},
            .snapshot = published,
            .previous = base};
}

ParamScope::ParamScope(const ParamSnapshotPtr* source, std::vector<ParamChange>* writes) noexcept
    : source_(source), writes_(writes), previous_(t_current_scope) {
    t_current_scope = this;
}

ParamScope::ParamScope(ParamSnapshotPtr snapshot, std::vector<ParamChange>* writes) noexcept
    : owned_(std::move(snapshot)), source_(&owned_), writes_(writes), previous_(t_current_scope) {
    t_current_scope = this;
}

ParamScope::~ParamScope() { t_current_scope = previous_; }

const ParamScope* ParamScope::current() noexcept { return t_current_scope; }

const ParamSnapshot& ParamScope::snapshot() const noexcept {
    static const ParamSnapshot kEmpty;
    return (source_ != nullptr && *source_) ? **source_ : kEmpty;
}

void ParamScope::write(ParamChange change) const {
    if (writes_ == nullptr) {
        throw std::logic_error("这里不能写参数（" + change.name + "）：只给了一份快照读");
    }
    writes_->push_back(std::move(change));
}

}  // namespace simpleslam::comm
