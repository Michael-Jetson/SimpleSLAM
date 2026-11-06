// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file plugin.hpp
/// 插件槽与目录：配置里"type + 参数"的节点，在装配时按目录还原成具体实现。
///
/// - 槽（`Slot<I>`）只写它要求的那一个接口 I，不列实现。满足 I 的实现都能选。
/// - 目录（`Catalog<I>`）每个接口一张，把类型名对到实现。内置实现由框架登记，
///   用户的实现在应用入口追加，不改框架代码。登记时编译器检查实现继承了 I、
///   带类型名、能用 `const Config&` 构造（`PluginImplementation`）。
/// - 每个实现带一个配置聚合体 `Config` 和一份字段清单 `Config::visit`，写法同
///   `visitConfig`：`v("名字", 字段, "说明")`。还原时据此严格校验：未知参数、
///   类型不符一律报错，没写的字段取成员默认值；字段是 `Slot<J>` 时递归还原。
/// - 还原时试构造一次实现，所以实现构造函数里的取值检查也在装配时报错，
///   不会拖到运行中。试构造期间的日志不记。
/// - 槽构造实现时把日志来源交给它（`logging/construction.hpp`）：顶层实现挂在模块下面，
///   子槽挂在父插件下面，所以实现在构造函数里就能用 `log_`。
///
/// 只依赖标准库与日志库：算法库包含它，不会连带依赖通信层。

#include <SimpleSLAM/config/error.hpp>
#include <SimpleSLAM/logging/construction.hpp>

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <typeindex>
#include <typeinfo>
#include <utility>
#include <variant>
#include <vector>

namespace simpleslam::config {

/// 插件参数的值。和配置里其他标量一样只有四种，不做字符串到数的隐式转换。
using PluginValue = std::variant<bool, std::int64_t, double, std::string>;

/// 配置里的一个插件节点。带 `type` 的嵌套映射是子槽，其余键是参数。
struct PluginSpec {
    std::string type;
    std::map<std::string, PluginValue> params;
    std::map<std::string, PluginSpec> children;

    friend bool operator==(const PluginSpec&, const PluginSpec&) = default;
};

/// 一组带名字的插件节点：名字 → 节点。评测函数这样列：同一种可以并排几份不同参数。
using PluginMap = std::map<std::string, PluginSpec>;

/// schema 里的一个字段：参数或子槽。
struct PluginField {
    std::string name;
    std::string kind;           ///< bool / int / float / double / string / slot:<接口名>
    /// 默认值的文本，口径同配置 schema；子槽写默认实现名，没有默认就空着。
    std::string default_value;
    std::string doc;
    /// 子槽的完整默认节点（类型加补齐的参数）。默认实现可以带非默认参数，
    /// 只给名字的话，按 schema 还原的一方（Python）就补不对。
    PluginSpec default_spec{};
};

/// 一个实现的 schema：导出给 Python、文档和命令行。
struct PluginSchema {
    std::string interface_name;
    std::string type;
    std::vector<PluginField> fields;
};

/// 接口要声明自己的名字，报错与 schema 都用它。
template <class I>
concept PluginInterface = requires {
    { I::kInterfaceName } -> std::convertible_to<std::string_view>;
};

/// 能登记进接口 I 的目录：继承 I、带类型名、配置聚合体能默认构造并用来构造实现。
template <class T, class I>
concept PluginImplementation =
    PluginInterface<I> && std::derived_from<T, I> &&
    std::default_initializable<typename T::Config> &&
    std::constructible_from<T, const typename T::Config&> && requires {
        { T::kTypeName } -> std::convertible_to<std::string_view>;
    };

class Catalogs;

template <class I>
class Catalog;

namespace detail {
class StampVisitor;
}  // namespace detail

/// 一个槽：选定的实现、它还原后的完整参数，以及构造它的办法。
///
/// 值类型，可以拷贝；`make()` 每次构造一个新实例，实例的寿命归调用者。
template <class I>
class Slot {
public:
    using Interface = I;

    Slot() = default;

    /// 不经 YAML、直接在 C++ 里选实现（L2、L3 用法）。
    template <class T>
        requires PluginImplementation<T, I>
    [[nodiscard]] static Slot of(typename T::Config config = {});

    [[nodiscard]] bool empty() const noexcept { return maker_ == nullptr; }
    [[nodiscard]] const std::string& type() const noexcept { return spec_.type; }
    /// 还原后的完整参数，已补齐默认值。规范化导出用它。
    [[nodiscard]] const PluginSpec& resolved() const noexcept { return spec_; }

    /// 构造一个实例。它的日志来源挂在父插件下面：父插件交来的槽记着父插件是谁，构造函数里
    /// 新建的挂在正在构造的那个对象下面；都不是就是没接上的，交给模块之后由模块接上。
    [[nodiscard]] std::unique_ptr<I> make() const {
        return build(parent_ != nullptr ? parent_ : logging::detail::ConstructionScope::current(),
                     field_);
    }

    /// 构造一个顶层实例，日志来源挂在 `parent`（模块或执行器）下面；`field` 是它在按来源设
    /// 等级的键里的那一段，与配置里的字段名相同（如 `plugin`）。
    [[nodiscard]] std::unique_ptr<I> make(const logging::Logger& parent,
                                          std::string_view field) const {
        return build(logging::detail::nodeOf(parent), field);
    }

private:
    using Maker = std::function<std::unique_ptr<I>(const logging::detail::NodePtr&)>;

    template <class>
    friend class Catalog;
    friend class detail::StampVisitor;

    static Slot create(PluginSpec spec, Maker maker) {
        Slot s;
        s.spec_ = std::move(spec);
        s.maker_ = std::make_shared<const Maker>(std::move(maker));
        return s;
    }

    [[nodiscard]] std::unique_ptr<I> build(const logging::detail::NodePtr& parent,
                                           std::string_view field) const {
        if (!maker_) {
            throw ConfigError("槽是空的：还没有选实现（" + std::string(I::kInterfaceName) + "）");
        }
        return (*maker_)(logging::detail::componentNode(parent, field, spec_.type));
    }

    /// 父插件构造之前，它配置里的子槽记下父插件是谁、自己叫什么。
    void stamp(logging::detail::NodePtr parent, std::string_view field) {
        parent_ = std::move(parent);
        field_ = std::string(field);
    }

    PluginSpec spec_;
    std::shared_ptr<const Maker> maker_;
    logging::detail::NodePtr parent_;
    std::string field_;
};

namespace detail {

template <class T>
struct IsSlot : std::false_type {};
template <class J>
struct IsSlot<Slot<J>> : std::true_type {};

inline std::string kindOf(const PluginValue& v) {
    switch (v.index()) {
        case 0: return "bool";
        case 1: return "int";
        case 2: return "double";
        default: return "string";
    }
}

template <class F>
std::string kindOfField() {
    if constexpr (IsSlot<F>::value) {
        return "slot:" + std::string(F::Interface::kInterfaceName);
    } else if constexpr (std::is_same_v<F, bool>) {
        return "bool";
    } else if constexpr (std::is_integral_v<F>) {
        return "int";
    } else if constexpr (std::is_same_v<F, float>) {
        // 单精度字段在赋值时就舍入了，还原后的节点存的是舍入后的值。schema 说出来，
        // 读 schema 的人才知道写进去的值会被舍入
        return "float";
    } else if constexpr (std::is_floating_point_v<F>) {
        return "double";
    } else {
        static_assert(std::is_same_v<F, std::string>,
                      "插件配置字段只能是 bool、整数、浮点、std::string 或 Slot<接口>");
        return "string";
    }
}

/// schema 与报错里的值文本，与 `config/loader.hpp` 的 schema 同一口径：双精度按 `%.17g`
/// 往返无损地写（不用会丢精度的 std::to_string），字符串原样写。
inline std::string defaultText(const PluginValue& v) {
    struct Renderer {
        std::string operator()(bool b) const { return b ? "true" : "false"; }
        std::string operator()(std::int64_t i) const { return std::to_string(i); }
        std::string operator()(double d) const {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.17g", d);
            return buf;
        }
        std::string operator()(const std::string& s) const { return s; }
    };
    return std::visit(Renderer{}, v);
}

inline std::string join(const std::vector<std::string>& items) {
    std::string out;
    for (const auto& item : items) out += (out.empty() ? "" : "、") + item;
    return out;
}

template <class F>
PluginValue toValue(const F& field) {
    if constexpr (std::is_same_v<F, bool>) {
        return field;
    } else if constexpr (std::is_integral_v<F>) {
        return static_cast<std::int64_t>(field);
    } else if constexpr (std::is_floating_point_v<F>) {
        return static_cast<double>(field);
    } else {
        return std::string(field);
    }
}

template <class F>
void assign(F& field, const PluginValue& value, const std::string& path) {
    const auto mismatch = [&] {
        return ConfigError(path + "：应为 " + kindOfField<F>() + "，实际是 " + kindOf(value));
    };
    if constexpr (std::is_same_v<F, bool>) {
        const bool* b = std::get_if<bool>(&value);
        if (b == nullptr) throw mismatch();
        field = *b;
    } else if constexpr (std::is_integral_v<F>) {
        const std::int64_t* i = std::get_if<std::int64_t>(&value);
        if (i == nullptr) throw mismatch();
        bool in_range = true;
        if constexpr (std::is_signed_v<F>) {
            in_range = *i >= static_cast<std::int64_t>(std::numeric_limits<F>::min()) &&
                       *i <= static_cast<std::int64_t>(std::numeric_limits<F>::max());
        } else {
            in_range = *i >= 0 && static_cast<std::uint64_t>(*i) <= std::numeric_limits<F>::max();
        }
        if (!in_range) {
            throw ConfigError(path + "：" + std::to_string(*i) + " 超出这个字段的取值范围");
        }
        field = static_cast<F>(*i);
    } else if constexpr (std::is_floating_point_v<F>) {
        double d = 0.0;
        if (const double* p = std::get_if<double>(&value)) {
            d = *p;
        } else if (const std::int64_t* i = std::get_if<std::int64_t>(&value)) {
            d = static_cast<double>(*i);  // YAML 里的 100 与 100.0 是同一个量
        } else {
            throw mismatch();
        }
        if (!std::isfinite(d)) throw ConfigError(path + "：不是有限数");
        // 超出 float 范围的转换是未定义行为，所以先查；对 double 这一条永远不成立
        if (d > static_cast<double>(std::numeric_limits<F>::max()) ||
            d < static_cast<double>(std::numeric_limits<F>::lowest())) {
            throw ConfigError(path + "：" + defaultText(value) + " 超出这个字段的取值范围");
        }
        field = static_cast<F>(d);
    } else {
        const std::string* s = std::get_if<std::string>(&value);
        if (s == nullptr) throw mismatch();
        field = *s;
    }
}

/// 把一个插件节点按字段清单还原进 Config。
class ParseVisitor {
public:
    ParseVisitor(const PluginSpec& spec, const Catalogs& catalogs, std::string path,
                 std::string type)
        : spec_(spec), catalogs_(catalogs), path_(std::move(path)), type_(std::move(type)) {}

    template <class F>
    void operator()(std::string_view name, F& field, std::string_view /*doc*/);

    /// 字段清单走完之后调用：剩下没用到的键就是拼错或不属于这个实现的参数。
    void finish() const {
        for (const auto& [key, value] : spec_.params) {
            if (used_.count(key) == 0) throw unknown(key);
        }
        for (const auto& [key, child] : spec_.children) {
            if (used_.count(key) == 0) throw unknown(key);
        }
    }

private:
    [[nodiscard]] ConfigError unknown(const std::string& key) const {
        std::string msg = path_ + "." + key + "：" + type_ + " 没有这个参数";
        msg += known_.empty() ? "，它不接受任何参数" : "，可用的有 " + join(known_);
        return ConfigError(msg);
    }

    const PluginSpec& spec_;
    const Catalogs& catalogs_;
    std::string path_;
    std::string type_;
    std::vector<std::string> known_;
    std::set<std::string> used_;
};

/// 把 Config 导出成规范化的插件节点（含默认值）。
class ExportVisitor {
public:
    explicit ExportVisitor(PluginSpec& out) : out_(out) {}

    template <class F>
    void operator()(std::string_view name, F& field, std::string_view /*doc*/) {
        const std::string key(name);
        if constexpr (IsSlot<F>::value) {
            if (!field.empty()) out_.children[key] = field.resolved();
        } else {
            out_.params[key] = toValue(field);
        }
    }

private:
    PluginSpec& out_;
};

/// 列出 Config 的字段与默认值。
class SchemaVisitor {
public:
    explicit SchemaVisitor(std::vector<PluginField>& out) : out_(out) {}

    template <class F>
    void operator()(std::string_view name, F& field, std::string_view doc) {
        PluginField f{.name = std::string(name),
                      .kind = kindOfField<F>(),
                      .default_value = {},
                      .doc = std::string(doc)};
        if constexpr (IsSlot<F>::value) {
            f.default_value = field.type();
            f.default_spec = field.resolved();
        } else {
            f.default_value = defaultText(toValue(field));
        }
        out_.push_back(std::move(f));
    }

private:
    std::vector<PluginField>& out_;
};

template <class Config>
PluginSpec exportSpec(Config config, std::string type) {
    PluginSpec spec;
    spec.type = std::move(type);
    ExportVisitor v(spec);
    config.visit(v);
    return spec;
}

/// 给 Config 里的子槽记上父插件：父插件什么时候用它们构造子插件，来源都挂在父插件下面。
class StampVisitor {
public:
    explicit StampVisitor(logging::detail::NodePtr parent) : parent_(std::move(parent)) {}

    template <class F>
    void operator()(std::string_view name, F& field, std::string_view /*doc*/) {
        if constexpr (IsSlot<F>::value) field.stamp(parent_, name);
    }

private:
    logging::detail::NodePtr parent_;
};

/// 构造实现的办法：子槽先记下父插件，再在构造期上下文里构造，基类从里面取自己的来源。
template <class I, class T>
auto makerFor(typename T::Config config) {
    return [config = std::move(config)](
               const logging::detail::NodePtr& node) -> std::unique_ptr<I> {
        typename T::Config stamped = config;
        StampVisitor stamp(node);
        stamped.visit(stamp);
        const logging::detail::ConstructionScope scope(node);
        return std::make_unique<T>(stamped);
    };
}

}  // namespace detail

/// 目录的类型无关部分：报错提示与 schema 导出要跨接口遍历。
class CatalogBase {
public:
    virtual ~CatalogBase() = default;
    [[nodiscard]] virtual std::string_view interfaceName() const noexcept = 0;
    [[nodiscard]] virtual std::vector<std::string> names() const = 0;
    [[nodiscard]] virtual bool contains(std::string_view type) const = 0;
    [[nodiscard]] virtual std::vector<PluginSchema> schemas() const = 0;
};

/// 一个接口的目录：类型名 → 实现。
template <class I>
class Catalog final : public CatalogBase {
    static_assert(PluginInterface<I>,
                  "接口要声明 static constexpr std::string_view kInterfaceName");

public:
    /// 登记一个实现，名字取它的 `kTypeName`。名字重复是错误。
    template <class T>
        requires PluginImplementation<T, I>
    void add() {
        add<T>(std::string(T::kTypeName));
    }

    template <class T>
        requires PluginImplementation<T, I>
    void add(const std::string& name);

    /// 按 `spec.type` 找到实现，把参数还原成它的 Config。`path` 是这个节点在配置里的位置。
    [[nodiscard]] Slot<I> resolve(const PluginSpec& spec, const Catalogs& catalogs,
                                  const std::string& path) const;

    [[nodiscard]] std::string_view interfaceName() const noexcept override {
        return I::kInterfaceName;
    }

    [[nodiscard]] std::vector<std::string> names() const override {
        std::vector<std::string> out;
        for (const auto& [name, entry] : entries_) out.push_back(name);
        return out;
    }

    [[nodiscard]] bool contains(std::string_view type) const override {
        return entries_.find(type) != entries_.end();
    }

    [[nodiscard]] std::vector<PluginSchema> schemas() const override {
        std::vector<PluginSchema> out;
        for (const auto& [name, entry] : entries_) out.push_back(entry.schema());
        return out;
    }

private:
    struct Entry {
        std::function<Slot<I>(const PluginSpec&, const Catalogs&, const std::string&)> resolve;
        std::function<PluginSchema()> schema;
    };
    std::map<std::string, Entry, std::less<>> entries_;
};

/// 全部接口的目录。应用入口建一个，登记内置实现，再追加用户实现，装配时只读。
///
/// 不是全局单例：同一进程里可以有几套互不相干的目录（比如测试里各建一个）。
class Catalogs {
public:
    template <class I>
        requires PluginInterface<I>
    Catalog<I>& of() {
        auto& slot = catalogs_[std::type_index(typeid(I))];
        if (!slot) slot = std::make_shared<Catalog<I>>();
        return static_cast<Catalog<I>&>(*slot);
    }

    template <class I>
        requires PluginInterface<I>
    [[nodiscard]] const Catalog<I>& of() const {
        const auto it = catalogs_.find(std::type_index(typeid(I)));
        if (it == catalogs_.end()) {
            throw ConfigError("没有 " + std::string(I::kInterfaceName) + " 接口的目录");
        }
        return static_cast<const Catalog<I>&>(*it->second);
    }

    /// 还原一个顶层槽。`path` 是它在配置里的位置，比如 `modules.odometry.plugin`。
    template <class I>
        requires PluginInterface<I>
    [[nodiscard]] Slot<I> resolve(const PluginSpec& spec, const std::string& path) const {
        return of<I>().resolve(spec, *this, path);
    }

    /// 哪些接口的目录里有这个类型名，按接口名排序。选错接口时给出提示用。
    [[nodiscard]] std::vector<std::string> interfacesProviding(std::string_view type) const {
        std::vector<std::string> out;
        for (const auto& [index, catalog] : catalogs_) {
            if (catalog->contains(type)) out.emplace_back(catalog->interfaceName());
        }
        std::sort(out.begin(), out.end());
        return out;
    }

    /// 全部实现的 schema，按接口名、类型名排序。
    [[nodiscard]] std::vector<PluginSchema> schemas() const {
        std::vector<PluginSchema> out;
        for (const auto& [index, catalog] : catalogs_) {
            auto part = catalog->schemas();
            out.insert(out.end(), part.begin(), part.end());
        }
        std::sort(out.begin(), out.end(), [](const PluginSchema& a, const PluginSchema& b) {
            return std::tie(a.interface_name, a.type) < std::tie(b.interface_name, b.type);
        });
        return out;
    }

private:
    std::map<std::type_index, std::shared_ptr<CatalogBase>> catalogs_;
};

// ── 模板成员的定义：要用到完整的 Catalogs ──

template <class I>
template <class T>
    requires PluginImplementation<T, I>
Slot<I> Slot<I>::of(typename T::Config config) {
    PluginSpec spec = detail::exportSpec(config, std::string(T::kTypeName));
    return create(std::move(spec), detail::makerFor<I, T>(std::move(config)));
}

template <class F>
void detail::ParseVisitor::operator()(std::string_view name, F& field, std::string_view /*doc*/) {
    const std::string key(name);
    known_.push_back(key);
    const std::string where = path_ + "." + key;
    if constexpr (IsSlot<F>::value) {
        const auto it = spec_.children.find(key);
        if (it == spec_.children.end()) {
            if (spec_.params.count(key) != 0) {
                throw ConfigError(where + "：这是一个槽，要写成带 type 的映射");
            }
            if (field.empty()) throw ConfigError(where + "：缺少这个槽，要用 type 选一个实现");
            return;  // 没写就用默认实现
        }
        used_.insert(key);
        field = catalogs_.template resolve<typename F::Interface>(it->second, where);
    } else {
        const auto it = spec_.params.find(key);
        if (it == spec_.params.end()) {
            if (spec_.children.count(key) != 0) throw ConfigError(where + "：这是一个参数，不是槽");
            return;  // 没写就用成员默认值
        }
        used_.insert(key);
        detail::assign(field, it->second, where);
    }
}

template <class I>
template <class T>
    requires PluginImplementation<T, I>
void Catalog<I>::add(const std::string& name) {
    if (entries_.count(name) != 0) {
        throw ConfigError(std::string(I::kInterfaceName) + " 的目录里已经有 " + name);
    }
    Entry entry;
    entry.resolve = [name](const PluginSpec& spec, const Catalogs& catalogs,
                           const std::string& path) {
        typename T::Config config{};
        detail::ParseVisitor visitor(spec, catalogs, path, name);
        config.visit(visitor);
        visitor.finish();
        try {
            const logging::detail::MutedConstruction muted;  // 试构造只为提前报错，日志不记
            T probe(config);  // 构造函数里的取值检查在这里就报，而不是运行中
            (void)probe;
        } catch (const ConfigError&) {
            throw;
        } catch (const std::exception& e) {
            throw ConfigError(path + "（" + name + "）：" + e.what());
        }
        PluginSpec resolved = detail::exportSpec(config, name);
        return Slot<I>::create(std::move(resolved), detail::makerFor<I, T>(config));
    };
    entry.schema = [name] {
        PluginSchema schema{
            .interface_name = std::string(I::kInterfaceName), .type = name, .fields = {}};
        typename T::Config config{};
        detail::SchemaVisitor visitor(schema.fields);
        config.visit(visitor);
        return schema;
    };
    entries_.emplace(name, std::move(entry));
}

template <class I>
Slot<I> Catalog<I>::resolve(const PluginSpec& spec, const Catalogs& catalogs,
                            const std::string& path) const {
    const std::string interface_name(I::kInterfaceName);
    if (spec.type.empty()) {
        throw ConfigError(path + ".type：没有写 type。这里要求 " + interface_name +
                          "，可选的实现有 " + detail::join(names()));
    }
    const auto it = entries_.find(spec.type);
    if (it == entries_.end()) {
        std::string msg = path + ".type = " + spec.type + "：这里要求 " + interface_name;
        const auto others = catalogs.interfacesProviding(spec.type);
        if (!others.empty()) msg += "，" + spec.type + " 是 " + detail::join(others) + " 的实现";
        msg += names().empty() ? "，目录里还没有任何实现"
                               : "；满足的实现有 " + detail::join(names());
        throw ConfigError(msg);
    }
    return it->second.resolve(spec, catalogs, path);
}

}  // namespace simpleslam::config
