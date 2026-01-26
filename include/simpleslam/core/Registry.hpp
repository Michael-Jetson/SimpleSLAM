#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <typeinfo>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace SimpleSLAM {

// ============================================================
// 1) Node: value type (scalar / array / object) backed by shared_ptr
//    Public-facing "params" is unordered_map<string, Node> as you want.
// ============================================================

class Node {
public:
  using Object = std::unordered_map<std::string, Node>;
  using Array  = std::vector<Node>;

  enum class Type { Null, String, Bool, Int, Double, Object, Array };

  // null
  Node();

  // scalars
  Node(const char* s);
  Node(std::string s);
  Node(bool b);
  Node(int64_t i);
  Node(double d);

  // convenience: integral -> int64, floating -> double (excluding bool)
  template <typename Int,
            typename = std::enable_if_t<std::is_integral_v<Int> && !std::is_same_v<Int, bool>>>
  Node(Int i) : Node(static_cast<int64_t>(i)) {}

  template <typename F,
            typename = std::enable_if_t<std::is_floating_point_v<F>>>
  Node(F d) : Node(static_cast<double>(d)) {}

  // containers
  Node(Object o);
  Node(Array a);

  // queries
  Type type() const;
  bool is_null()   const { return type() == Type::Null; }
  bool is_string() const { return type() == Type::String; }
  bool is_bool()   const { return type() == Type::Bool; }
  bool is_int()    const { return type() == Type::Int; }
  bool is_double() const { return type() == Type::Double; }
  bool is_object() const { return type() == Type::Object; }
  bool is_array()  const { return type() == Type::Array; }

  // accessors (throw on mismatch)
  const std::string& as_string() const;
  bool as_bool() const;
  int64_t as_int() const;
  double as_double() const;
  double as_number() const;  // accept int or double
  const Object& as_object() const;
  const Array& as_array() const;

  // minimal typename for error messages
  std::string type_name() const;

private:
  struct Data;
  std::shared_ptr<const Data> data_;
};

// Node internal storage (defined after Node is complete)
struct Node::Data {
  using Value = std::variant<std::monostate, std::string, bool, int64_t, double, Object, Array>;
  Value v;
  explicit Data(Value vv) : v(std::move(vv)) {}
};

// ---- Node implementations (inline, header-only) ----

inline Node::Node() : data_(std::make_shared<Data>(Data::Value{std::monostate{}})) {}

inline Node::Node(const char* s)
  : data_(std::make_shared<Data>(Data::Value{std::string(s ? s : "")})) {}

inline Node::Node(std::string s)
  : data_(std::make_shared<Data>(Data::Value{std::move(s)})) {}

inline Node::Node(bool b)
  : data_(std::make_shared<Data>(Data::Value{b})) {}

inline Node::Node(int64_t i)
  : data_(std::make_shared<Data>(Data::Value{i})) {}

inline Node::Node(double d)
  : data_(std::make_shared<Data>(Data::Value{d})) {}

inline Node::Node(Object o)
  : data_(std::make_shared<Data>(Data::Value{std::move(o)})) {}

inline Node::Node(Array a)
  : data_(std::make_shared<Data>(Data::Value{std::move(a)})) {}

inline Node::Type Node::type() const {
  const auto& v = data_->v;
  if (std::holds_alternative<std::monostate>(v)) return Type::Null;
  if (std::holds_alternative<std::string>(v))    return Type::String;
  if (std::holds_alternative<bool>(v))           return Type::Bool;
  if (std::holds_alternative<int64_t>(v))        return Type::Int;
  if (std::holds_alternative<double>(v))         return Type::Double;
  if (std::holds_alternative<Object>(v))         return Type::Object;
  if (std::holds_alternative<Array>(v))          return Type::Array;
  return Type::Null;
}

inline std::string Node::type_name() const {
  switch (type()) {
    case Type::Null:   return "null";
    case Type::String: return "string";
    case Type::Bool:   return "bool";
    case Type::Int:    return "int";
    case Type::Double: return "double";
    case Type::Object: return "object";
    case Type::Array:  return "array";
  }
  return "unknown";
}

inline const std::string& Node::as_string() const {
  if (!is_string()) throw std::runtime_error("Node: expected string, actual=" + type_name());
  return std::get<std::string>(data_->v);
}

inline bool Node::as_bool() const {
  if (!is_bool()) throw std::runtime_error("Node: expected bool, actual=" + type_name());
  return std::get<bool>(data_->v);
}

inline int64_t Node::as_int() const {
  if (!is_int()) throw std::runtime_error("Node: expected int, actual=" + type_name());
  return std::get<int64_t>(data_->v);
}

inline double Node::as_double() const {
  if (!is_double()) throw std::runtime_error("Node: expected double, actual=" + type_name());
  return std::get<double>(data_->v);
}

inline double Node::as_number() const {
  if (is_double()) return as_double();
  if (is_int()) return static_cast<double>(as_int());
  throw std::runtime_error("Node: expected number(int/double), actual=" + type_name());
}

inline const Node::Object& Node::as_object() const {
  if (!is_object()) throw std::runtime_error("Node: expected object, actual=" + type_name());
  return std::get<Object>(data_->v);
}

inline const Node::Array& Node::as_array() const {
  if (!is_array()) throw std::runtime_error("Node: expected array, actual=" + type_name());
  return std::get<Array>(data_->v);
}

// ============================================================
// 2) Params type (what you pass to constructors and factory)
//    This is exactly an unordered_map<string, Node>.
// ============================================================

using Params = Node::Object;

// Convenience builders (optional, to make code less verbose)
inline Node::Array Arr(std::initializer_list<Node> items) {
  return Node::Array(items);
}
inline Params Obj(std::initializer_list<std::pair<std::string, Node>> items) {
  Params o;
  o.reserve(items.size());
  for (const auto& kv : items) o.emplace(kv.first, kv.second);
  return o;
}

// Minimal key lookup helpers (no advanced error定位)
inline const Node* Find(const Params& p, const std::string& key) {
  auto it = p.find(key);
  return (it == p.end()) ? nullptr : &it->second;
}

inline const Node& Require(const std::string& owner_type,
                           const std::string& owner_name,
                           const Params& p,
                           const std::string& key) {
  auto it = p.find(key);
  if (it == p.end()) {
    throw std::runtime_error("Missing key '" + key + "' while creating " + owner_type + "(" + owner_name + ")");
  }
  return it->second;
}

// ============================================================
// 3) Base classes
// ============================================================

class ModuleBase {
public:
  // Lifecycle states for diagnostics and basic control.
  enum class State {
    kCreated,
    kInitialized,
    kRunning,
    kStopped,
    kError,
    kShutdown
  };

  // Scheduling hint for the runtime/executor.
  enum class ExecMode {
    kPassive,
    kActive
  };

  ModuleBase(const ModuleBase&) = delete;
  ModuleBase& operator=(const ModuleBase&) = delete;
  ModuleBase(ModuleBase&&) = delete;
  ModuleBase& operator=(ModuleBase&&) = delete;

  virtual ~ModuleBase() = default;

  // Identity and config as provided by the factory/Builder.
  const std::string& name() const noexcept { return name_; }
  const Params& params() const noexcept { return params_; }

  // RTTI-based type name is used mainly for logging and diagnostics.
  virtual std::string_view type_name() const noexcept { return typeid(*this).name(); }
  // Default is passive; active modules are explicitly scheduled by runtime.
  virtual ExecMode exec_mode() const noexcept { return ExecMode::kPassive; }

  // Lifecycle hooks; runtime calls these in order when available.
  virtual bool Configure() { return true; }
  virtual bool Init() { return true; }
  virtual bool Start() { return true; }
  virtual void Stop() {}
  virtual void Reset() {}
  virtual void Shutdown() {}

  // Updated by runtime/module; errors should set kError.
  State state() const noexcept { return state_; }
  const std::string& last_error() const noexcept { return last_error_; }

protected:
  // Modules are constructed by the factory with a name and params.
  explicit ModuleBase(std::string name, Params params)
      : name_(std::move(name)), params_(std::move(params)) {}

  void set_state(State s) noexcept { state_ = s; }
  // Record error and mark state as kError.
  void set_error(std::string msg) {
    last_error_ = std::move(msg);
    state_ = State::kError;
  }

  std::string name_;
  Params params_;
  State state_{State::kCreated};
  std::string last_error_;
};

class PluginBase {
public:
  // Thread-safety declaration for the runtime and module owners.
  enum class ThreadModel {
    kSingleThread,
    kThreadSafe,
    kExternalSync
  };

  PluginBase(const PluginBase&) = delete;
  PluginBase& operator=(const PluginBase&) = delete;
  PluginBase(PluginBase&&) = delete;
  PluginBase& operator=(PluginBase&&) = delete;

  virtual ~PluginBase() = default;

  // Identity and config as provided by the factory/Builder.
  const std::string& name() const noexcept { return name_; }
  const Params& params() const noexcept { return params_; }

  // RTTI-based type name is used mainly for logging and diagnostics.
  virtual std::string_view type_name() const noexcept { return typeid(*this).name(); }
  // Thread model hint to avoid unsafe concurrent calls.
  virtual ThreadModel thread_model() const noexcept { return ThreadModel::kSingleThread; }

  // Minimal lifecycle; modules decide how/when to use plugins.
  virtual bool Init() { return true; }
  virtual void Reset() {}
  virtual void Shutdown() {}

  // Error string for diagnostics (kept by plugin itself).
  const std::string& last_error() const noexcept { return last_error_; }

protected:
  // Plugins are constructed by the factory with a name and params.
  explicit PluginBase(std::string name, Params params)
      : name_(std::move(name)), params_(std::move(params)) {}

  // Record error without changing any external state.
  void set_error(std::string msg) { last_error_ = std::move(msg); }

  std::string name_;
  Params params_;
  std::string last_error_;
};

// ============================================================
// 4) RegistryFactory: type string -> creator(instance_name, Params)
// ============================================================

template <typename Base>
class RegistryFactory {
public:
  using Creator = std::function<std::unique_ptr<Base>(std::string instance_name, Params params)>;

  static RegistryFactory& Instance() {
    static RegistryFactory inst;
    return inst;
  }

  // default policy: throw on duplicate registration
  void RegisterOrThrow(const std::string& type, Creator creator) {
    std::lock_guard<std::mutex> lock(mu_);
    auto [it, inserted] = creators_.emplace(type, std::move(creator));
    if (!inserted) {
      throw std::runtime_error("RegistryFactory: duplicate registration for type: " + type);
    }
  }

  // optional: override policy (not used by default macros)
  void RegisterOrOverride(const std::string& type, Creator creator) {
    std::lock_guard<std::mutex> lock(mu_);
    creators_[type] = std::move(creator);
  }

  std::unique_ptr<Base> Create(const std::string& type, std::string instance_name, Params params) const {
    Creator creator;
    {
      std::lock_guard<std::mutex> lock(mu_);
      auto it = creators_.find(type);
      if (it == creators_.end()) {
        throw std::runtime_error("RegistryFactory: unknown type: " + type);
      }
      creator = it->second; // copy out; call without holding lock
    }
    return creator(std::move(instance_name), std::move(params));
  }

private:
  RegistryFactory() = default;

  mutable std::mutex mu_;
  std::unordered_map<std::string, Creator> creators_;
};

using ModuleFactory = RegistryFactory<ModuleBase>;
using PluginFactory = RegistryFactory<PluginBase>;

// ============================================================
// 5) Header-only registration helpers (C++17 inline variables)
//    Goal: even if included/linked multiple times, register once per program image.
// ============================================================

namespace detail {

template <typename Base, typename Derived>
inline bool RegisterTypeOrThrow(const char* type_str) {
  static_assert(std::is_base_of_v<Base, Derived>, "Derived must inherit from Base");
  static_assert(std::is_constructible_v<Derived, std::string, Params>,
                "Derived must be constructible as Derived(std::string instance_name, Params params)");

  RegistryFactory<Base>::Instance().RegisterOrThrow(
    type_str,
    [](std::string instance_name, Params params) -> std::unique_ptr<Base> {
      return std::make_unique<Derived>(std::move(instance_name), std::move(params));
    }
  );
  return true;
}

template <typename Base, typename Derived>
inline bool RegisterTypeOrOverride(const char* type_str) {
  static_assert(std::is_base_of_v<Base, Derived>, "Derived must inherit from Base");
  static_assert(std::is_constructible_v<Derived, std::string, Params>,
                "Derived must be constructible as Derived(std::string instance_name, Params params)");

  RegistryFactory<Base>::Instance().RegisterOrOverride(
    type_str,
    [](std::string instance_name, Params params) -> std::unique_ptr<Base> {
      return std::make_unique<Derived>(std::move(instance_name), std::move(params));
    }
  );
  return true;
}

} // namespace detail

// Default macros: type string == class name literal (#CLASS).
// Works when CLASS is an unqualified identifier (e.g., Frontend).
#define SIMPLESLAM_REGISTER_MODULE(CLASS) \
  namespace SimpleSLAM::reg { \
    inline const bool reg_module_##CLASS = \
      ::SimpleSLAM::detail::RegisterTypeOrThrow<::SimpleSLAM::ModuleBase, CLASS>(#CLASS); \
  }

#define SIMPLESLAM_REGISTER_PLUGIN(CLASS) \
  namespace SimpleSLAM::reg { \
    inline const bool reg_plugin_##CLASS = \
      ::SimpleSLAM::detail::RegisterTypeOrThrow<::SimpleSLAM::PluginBase, CLASS>(#CLASS); \
  }

// Namespaced/explicit versions:
// TYPE_STR: registry key (should match config "type")
// QUALIFIED_CLASS: e.g. myslam::Frontend
// TAG: a simple identifier for the inline variable name
#define SIMPLESLAM_REGISTER_MODULE_AS(TYPE_STR, QUALIFIED_CLASS, TAG) \
  namespace SimpleSLAM::reg { \
    inline const bool reg_module_##TAG = \
      ::SimpleSLAM::detail::RegisterTypeOrThrow<::SimpleSLAM::ModuleBase, QUALIFIED_CLASS>(TYPE_STR); \
  }

#define SIMPLESLAM_REGISTER_PLUGIN_AS(TYPE_STR, QUALIFIED_CLASS, TAG) \
  namespace SimpleSLAM::reg { \
    inline const bool reg_plugin_##TAG = \
      ::SimpleSLAM::detail::RegisterTypeOrThrow<::SimpleSLAM::PluginBase, QUALIFIED_CLASS>(TYPE_STR); \
  }

// Optional override macros (if you ever want last-wins):
#define SIMPLESLAM_REGISTER_MODULE_OVERRIDE(CLASS) \
  namespace SimpleSLAM::reg { \
    inline const bool reg_module_override_##CLASS = \
      ::SimpleSLAM::detail::RegisterTypeOrOverride<::SimpleSLAM::ModuleBase, CLASS>(#CLASS); \
  }

#define SIMPLESLAM_REGISTER_PLUGIN_OVERRIDE(CLASS) \
  namespace SimpleSLAM::reg { \
    inline const bool reg_plugin_override_##CLASS = \
      ::SimpleSLAM::detail::RegisterTypeOrOverride<::SimpleSLAM::PluginBase, CLASS>(#CLASS); \
  }

} // namespace SimpleSLAM
