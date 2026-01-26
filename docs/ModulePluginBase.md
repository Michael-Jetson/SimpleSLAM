# 模块与插件基类说明

本文档说明 `include/simpleslam/core/Registry.hpp` 中新增的模块/插件基类含义、使用约定，
以及 `include/simpleslam/modules/ModuleBased.hpp` 和 `include/simpleslam/plugins/PluginBased.hpp`
中的轻量封装。目标是让 Builder、运行时调度器和开发者对行为有一致预期。

## 范围

覆盖类型：

- `SimpleSLAM::ModuleBase`
- `SimpleSLAM::PluginBase`
- `SimpleSLAM::IModule`（薄封装）
- `SimpleSLAM::IPlugin`（薄封装）

相关构造类型：

- `SimpleSLAM::Params`（`std::unordered_map<std::string, Node>` 的别名）
- `SimpleSLAM::RegistryFactory`

## 设计意图

1. 模块是系统级功能单元（Tracking/Mapping/Backend 等）。
2. 插件是模块内部可替换的算法实现。
3. 二者都由注册表按 `type` 字符串创建，构造函数签名为 `(name, params)`。
4. 生命周期显式且最小化，运行时决定何时调用。
5. 线程安全与调度方式需要声明，而不是默认猜测。

## ModuleBase 约定

### 身份与配置

模块保存由工厂传入的实例名和参数：

- `name()` 返回实例名（来自配置的 `Name` 或模块 key）。
- `params()` 返回参数表（解析后的 `Params`）。

`type_name()` 默认使用 RTTI，仅用于日志和诊断，不应作为稳定 ABI 或配置 key。

### 生命周期

默认钩子为最小 no-op：

- `Configure()`  -> 构造后、资源绑定前调用。
- `Init()`       -> 资源/插件绑定完成后调用。
- `Start()`      -> 运行时启动模块时调用。
- `Stop()`       -> 停止执行时调用。
- `Reset()`      -> 不销毁对象的状态重置。
- `Shutdown()`   -> 最终清理。

默认实现返回成功或空实现，模块按需覆盖。运行时/Builder 应按序调用。

### 执行提示

`exec_mode()` 是调度提示：

- `ExecMode::kPassive` 被动响应消息。
- `ExecMode::kActive`  需要运行时周期调度/显式 tick。

它不是硬规则，而是模块与执行器之间的契约。

### 状态与错误

`state()` 与 `last_error()` 是基本诊断接口。模块可通过 `set_error(...)`
记录错误并将状态置为 `kError`。

状态含义：

- `kCreated`     构造完成。
- `kInitialized` `Init()` 成功后。
- `kRunning`     `Start()` 成功后。
- `kStopped`     `Stop()` 后。
- `kError`       出错并记录 `last_error()`。
- `kShutdown`    `Shutdown()` 后。

运行时应在生命周期转换时更新 `state()`。

### 构造约定（重要）

所有可注册模块必须提供构造函数：

```
Derived(std::string instance_name, Params params)
```

基类构造是 `protected`，负责存储 name/params。注册表强制该签名，
否则无法注册与创建。

## PluginBase 约定

### 身份与配置

插件同样保存 `name()` 与 `params()`；`type_name()` 用于诊断输出。

### 生命周期

插件生命周期更轻量：

- `Init()`     -> 由模块或运行时调用。
- `Reset()`    -> 可选重置。
- `Shutdown()` -> 最终清理。

### 线程模型声明

`thread_model()` 表达线程安全契约：

- `kSingleThread`  只能串行调用。
- `kThreadSafe`    可并发调用。
- `kExternalSync`  由调用者外部加锁。

模块调度插件时必须遵守此声明，避免数据竞争。

### 错误记录

`last_error()` 用于诊断，插件可用 `set_error(...)` 写入错误信息。
基类不改变状态，如何处理由调用方决定。

### 构造约定（重要）

所有可注册插件必须提供：

```
Derived(std::string instance_name, Params params)
```

## IModule / IPlugin 轻量封装

`IModule` 与 `IPlugin` 只是薄封装，继承基类且不新增行为，
用于统一命名风格与便于模块/插件头文件书写。

## 使用模式

最小模块示例：

```cpp
class MyTracker : public SimpleSLAM::IModule {
public:
  MyTracker(std::string name, SimpleSLAM::Params params)
      : IModule(std::move(name), std::move(params)) {}

  bool Init() override {
    // 解析参数、绑定资源等
    return true;
  }
};
```

最小插件示例：

```cpp
class MyMatcher : public SimpleSLAM::IPlugin {
public:
  MyMatcher(std::string name, SimpleSLAM::Params params)
      : IPlugin(std::move(name), std::move(params)) {}
};
```

## 运行时/Builder 侧约定

- 使用 `RegistryFactory` 通过 `type` 创建模块/插件实例。
- 绑定与校验依赖后再调用 `Init()`。
- 根据 `exec_mode()` 与 `thread_model()` 做调度决策。
- 出错时读取 `last_error()` 统一上报。

## 模块/插件作者约定

- 构造函数只做轻量操作；重活放在 `Init()` 或 `Configure()`。
- 析构函数不抛异常，失败用返回值或 `set_error(...)` 表达。
- 不要虚报 `kThreadSafe`，否则后续难以定位并发问题。

## 当前非目标

- 依赖槽位系统尚未在这里实现。
- `Params` 的 schema 校验未在此处强制。
- 并发原语不由基类提供，模块自行管理内部锁。

这些能力可以在后续 Builder/Executor 中叠加实现。
