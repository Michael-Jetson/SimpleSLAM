// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file logger.hpp
/// 日志对象：模块与插件的成员 `log_`，写法是 `log_.info("内点 {} 个", n)`。
///
/// 写日志的人只决定等级与正文，其余都是自动的：
/// - 来源跟着装配路径走：模块名 › 实现类型 › 子槽(实现类型)。按来源设等级用的键与配置路径
///   同一种写法（`odometry.plugin.map`）；
/// - 帧序号与数据时刻由所属模块在处理每条输入之前设上（`setFrame`），下面的插件沿来源读到；
/// - 时刻取运行时的时钟：离线是数据时间，在线是墙钟。
///
/// 去向由日志组件定（`router.hpp`），日志对象不碰终端和文件。同一个对象也记事件与度量
/// （`event`、`metric`、`timer`），它们交给诊断，一条不丢。
///
/// 成员名是 `log_` 而不是 `log`：派生类的成员函数里，叫 `log` 的成员会遮住数学函数，
/// 不加 `std::` 的 `log(x)` 就编译不过。

#include <chrono>
#include <concepts>
#include <cstdint>
#include <format>
#include <memory>
#include <optional>
#include <source_location>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

namespace simpleslam::logging {

/// 等级由调用决定。
enum class Level : std::uint8_t { kDebug, kInfo, kWarn, kError };

[[nodiscard]] std::string_view toString(Level level) noexcept;
/// `debug`、`info`、`warn`、`error`；别的写法返回空。
[[nodiscard]] std::optional<Level> parseLevel(std::string_view text) noexcept;

/// 记录关于的那一帧。所属模块在处理每条输入之前设上（`Logger::setFrame`），不针对某条输入的
/// 记录带最近处理的那一条。序号按模块自己的输入计：里程计是包序号，后端是关键帧库序号。
struct Frame {
    std::uint64_t sequence = 0;
    std::int64_t data_time_ns = 0;
    std::uint64_t session = 0;
    std::uint64_t parameter_revision = 0;
};

/// 格式串，连同写日志的那一行代码的位置。占位符与参数对不上时编译就报错。
template <class... Args>
struct BasicFormat {
    template <class Text>
        requires std::convertible_to<const Text&, std::string_view>
    consteval BasicFormat(  // NOLINT(google-explicit-constructor): 要从字符串字面量隐式转换
        const Text& literal, std::source_location location = std::source_location::current())
        : text(literal), where(location) {}

    std::format_string<Args...> text;
    std::source_location where;
};

/// 参数类型由后面的实参推导，格式串只负责核对。
template <class... Args>
using Format = BasicFormat<std::type_identity_t<Args>...>;

namespace detail {
class Node;
using NodePtr = std::shared_ptr<Node>;
struct Access;
}  // namespace detail

class Throttled;
class Timer;

/// 日志对象。拷贝是同一个来源。
class Logger {
public:
    /// 没接到日志组件的日志对象：记录同步写 stderr。不经框架、直接构造组件时就是它。
    Logger();

    template <class... Args>
    void debug(Format<Args...> format, Args&&... args) const {
        emit(Level::kDebug, format, std::forward<Args>(args)...);
    }
    template <class... Args>
    void info(Format<Args...> format, Args&&... args) const {
        emit(Level::kInfo, format, std::forward<Args>(args)...);
    }
    template <class... Args>
    void warn(Format<Args...> format, Args&&... args) const {
        emit(Level::kWarn, format, std::forward<Args>(args)...);
    }
    template <class... Args>
    void error(Format<Args...> format, Args&&... args) const {
        emit(Level::kError, format, std::forward<Args>(args)...);
    }

    /// 这一等级会不会记下。参数在调用之前就求值了，昂贵的调试计算先用它判断。
    [[nodiscard]] bool enabled(Level level) const;

    /// 同一行代码每隔 `period` 至多记一条，下一条写明期间合并了几条。时间跟运行时的时钟走，
    /// 离线时是数据时间。
    [[nodiscard]] Throttled every(
        std::chrono::nanoseconds period,
        std::source_location where = std::source_location::current()) const noexcept;

    /// 事件：实际生效的决定（降级、跳帧、校正生效……）。交给诊断，一条不丢。
    void event(std::string_view key, std::string_view detail = {}) const;
    /// 度量：逐帧的数值（迭代次数、内点数……）。交给诊断，一条不丢。
    void metric(std::string_view key, double value, std::string_view unit = {}) const;
    /// 作用域计时：结束时记一条度量 `<step>.duration`，单位毫秒，测的是墙钟。
    [[nodiscard]] Timer timer(std::string_view step) const;

    /// 设上正在处理的那一帧。只对模块的日志对象（来源树的根）起作用。
    void setFrame(const Frame& frame) const noexcept;

    /// 来源的显示形式，如 `/slam/odometry › my_lio › map(voxel_point_map)`。
    [[nodiscard]] std::string source() const;
    /// 按来源设等级用的键，如 `odometry.plugin.map`。
    [[nodiscard]] std::string key() const;

private:
    friend class Throttled;
    friend struct detail::Access;
    explicit Logger(detail::NodePtr node) noexcept;

    template <class... Args>
    void emit(Level level, const Format<Args...>& format, Args&&... args) const {
        if (!enabled(level)) return;
        write(level, format.where, std::format(format.text, std::forward<Args>(args)...));
    }
    void write(Level level, const std::source_location& where, std::string text) const;

    detail::NodePtr node_;
};

/// `log_.every(period)` 的结果：同一行代码按时间限频。只在当前这条语句里用。
class Throttled {
public:
    template <class... Args>
    void debug(Format<Args...> format, Args&&... args) const {
        emit(Level::kDebug, format, std::forward<Args>(args)...);
    }
    template <class... Args>
    void info(Format<Args...> format, Args&&... args) const {
        emit(Level::kInfo, format, std::forward<Args>(args)...);
    }
    template <class... Args>
    void warn(Format<Args...> format, Args&&... args) const {
        emit(Level::kWarn, format, std::forward<Args>(args)...);
    }
    template <class... Args>
    void error(Format<Args...> format, Args&&... args) const {
        emit(Level::kError, format, std::forward<Args>(args)...);
    }

private:
    friend class Logger;
    Throttled(const Logger& logger, std::chrono::nanoseconds period,
              std::source_location site) noexcept
        : logger_(logger), period_(period), site_(site) {}

    template <class... Args>
    void emit(Level level, const Format<Args...>& format, Args&&... args) const {
        if (!logger_.enabled(level)) return;
        const std::optional<std::uint64_t> merged = admit();
        if (!merged) return;
        std::string text = std::format(format.text, std::forward<Args>(args)...);
        if (*merged > 0) text += std::format("（期间合并了 {} 条）", *merged);
        logger_.write(level, format.where, std::move(text));
    }
    /// 这一次放不放行；放行时给出上次放行以来合并了几条。
    [[nodiscard]] std::optional<std::uint64_t> admit() const;

    const Logger& logger_;
    std::chrono::nanoseconds period_;
    std::source_location site_;
};

/// `log_.timer(step)` 的结果：作用域结束时记下这一步的耗时。
class Timer {
public:
    Timer(Timer&& other) noexcept;
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;
    Timer& operator=(Timer&&) = delete;
    ~Timer();

private:
    friend class Logger;
    Timer(Logger logger, std::string key) noexcept;

    Logger logger_;
    std::string key_;
    std::chrono::steady_clock::time_point start_;
    bool active_ = true;
};

/// 插件接口的基类部件：持有 `log_`。槽构造实现时把来源交给它，所以构造函数里就能用；
/// 不经槽、直接构造的是没接上的日志对象，交给模块之后由模块接上（`adopt`）。
class Loggable {
public:
    Loggable(const Loggable&) = delete;
    Loggable& operator=(const Loggable&) = delete;

protected:
    Loggable();
    ~Loggable() = default;

    // 写法定为 `log_.info(…)`；const 保证派生类换不掉它
    const Logger log_;  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes)

private:
    friend struct detail::Access;
};

/// 把没接上的组件挂到 `parent` 下面：`field` 是它在键里的那一段（如 `plugin`），`display` 是
/// 没有实现类型可显示时用的名字。已经接上的不动。只在运行开始之前调用。
void adopt(const Loggable& component, const Logger& parent, std::string_view field,
           std::string_view display);

}  // namespace simpleslam::logging
