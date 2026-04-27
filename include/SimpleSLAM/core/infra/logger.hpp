#pragma once

/// @file logger.hpp
/// 日志系统——基于 spdlog 的模块化日志封装
///
/// 特性：
///   - 按模块命名独立 logger（如 "odometry"、"backend"、"io"）
///   - 异步日志支持，实时线程使用 overrun_oldest 策略（不阻塞）
///   - backtrace ring buffer，致命错误时可 dump 最近日志
///   - 日志级别从 YAML 按模块独立配置

#include <memory>
#include <string>

#include <spdlog/spdlog.h>

// 前置声明，避免在头文件中暴露 yaml-cpp
namespace YAML {
class Node;
}

namespace simpleslam {

class Logger {
public:
    /// 从 YAML 配置初始化日志系统
    /// @param config YAML 的 "logging" 节点
    static void init(const YAML::Node& config);

    /// 使用默认设置初始化（控制台 info 级别）
    static void initDefault();

    /// 获取指定模块的 logger，不存在则自动创建
    /// 内部有 mutex 保护，热路径应缓存返回值而非反复调用：
    ///   auto log = Logger::get("odometry");  // 缓存到局部变量或成员
    ///   log->debug("...");
    static std::shared_ptr<spdlog::logger> get(const std::string& module_name);

    /// dump 指定模块的 backtrace ring buffer
    static void dumpBacktrace(const std::string& module_name);

    /// 关闭所有 logger（flush + 释放资源）
    static void shutdown();

private:
    Logger() = default;
};

}  // namespace simpleslam
