#pragma once

/// @file service_base.hpp
/// 后端服务可选基类——提供生命周期方法和名称标识
///
/// 后端服务可以继承此类获得统一的 initialize/shutdown 生命周期，
/// 也可以完全不继承，直接订阅 Topic 话题使用。
/// 这是一个便利基类，不是强制接口。

#include <SimpleSLAM/core/infra/logger.hpp>

#include <string>
#include <utility>

namespace simpleslam {

class TopicHub;  // 前向声明

class ServiceBase {
public:
    explicit ServiceBase(std::string service_name)
        : name_(std::move(service_name))
        , log_(Logger::get(name_)) {}

    virtual ~ServiceBase() = default;

    /// 初始化——在 Runner 组装完成后、主循环开始前调用
    /// 子类在此注册 Topic 订阅
    virtual void initialize(TopicHub& hub) { (void)hub; }

    /// 关闭——在系统停止时调用，释放资源
    virtual void shutdown() {}

    [[nodiscard]] const std::string& name() const { return name_; }

    ServiceBase(const ServiceBase&) = delete;
    ServiceBase& operator=(const ServiceBase&) = delete;

protected:
    std::string name_;
    std::shared_ptr<spdlog::logger> log_;  ///< 以模块名命名的专属日志器
};

}  // namespace simpleslam
