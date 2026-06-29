#pragma once

/// @file comm.hpp
/// 通信机制统一入口 —— 一行 include 获得话题 + 服务 + ROS1 门面。
///
///   #include <SimpleSLAM/core/infra/comm.hpp>
///
/// 进程内、零序列化、header-only。需要细粒度时仍可单独 include 各头文件。

#include <SimpleSLAM/core/infra/topic.hpp>        // Topic<T> + TopicHub + Publisher / MsgPtr / QoS
#include <SimpleSLAM/core/infra/topic_names.hpp>  // 话题名常量
#include <SimpleSLAM/core/infra/service.hpp>      // 进程内服务 request/reply
#include <SimpleSLAM/core/infra/action.hpp>       // 进程内 Action（长任务/取消/反馈）
