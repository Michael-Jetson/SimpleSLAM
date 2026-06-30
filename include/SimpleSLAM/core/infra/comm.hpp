#pragma once

/// @file comm.hpp
/// 通信机制统一入口 —— 一行 include 获得话题发布-订阅总线。
///
///   #include <SimpleSLAM/core/infra/comm.hpp>
///
/// 进程内、零序列化、header-only。需要细粒度时仍可单独 include 各头文件。
/// 注：Service/Action（request-reply RPC / 可取消长任务）已绞杀——零生产消费者，
/// 属蓝图反模式14"通信过度设计"的回潮。需要时从 git 历史恢复（R2 / strangler-fig）。

#include <SimpleSLAM/core/infra/comm/topic.hpp>        // Topic<T> + TopicHub + Publisher / MsgPtr / QoS
#include <SimpleSLAM/core/infra/comm/topic_names.hpp>  // 话题名常量
