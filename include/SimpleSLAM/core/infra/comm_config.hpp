#pragma once

/// @file comm_config.hpp
/// 通信参数的**运行期加载** —— 各模块对自己的 config 段调，把 YAML 转成
/// 话题名 / QoS / SubscribeOptions。
///
/// 分工：消息**类型**编译期（模块代码定，`Topic<LidarScan>`）；**怎么投递**的参数
/// （名字、QoS、队列深度、丢弃、节流、投递模式）运行期从 YAML 读。
///
/// 依赖 config.hpp（yaml-cpp），故单独成文件，`topic.hpp` 保持纯 std、零依赖。
/// 用法（模块在自己 ctor 里，对自己的 config 段）：
///   auto in = loadTopicSpec(cfg.node("input"));
///   sub_ = TopicHub::createSubscriber<LidarScan>(in.name, &Mod::onScan, this, in.options);

#include <SimpleSLAM/core/infra/config.hpp>
#include <SimpleSLAM/core/infra/topic.hpp>

#include <string>
#include <string_view>

#include <yaml-cpp/yaml.h>

namespace simpleslam {

// ── 枚举字符串解析（纯字符串 → 枚举）──

inline QoS parseQoS(std::string_view s) {
    if (s == "event") return QoS::Event;
    if (s == "stream") return QoS::Stream;
    if (s == "latest") return QoS::Latest;
    throw ConfigError("未知 QoS: " + std::string(s) + "（应为 event/stream/latest）");
}

inline DropPolicy parseDropPolicy(std::string_view s) {
    if (s == "oldest") return DropPolicy::DropOldest;
    if (s == "newest") return DropPolicy::DropNewest;
    throw ConfigError("未知 drop: " + std::string(s) + "（应为 oldest/newest）");
}

inline DeliveryMode parseDeliveryMode(std::string_view s) {
    if (s == "sync") return DeliveryMode::Sync;
    if (s == "async") return DeliveryMode::Async;
    throw ConfigError("未知 delivery: " + std::string(s) + "（应为 sync/async）");
}

// ── 从 YAML 节点读参数（缺省字段取默认）──

/// 读 SubscribeOptions（queue_depth / drop / throttle / delivery）。
inline SubscribeOptions parseSubscribeOptions(const YAML::Node& node) {
    SubscribeOptions opts;
    if (node["queue_depth"]) opts.queue_depth = node["queue_depth"].as<size_t>();
    if (node["drop"]) opts.drop = parseDropPolicy(node["drop"].as<std::string>());
    if (node["throttle"]) opts.throttle_every = node["throttle"].as<int>();
    if (node["delivery"]) opts.delivery = parseDeliveryMode(node["delivery"].as<std::string>());
    return opts;
}

/// 话题规格：名字 + QoS（发布侧）+ SubscribeOptions（订阅侧）。
struct TopicSpec {
    std::string name;
    QoS qos = QoS::Event;
    SubscribeOptions options;
};

/// 从一个话题配置块读 TopicSpec，如 `{topic: sensor/lidar, qos: stream, queue_depth: 5}`。
inline TopicSpec loadTopicSpec(const YAML::Node& node) {
    if (!node || !node["topic"]) {
        throw ConfigError("话题配置缺 'topic' 字段");
    }
    TopicSpec spec;
    spec.name = node["topic"].as<std::string>();
    if (node["qos"]) spec.qos = parseQoS(node["qos"].as<std::string>());
    spec.options = parseSubscribeOptions(node);
    return spec;
}

/// 便捷：从 Config 的 dot-path 读 TopicSpec。
inline TopicSpec loadTopicSpec(const Config& cfg, const std::string& dot_path) {
    return loadTopicSpec(cfg.node(dot_path));
}

/// 系统级：总线离线/在线模式（整条总线一致，Runner 启动期读）。默认离线（确定性）。
inline bool loadOfflineMode(const Config& cfg) {
    return cfg.get<bool>("runtime.offline_mode", true);
}

}  // namespace simpleslam
