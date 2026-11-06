// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file node.hpp
/// 来源树的节点（实现细节）。根是模块或执行器，带着日志组件与当前帧；组件节点只记自己那一段
/// 名字与父节点。完整来源在第一次记日志时沿父链拼出来、缓存住：父链只在运行开始之前变
/// （`adopt`），之后缓存一直有效，关掉的等级只花一次原子读和一次比较。

#include <SimpleSLAM/logging/logger.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <source_location>
#include <string>
#include <string_view>
#include <utility>

namespace simpleslam::logging {
class Router;
}

namespace simpleslam::logging::detail {

/// 接上日志组件之后，一个节点的完整来源。
struct Resolved {
    Router* router = nullptr;
    const Node* root = nullptr;
    std::string module;  ///< 根的名字
    std::string source;
    std::string key;
    Level console = Level::kInfo;
    Level file = Level::kDebug;
    Level gate = Level::kDebug;  ///< 两者较低的那个：低于它的调用直接返回
    bool plain = false;
};

class Node : public std::enable_shared_from_this<Node> {
public:
    struct Init {
        NodePtr parent;
        std::string key_segment;
        std::string display_segment;
        std::shared_ptr<Router> router;  ///< 只有根有
        bool root = false;
        bool plain = false;
        bool muted = false;
    };

    explicit Node(Init init);

    /// 接上了才有；没接上（父链到不了带日志组件的根）时为空。
    [[nodiscard]] const Resolved* resolved();
    [[nodiscard]] bool muted() const noexcept { return muted_; }
    [[nodiscard]] bool isRoot() const noexcept { return root_; }

    /// 没接上时挂到 `parent` 下面；已经有父节点的不动。
    void adopt(const NodePtr& parent, std::string key_segment, std::string_view display);

    void setFrame(const Frame& frame) noexcept;
    [[nodiscard]] std::optional<Frame> frame() const noexcept;

    /// 限频：这一行代码这一次放不放行，放行时给出合并了几条。
    [[nodiscard]] std::optional<std::uint64_t> admit(const std::source_location& site,
                                                     std::chrono::nanoseconds period,
                                                     std::int64_t now);

    /// 沿父链拼出来的显示形式与键。没接上时也能用（只拼出已知的部分）。
    [[nodiscard]] std::string sourceText() const;
    [[nodiscard]] std::string keyText() const;

private:
    [[nodiscard]] NodePtr parent() const;

    mutable std::mutex mutex_;
    NodePtr parent_;
    std::string key_segment_;
    std::string display_segment_;
    std::shared_ptr<Router> router_;
    bool root_;
    bool plain_;
    bool muted_;

    std::atomic<std::uint64_t> sequence_{0};
    std::atomic<std::int64_t> data_time_{0};
    std::atomic<std::uint64_t> session_{0};
    std::atomic<std::uint64_t> revision_{0};
    std::atomic<bool> has_frame_{false};

    std::atomic<const Resolved*> cache_{nullptr};
    std::unique_ptr<Resolved> cache_owner_;

    struct Throttle {
        std::int64_t last = 0;
        std::uint64_t merged = 0;
    };
    std::mutex throttle_mutex_;
    std::map<std::pair<std::string_view, std::uint_least32_t>, Throttle> throttles_;
};

/// 一个字母的等级：D、I、W、E。
[[nodiscard]] constexpr char levelLetter(Level level) noexcept {
    switch (level) {
        case Level::kDebug: return 'D';
        case Level::kInfo: return 'I';
        case Level::kWarn: return 'W';
        case Level::kError: return 'E';
    }
    return '?';
}

/// 日志对象与组件的私有部分只经这里取。
struct Access {
    static const NodePtr& node(const Logger& logger) noexcept { return logger.node_; }
    static Logger make(NodePtr node) noexcept { return Logger(std::move(node)); }
    static const Logger& log(const Loggable& component) noexcept { return component.log_; }
};

}  // namespace simpleslam::logging::detail
