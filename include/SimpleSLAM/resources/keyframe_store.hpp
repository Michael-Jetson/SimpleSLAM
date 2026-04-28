#pragma once

/// @file keyframe_store.hpp
/// 关键帧存储——写后不可变模式 + 可扩展属性
///
/// insert() 后核心字段（id, timestamp, pose, scan, image）不可修改。
/// 后端服务可通过 setExtension() 添加扩展属性（每个 key 只写一次）。

#include <SimpleSLAM/core/types/keyframe.hpp>

#include <any>
#include <cstdint>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace simpleslam {

class KeyframeStore {
public:
    /// 插入关键帧（ID 重复时抛异常）
    void insert(KeyframeData keyframe);

    /// 读取关键帧（零拷贝 shared_ptr<const>）
    [[nodiscard]] std::shared_ptr<const KeyframeData> get(uint64_t id) const;

    [[nodiscard]] bool contains(uint64_t id) const;
    [[nodiscard]] size_t size() const;
    [[nodiscard]] std::vector<uint64_t> allIds() const;

    /// 最近插入的关键帧
    [[nodiscard]] std::shared_ptr<const KeyframeData> latest() const;

    // ── 扩展属性（后端服务附加数据，每个 key 写一次）──

    void setExtension(uint64_t keyframe_id, const std::string& key, std::any value);

    template <typename T>
    [[nodiscard]] const T* getExtension(uint64_t keyframe_id,
                                        const std::string& key) const {
        std::shared_lock lock(mutex_);
        auto kf_it = extensions_.find(keyframe_id);
        if (kf_it == extensions_.end()) return nullptr;
        auto attr_it = kf_it->second.find(key);
        if (attr_it == kf_it->second.end()) return nullptr;
        return std::any_cast<T>(&attr_it->second);
    }

    [[nodiscard]] bool hasExtension(uint64_t keyframe_id,
                                    const std::string& key) const;

private:
    mutable std::shared_mutex mutex_;
    std::unordered_map<uint64_t, std::shared_ptr<const KeyframeData>> keyframes_;
    std::unordered_map<uint64_t, std::unordered_map<std::string, std::any>> extensions_;
    uint64_t latest_id_{0};
};

}  // namespace simpleslam
