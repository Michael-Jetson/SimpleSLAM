#include <SimpleSLAM/resources/keyframe_store.hpp>

#include <mutex>

namespace simpleslam {

void KeyframeStore::insert(KeyframeData keyframe) {
    std::unique_lock lock(mutex_);
    if (keyframes_.count(keyframe.id)) {
        throw std::runtime_error(
            "KeyframeStore: duplicate keyframe ID: " + std::to_string(keyframe.id));
    }
    uint64_t id = keyframe.id;
    keyframes_[id] = std::make_shared<const KeyframeData>(std::move(keyframe));
    latest_id_ = id;
}

std::shared_ptr<const KeyframeData> KeyframeStore::get(uint64_t id) const {
    std::shared_lock lock(mutex_);
    auto it = keyframes_.find(id);
    if (it == keyframes_.end()) return nullptr;
    return it->second;
}

bool KeyframeStore::contains(uint64_t id) const {
    std::shared_lock lock(mutex_);
    return keyframes_.count(id) > 0;
}

size_t KeyframeStore::size() const {
    std::shared_lock lock(mutex_);
    return keyframes_.size();
}

std::vector<uint64_t> KeyframeStore::allIds() const {
    std::shared_lock lock(mutex_);
    std::vector<uint64_t> ids;
    ids.reserve(keyframes_.size());
    for (const auto& [id, _] : keyframes_) {
        ids.push_back(id);
    }
    return ids;
}

std::shared_ptr<const KeyframeData> KeyframeStore::latest() const {
    std::shared_lock lock(mutex_);
    auto it = keyframes_.find(latest_id_);
    if (it == keyframes_.end()) return nullptr;
    return it->second;
}

void KeyframeStore::setExtension(uint64_t keyframe_id,
                                 const std::string& key,
                                 std::any value) {
    std::unique_lock lock(mutex_);
    if (!keyframes_.count(keyframe_id)) {
        throw std::runtime_error(
            "KeyframeStore: keyframe not found: " + std::to_string(keyframe_id));
    }
    auto& attrs = extensions_[keyframe_id];
    if (attrs.count(key)) {
        throw std::runtime_error(
            "KeyframeStore: extension already set: " + key);
    }
    attrs[key] = std::move(value);
}

bool KeyframeStore::hasExtension(uint64_t keyframe_id,
                                 const std::string& key) const {
    std::shared_lock lock(mutex_);
    auto kf_it = extensions_.find(keyframe_id);
    if (kf_it == extensions_.end()) return false;
    return kf_it->second.count(key) > 0;
}

}  // namespace simpleslam
