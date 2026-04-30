#pragma once

/// @file callback_slot.hpp
/// 同步回调槽——OdometryBase 的 on_keyframe / on_after_prediction 等扩展点
///
/// 与 Topic 话题的区别：
///   - CallbackSlot 是同步的，在调用者线程内执行，可读取中间状态
///   - Topic 是异步/松耦合通知，用于线程间通信
/// 两者互补，不冲突。

#include <cstddef>
#include <functional>
#include <mutex>
#include <vector>

namespace simpleslam {

/// 同步回调列表，支持线程安全的注册/注销
template <typename... Args>
class CallbackSlot {
public:
    using Callback = std::function<void(Args...)>;

    /// RAII 连接句柄——析构时自动 disconnect
    class ScopedConnection {
    public:
        ScopedConnection() = default;
        ScopedConnection(CallbackSlot* slot, size_t handle)
            : slot_(slot), handle_(handle) {}
        ~ScopedConnection() { reset(); }

        ScopedConnection(ScopedConnection&& other) noexcept
            : slot_(other.slot_), handle_(other.handle_) {
            other.slot_ = nullptr;
        }
        ScopedConnection& operator=(ScopedConnection&& other) noexcept {
            if (this != &other) {
                reset();
                slot_ = other.slot_;
                handle_ = other.handle_;
                other.slot_ = nullptr;
            }
            return *this;
        }

        ScopedConnection(const ScopedConnection&) = delete;
        ScopedConnection& operator=(const ScopedConnection&) = delete;

        void reset() {
            if (slot_) {
                slot_->disconnect(handle_);
                slot_ = nullptr;
            }
        }

    private:
        CallbackSlot* slot_ = nullptr;
        size_t handle_ = 0;
    };

    /// 注册回调，返回 handle 用于注销
    size_t connect(Callback cb) {
        std::lock_guard lock(mutex_);
        size_t handle = next_handle_++;
        entries_.push_back({handle, std::move(cb)});
        return handle;
    }

    /// 注册成员函数回调
    template <typename Obj>
    size_t connect(void (Obj::*method)(Args...), Obj* obj) {
        return connect([obj, method](Args... args) { (obj->*method)(args...); });
    }

    /// 注册回调并返回 RAII 句柄（析构自动退订）
    ScopedConnection connectScoped(Callback cb) {
        return ScopedConnection(this, connect(std::move(cb)));
    }

    /// 注册成员函数并返回 RAII 句柄
    template <typename Obj>
    ScopedConnection connectScoped(void (Obj::*method)(Args...), Obj* obj) {
        return ScopedConnection(this, connect(method, obj));
    }

    /// 按 handle 注销回调
    void disconnect(size_t handle) {
        std::lock_guard lock(mutex_);
        entries_.erase(
            std::remove_if(entries_.begin(), entries_.end(),
                           [handle](const Entry& e) { return e.handle == handle; }),
            entries_.end());
    }

    /// 同步调用所有已注册回调（按注册顺序）
    void emit(Args... args) const {
        std::vector<Callback> snapshot;
        {
            std::lock_guard lock(mutex_);
            snapshot.reserve(entries_.size());
            for (const auto& e : entries_) {
                snapshot.push_back(e.callback);
            }
        }
        for (const auto& cb : snapshot) {
            cb(args...);
        }
    }

    [[nodiscard]] size_t size() const {
        std::lock_guard lock(mutex_);
        return entries_.size();
    }

    [[nodiscard]] bool empty() const {
        std::lock_guard lock(mutex_);
        return entries_.empty();
    }

private:
    struct Entry {
        size_t handle;
        Callback callback;
    };

    mutable std::mutex mutex_;
    std::vector<Entry> entries_;
    size_t next_handle_{0};
};

}  // namespace simpleslam
