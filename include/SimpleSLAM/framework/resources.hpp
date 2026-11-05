// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file resources.hpp
/// 资源：版本化不可变快照 + move-only 单写者令牌。
///
/// 写操作在锁外构建新版本、锁内换指针，因此写锁持有时间是常数级，
/// 且天然有强异常保证：构建新版本时抛异常，读者看到的仍是上一个完整版本。
/// 一个资源只发放一个写令牌——"单写者"由类型系统保证，不靠评审。
///
/// 两条边界：
/// 1. 换下来的旧快照在锁外析构。否则"锁内只换指针"只在最后一个引用不在此处时成立，
///    一旦本次提交恰好是旧版本的最后一个持有者，整份数据的析构就发生在锁内。
/// 2. modify() 复制整份 T，所以 T 自己要选合适的存储。只增不改的大资源用
///    AppendLog（append_log.hpp），小状态才适合整份快照。这是资源自己的事，
///    不是本模板的事——本模板只负责版本、令牌与不可变读取。

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace simpleslam::fw {

template <class T>
class Resource;

/// 写令牌。move-only，销毁即归还。
template <class T>
class WriteToken {
public:
    WriteToken() = default;

    WriteToken(const WriteToken&) = delete;
    WriteToken& operator=(const WriteToken&) = delete;

    WriteToken(WriteToken&& other) noexcept : resource_(other.resource_) {
        other.resource_ = nullptr;
    }
    WriteToken& operator=(WriteToken&& other) noexcept {
        if (this != &other) {
            release();
            resource_ = other.resource_;
            other.resource_ = nullptr;
        }
        return *this;
    }
    ~WriteToken() { release(); }

    [[nodiscard]] bool valid() const noexcept { return resource_ != nullptr; }

    /// 读-改-写：在锁外复制并修改，锁内换指针。返回新版本号。
    template <class F>
    std::uint64_t modify(F&& mutate) {
        requireValid();
        auto draft = std::make_shared<T>(*resource_->snapshot());
        mutate(*draft);
        return resource_->commit(std::move(draft));
    }

    /// 读-改-写，并返回刚刚提交的那一份快照。
    ///
    /// 与"modify 之后再 snapshot()"的区别：那是两次读取，中间存在一个窗口。
    /// 需要把"这次提交的结果"原样发给消费者时（如校正事件携带解快照），用这个。
    template <class F>
    std::shared_ptr<const T> modifyAndSnapshot(F&& mutate) {
        requireValid();
        auto draft = std::make_shared<T>(*resource_->snapshot());
        mutate(*draft);
        std::shared_ptr<const T> published = draft;
        resource_->commit(std::move(draft));
        return published;
    }

    /// 直接提交一个已构建好的新版本。
    std::uint64_t commit(T next) {
        requireValid();
        return resource_->commit(std::make_shared<T>(std::move(next)));
    }

private:
    friend class Resource<T>;
    explicit WriteToken(Resource<T>* resource) : resource_(resource) {}

    void requireValid() const {
        if (resource_ == nullptr) throw std::logic_error("使用了已失效的写令牌");
    }
    void release() noexcept {
        if (resource_ != nullptr) {
            resource_->returnToken();
            resource_ = nullptr;
        }
    }

    Resource<T>* resource_ = nullptr;
};

/// 版本化资源。读者随时取快照，快照内容永不变。
template <class T>
class Resource {
public:
    using Snapshot = std::shared_ptr<const T>;

    Resource() : snapshot_(std::make_shared<const T>()) {}
    explicit Resource(T initial) : snapshot_(std::make_shared<const T>(std::move(initial))) {}

    Resource(const Resource&) = delete;
    Resource& operator=(const Resource&) = delete;

    [[nodiscard]] Snapshot snapshot() const {
        std::lock_guard lock(mutex_);
        return snapshot_;
    }

    [[nodiscard]] std::uint64_t version() const {
        std::lock_guard lock(mutex_);
        return version_;
    }

    /// 取写令牌。第二次取会抛——单写者是硬约束。
    [[nodiscard]] WriteToken<T> acquireWriteToken() {
        bool expected = false;
        if (!token_issued_.compare_exchange_strong(expected, true)) {
            throw std::logic_error("资源已有写者，不得发放第二个写令牌");
        }
        return WriteToken<T>(this);
    }

private:
    friend class WriteToken<T>;

    std::uint64_t commit(std::shared_ptr<T> next) {
        Snapshot previous;  ///< 在锁外析构：旧版本可能很大，不该占着写锁释放
        std::uint64_t version = 0;
        {
            std::lock_guard lock(mutex_);
            previous = std::move(snapshot_);
            snapshot_ = std::move(next);
            version = ++version_;
        }
        previous.reset();
        return version;
    }
    void returnToken() noexcept { token_issued_.store(false); }

    mutable std::mutex mutex_;
    Snapshot snapshot_;
    std::uint64_t version_ = 0;
    std::atomic<bool> token_issued_{false};
};

}  // namespace simpleslam::fw
