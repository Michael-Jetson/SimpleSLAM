// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file append_log.hpp
/// 追加式不可变日志：给"只增不改"的大资源用的存储。
///
/// 起因：通用的 `Resource<T>` 每次 modify() 都整份复制 T。
/// 对关键帧库这种只追加的资源，长期累计是平方量级——新增第 n 帧要复制前 n-1 帧的索引。
/// 但"版本化不可变快照"这个语义本身是对的，不该为性能放弃。
///
/// 于是把存储换掉而不是把语义换掉：日志按定长块组织，封口的块只共享指针，
/// 只有尚未写满的那一块在有快照引用时才写时复制。于是
///   - 复制一份日志（= 取快照 / 建草稿）：O(块数) = O(n / B)，B 是块容量
///   - 追加一条：O(B)
///
/// **代价如实说**：单次操作是 O(n/B + B)，不是"与 n 无关"——
/// 块指针向量仍然随 n 线性增长。逐条提交 n 条的累计代价是 O(n²/B + nB)，
/// 相对整份复制的 O(n²) 小了 B 倍，但同阶项还在。B = 256 时，
/// 十万条关键帧的单次复制约 400 个指针，实践上可以忽略；真到需要在意的规模，
/// 该换的是持久化索引或分层目录，而不是继续调 B。先把代价记准，再按实测决定。
///
/// 写时复制的判据是 use_count()：引用计数只会被写者自己（建草稿时复制日志）抬高，
/// 被快照析构降低，因此写者读到的值只会偏大（多复制一次，安全），不会偏小。

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace simpleslam::fw {

template <class T, std::size_t BlockCapacity = 256>
class AppendLog {
    static_assert(BlockCapacity > 0, "块容量必须为正");

public:
    using value_type = T;
    using Block = std::vector<T>;

    AppendLog() = default;

    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] bool empty() const noexcept { return size_ == 0; }
    [[nodiscard]] std::size_t blockCount() const noexcept {
        return sealed_.size() + (open_ ? 1U : 0U);
    }

    /// 追加一条。封口的块不动，只有开放块在被快照引用时才复制。
    void push_back(T value) {
        if (!open_) {
            open_ = std::make_shared<Block>();
            open_->reserve(BlockCapacity);
        } else if (open_->size() >= BlockCapacity) {
            sealed_.push_back(std::const_pointer_cast<const Block>(open_));
            open_ = std::make_shared<Block>();
            open_->reserve(BlockCapacity);
        } else if (open_.use_count() > 1) {
            // 有快照正引用这一块：写时复制，已发布的内容不得被改动
            auto copy = std::make_shared<Block>(*open_);
            copy->reserve(BlockCapacity);
            open_ = std::move(copy);
        }
        open_->push_back(std::move(value));
        ++size_;
    }

    [[nodiscard]] const T& at(std::size_t index) const {
        if (index >= size_) throw std::out_of_range("AppendLog 下标越界");
        const std::size_t block = index / BlockCapacity;
        const std::size_t offset = index % BlockCapacity;
        if (block < sealed_.size()) return (*sealed_[block])[offset];
        return (*open_)[offset];
    }

    [[nodiscard]] const T& operator[](std::size_t index) const { return at(index); }
    [[nodiscard]] const T& back() const { return at(size_ - 1); }
    [[nodiscard]] const T& front() const { return at(0); }

    /// 按下标序遍历。遍历顺序即追加顺序，因此离线复现是确定的。
    template <class F>
    void forEach(F&& visit) const {
        for (const auto& block : sealed_) {
            for (const auto& item : *block) visit(item);
        }
        if (open_) {
            for (const auto& item : *open_) visit(item);
        }
    }

    /// 从某个下标起的尾段。消费者按"消费到哪里"补读断号时用。
    [[nodiscard]] std::vector<T> tail(std::size_t from_index) const {
        std::vector<T> out;
        if (from_index >= size_) return out;
        out.reserve(size_ - from_index);
        for (std::size_t i = from_index; i < size_; ++i) out.push_back(at(i));
        return out;
    }

    /// 摊平成一个普通 vector。给"要整体重算"的消费者（如图优化）用。
    [[nodiscard]] std::vector<T> flatten() const { return tail(0); }

    /// 用一个普通 vector 整体重建。给"解已经全部换过"的写者用，代价 O(n)。
    static AppendLog fromVector(const std::vector<T>& items) {
        AppendLog log;
        for (const auto& item : items) log.push_back(item);
        return log;
    }

    void clear() noexcept {
        sealed_.clear();
        open_.reset();
        size_ = 0;
    }

private:
    std::vector<std::shared_ptr<const Block>> sealed_;
    std::shared_ptr<Block> open_;
    std::size_t size_ = 0;
};

}  // namespace simpleslam::fw
