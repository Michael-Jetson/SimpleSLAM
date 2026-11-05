// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/comm/parameters.hpp>

#include <utility>

namespace simpleslam::comm {

void ParameterStore::preset(std::string name, ParamValue value) {
    std::lock_guard lock(mutex_);
    auto next = std::make_shared<ParamSnapshot>(*snapshot_);
    next->values.insert_or_assign(std::move(name), std::move(value));
    snapshot_ = std::move(next);
}

ParamCommitResult ParameterStore::commit(const std::vector<ParamChange>& changes) {
    // 校验是用户代码，不在锁内跑。校验期间快照被别人换了就基于新快照重来
    for (;;) {
        ParamSnapshotPtr base;
        Validator validator;
        {
            std::lock_guard lock(mutex_);
            base = snapshot_;
            validator = validator_;
        }
        auto next = std::make_shared<ParamSnapshot>(*base);
        for (const auto& change : changes) {
            auto it = next->values.find(change.name);
            if (it == next->values.end()) {
                return {ParamCommit::kUnknownName, base->revision, change.name, nullptr};
            }
            if (it->second.index() != change.value.index()) {
                return {ParamCommit::kRejectedType, base->revision, change.name, nullptr};
            }
            it->second = change.value;
        }
        if (validator) {
            std::string detail;
            if (!validator(*next, detail)) {
                return {ParamCommit::kRejectedValue, base->revision, std::move(detail), nullptr};
            }
        }
        next->revision = base->revision + 1;
        ParamSnapshotPtr published = next;
        std::lock_guard lock(mutex_);
        if (snapshot_ != base) continue;
        snapshot_ = std::move(next);
        return {ParamCommit::kStored, published->revision, {}, published};
    }
}

}  // namespace simpleslam::comm
