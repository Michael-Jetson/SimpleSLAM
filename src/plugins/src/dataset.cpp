// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/plugins/dataset.hpp>

#include <format>
#include <stdexcept>

namespace simpleslam::plugin {

Dataset DatasetReaderBase::open(const DatasetRequest& request) const {
    Dataset dataset = doOpen(request);
    if (dataset.source == nullptr) {
        throw std::logic_error(std::format("数据集读取器 {} 没有给出来源", name()));
    }
    return dataset;
}

}  // namespace simpleslam::plugin
