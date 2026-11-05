// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/app/replay_reader.hpp>
#include <SimpleSLAM/record/reader.hpp>

#include <format>
#include <memory>
#include <stdexcept>

namespace simpleslam::app {

ReplayReader::ReplayReader(const Config& config) : config_(config) {
    if (config_.path.empty()) throw std::invalid_argument("replay.path 不能为空");
}

std::string ReplayReader::name() const { return std::format("replay({})", config_.path); }

plugin::Dataset ReplayReader::doOpen(const plugin::DatasetRequest& /*request*/) const {
    plugin::Dataset dataset;
    dataset.source = std::make_unique<record::ReplaySource>(config_.path);
    return dataset;
}

}  // namespace simpleslam::app
