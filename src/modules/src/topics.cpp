// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/modules/topics.hpp>

#include <stdexcept>
#include <utility>

namespace simpleslam::modules {

std::uint64_t SlamResources::setCalibration(fw::CalibrationData data) {
    std::lock_guard lock(calibration_mutex_);
    return commitCalibrationLocked(std::move(data));
}

void SlamResources::freeze() {
    std::lock_guard lock(calibration_mutex_);
    frozen_ = true;
}

std::uint64_t SlamResources::commitCalibrationLocked(fw::CalibrationData data) {
    if (frozen_) {
        throw std::logic_error("运行已经终结，标定是最终结果的一部分，不再接受写入");
    }
    if (const auto problem = fw::validateCalibration(data); !problem.empty()) {
        throw std::invalid_argument("标定不合法：" + problem);
    }
    const std::uint64_t next = calibration.version() + 1;
    data.revision = next;
    calibration_writer_.commit(std::move(data));
    return next;
}

}  // namespace simpleslam::modules
