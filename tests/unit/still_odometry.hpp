// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// 测试用的里程计：原地不动，每 keyframe_every 帧给一个关键帧建议。
/// 执行器的测试与多序列子进程的入口共用它：子进程得登记同一个实现。

#include <SimpleSLAM/plugins/odometry.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

namespace simpleslam::test {

class StillOdometry final : public plugin::OdometryBase {
public:
    static constexpr std::string_view kTypeName = "still";

    struct Config {
        std::int64_t keyframe_every = 5;

        template <class V>
        void visit(V& v) {
            v("keyframe_every", keyframe_every, "每几帧一个关键帧");
        }
    };

    explicit StillOdometry(const Config& config) : config_(config) {
        if (config_.keyframe_every < 1) throw std::invalid_argument("keyframe_every 至少为 1");
    }

    [[nodiscard]] plugin::SupportGapPolicy supportGapPolicy() const override {
        return plugin::SupportGapPolicy::kDegrade;
    }
    [[nodiscard]] std::string name() const override { return "still"; }

protected:
    plugin::TrackResult track(const fw::SensorPacket& /*packet*/) override {
        plugin::TrackResult result;
        result.suggest_keyframe = frames_ % config_.keyframe_every == 0;
        log_.debug("原地不动，第 {} 帧", frames_++);
        return result;
    }

private:
    Config config_;
    std::int64_t frames_ = 0;
};

}  // namespace simpleslam::test
