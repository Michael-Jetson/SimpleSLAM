// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file replay_reader.hpp
/// 回放读取器：把一份录制当成数据集读。录制里没有推荐的打包策略，用系统配置的；
/// 标定由任务的 `data.calibration` 给出。

#include <SimpleSLAM/plugins/dataset.hpp>

#include <string>
#include <string_view>

namespace simpleslam::app {

class ReplayReader final : public plugin::DatasetReader {
public:
    static constexpr std::string_view kTypeName = "replay";

    struct Config {
        std::string path;

        template <class V>
        void visit(V& v) {
            v("path", path, "录制文件路径");
        }
    };

    /// 路径为空时抛 `std::invalid_argument`。
    explicit ReplayReader(const Config& config);

    [[nodiscard]] std::string name() const override;

private:
    [[nodiscard]] plugin::Dataset doOpen(const plugin::DatasetRequest& request) const override;

    Config config_;
};

}  // namespace simpleslam::app
