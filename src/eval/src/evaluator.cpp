// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/eval/evaluator.hpp>

#include <algorithm>
#include <cctype>
#include <format>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace simpleslam::eval {

namespace {

/// 名字只收字母、数字与 `_`；允许分段时还收 `.`，但不在两头、不连写。
bool plainName(std::string_view name, bool segments) {
    if (name.empty() || name.front() == '.' || name.back() == '.') return false;
    if (name.find("..") != std::string_view::npos) return false;
    return std::all_of(name.begin(), name.end(), [&](char c) {
        const bool plain = std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
        return plain || (segments && c == '.');
    });
}

}  // namespace

fw::Trajectory IEvaluationInputs::estimate(std::string_view name) const {
    throw std::runtime_error("这份评测输入不单独给估计轨迹（要的是 " + std::string(name) + "）");
}

std::vector<MetricSample> IEvaluationInputs::metrics(std::string_view key) const {
    throw std::runtime_error("这份评测输入没有逐帧度量（要的是 " + std::string(key) + "）");
}

std::vector<EventSample> IEvaluationInputs::events(std::string_view key) const {
    throw std::runtime_error("这份评测输入没有事件（要的是 " + std::string(key) + "）");
}

void checkEvaluation(std::string_view evaluator, const Evaluation& result) {
    const auto fail = [&](const std::string& what) {
        throw std::runtime_error(std::format("评测函数 {} 给的结果不合格：{}", evaluator, what));
    };
    std::vector<std::string_view> scalars;
    for (const auto& scalar : result.scalars) {
        if (!plainName(scalar.name, true)) {
            fail(std::format("标量名 {} 只能用字母、数字、_ 与分段的 .", scalar.name));
        }
        if (scalar.name == "samples") fail("标量名 samples 留给样本数");
        if (std::find(scalars.begin(), scalars.end(), scalar.name) != scalars.end()) {
            fail(std::format("标量名 {} 重复", scalar.name));
        }
        scalars.push_back(scalar.name);
    }
    std::vector<std::string_view> tables;
    for (const auto& table : result.tables) {
        if (!plainName(table.name, false)) {
            fail(std::format("表名 {} 只能用字母、数字与 _：它是文件名的一段", table.name));
        }
        if (std::find(tables.begin(), tables.end(), table.name) != tables.end()) {
            fail(std::format("表名 {} 重复", table.name));
        }
        tables.push_back(table.name);
        for (const auto& column : table.columns) {
            if (!plainName(column.name, false)) {
                fail(std::format("表 {} 的列名 {} 只能用字母、数字与 _", table.name, column.name));
            }
        }
        for (const auto& row : table.rows) {
            if (row.size() != table.columns.size()) {
                fail(std::format("表 {} 有一行的格数是 {}，列数是 {}", table.name, row.size(),
                                 table.columns.size()));
            }
        }
    }
}

}  // namespace simpleslam::eval
