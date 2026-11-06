// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file run.hpp
/// 任务执行器与命令行入口。
///
/// 一个任务是"用哪个 SLAM 系统、在线还是离线、处理哪份数据、评不评测"。
/// 执行器依次：读并还原任务 → 核对组合规则 → 打开数据、合并外部标定 → 按管线装配 →
/// 驱动运行 → 终结、写任务记录 → 列了评测函数就对任务记录评测 → 按运行是否完整返回
/// 退出码。多序列（`data.sequences` 非空）时，每个序列用同一个可执行文件起一个
/// 子进程，各自写一份子任务记录，最后汇总成一张表。评测也可以单独做（`evaluateTask`）。
///
/// 入口只有几行：用户的实现先追加到目录，再交给 `run`。
///
///     int main(int argc, char** argv) {
///         auto catalogs = simpleslam::app::builtinCatalogs();
///         catalogs.of<simpleslam::plugin::OdometryBase>().add<MyLo>();
///         return simpleslam::app::run(argc, argv, catalogs);
///     }

#include <SimpleSLAM/config/plugin.hpp>
#include <SimpleSLAM/framework/time.hpp>

#include <cstddef>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

namespace simpleslam::app {

struct RunOptions {
    std::string task_file;
    /// 路径=值，排在任务文件自己的 overrides 之后。
    std::vector<std::string> overrides;
    /// 任务记录目录已有内容时照样写：本执行器写的那些文件先删掉，别的文件不动。
    bool overwrite = false;
    /// 多序列时同时跑几个序列。
    std::size_t jobs = 1;
    /// 多序列时为每个序列起子进程的可执行文件。空就用当前的可执行文件，用户登记的实现
    /// 在子进程里因此同样有。
    std::string executable{};
    /// 另存进任务记录 `source/` 的原始文件，如操作台渲染之前的 `.py` 任务。空表示没有。
    std::string source_file{};
};

/// 跑一个任务。进度与结论写到 out，问题写到 err；多序列时每个序列的输出在它的日志里。
/// 返回退出码：0 跑完且完整；4 跑完但不完整（任务记录照常写完）；1 出错；2 用法错误。
/// 多序列时返回各序列里最严重的那个：出错 > 不完整 > 完整。
int runTask(const RunOptions& options, const config::Catalogs& catalogs, std::ostream& out,
            std::ostream& err);

/// 单独评测一份任务记录：按它的 `task.yaml` 与覆盖项（只能改 `evaluation.*`）评测，改写
/// 结论与表格，运行的产物不动。多序列的任务记录逐个子任务记录评测，再汇总。
/// 退出码同 `runTask`。
int evaluateTask(const std::string& dir, const std::vector<std::string>& overrides,
                 const config::Catalogs& catalogs, std::ostream& out, std::ostream& err);

/// 单独评测别的系统跑出来的轨迹：写出一份最小的任务记录（两条轨迹、结论、表格），
/// 操作台能把它和自己的结果放进一张表。`simpleslam-eval` 就是它。
struct TrajectoryEvaluation {
    std::string estimate;     ///< 估计轨迹文件（TUM 或 KITTI）
    std::string groundtruth;  ///< 真值轨迹文件
    std::string format{};     ///< tum / kitti；空表示按内容判断
    std::string out;          ///< 写出的任务记录目录
    bool overwrite = false;
    /// false 时只打印结论、不报任务记录的位置：调用方把它写在临时目录里，评测完删掉。
    bool keep_record = true;
    /// 结论里的名字，放进对比表时认它。空表示取估计轨迹的文件名。
    std::string name{};
    /// 评测函数，名字 → 插件节点。空表示 ate 与 rpe 都用默认参数。
    config::PluginMap evaluators{};
    /// 目标窗口：任务完成度按它算。空表示不算任务完成度。
    std::optional<fw::TimeRange> window{};
};

/// 只评测、写结论：别的系统跑出来的轨迹进对比表，不判通过与否。
/// 返回退出码：0 评测完成（没测出东西的评测函数在结论里写无结果），1 出错。
int evaluateTrajectories(const TrajectoryEvaluation& request, const config::Catalogs& catalogs,
                         std::ostream& out, std::ostream& err);

/// 配置命令：schema、default、check、dump。不是配置命令时返回 -1，用法错误返回 2。
int configCommand(const std::vector<std::string>& args, const config::Catalogs& catalogs,
                  std::ostream& out, std::ostream& err);

/// 命令行入口：
///
///     <任务.yaml> [路径=值 …] [--overwrite] [--jobs=N]
///                                             跑任务（也可以写成 run <任务.yaml> …）；
///                                             多序列时同时跑 N 个序列
///     evaluate <任务记录> [evaluation.…=值 …]  单独评测，不重跑
///     check <任务.yaml> [路径=值 …]           校验，插件节点由目录还原
///     dump  <任务.yaml> [路径=值 …]           还原后的完整任务
///     schema                                  配置 schema，含目录里每个实现的字段
///     default                                 全部默认值
int run(int argc, char** argv, const config::Catalogs& catalogs);

}  // namespace simpleslam::app
