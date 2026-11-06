# SimpleSLAM

面向视觉、激光、IMU、GNSS 四种模态及其组合的**模块化 C++20 SLAM 框架**，兼顾定位与建图。

## 项目定位

SimpleSLAM 不是又一个 SLAM 算法实现，而是一个让 SLAM 工程师能快速验证算法组合的构建框架。

- **有限自由内的任意组合**：用一份 YAML 装配完整系统，选管线、选算法（LO / LIO / VIO / LIVO 等）、选每一步的实现。每个算法声明自己哪几步可以替换、每一步要求什么接口；满足接口的实现都能换上，比如换一种地图表示，或者换掉预处理、去畸变、初始化、视觉前端里的某一步。在这个范围之内任意组合都成立。用户自己写的模块和插件也一样用 YAML 装配，满足约束即可；约束在装配时核对。换实现只改配置，不改框架代码；超出范围的选择在运行之前就报错。
- **模态**：视觉（单目、双目 RGB、深度相机）、激光（点云）、IMU、GNSS，以及它们的组合。
- **运行形态**：一个可执行文件，相当于 ROS 的一个节点。内部按模块分线程，模块之间用自带的进程内通信，只求轻量和实时；对外接 ROS 等系统时，经可选的适配模块。
- **扩展表征**：几何地图之外，支持 3DGS、语义、场景图等表征。它们都由关键帧和位姿图派生，回环后随之更新。

**SimpleSLAM 不是什么**：
- 不是万能机器人框架——专注 SLAM，不做导航规划和运动控制
- 不重新发明优化器——后端使用 GTSAM / Ceres 等成熟因子图库
- 不依赖 ROS 编译——核心库纯 C++20，ROS 2 集成是可选适配层
- 不追求不受约束的任意替换——可替换的范围由算法按 SLAM 的数学结构声明，紧耦合的地方不拆；范围之内任意组合

## 架构概览

```
数据来源 ─▶ 传感器接入（打包）─▶ 里程计 ─┬─▶ 轨迹收集
                 ▲                │  ▲   └─▶ 位置识别 ─▶ 图优化 ─┐
                 └── 处理进度 ─────┘  └───────── 校正 ───────────┘

资源：关键帧库 · 位姿图 · 当前状态 · 标定          （版本化快照 + 单写者令牌）
运行时：话题 · 服务 · 参数 · 定时器 · 模块收件箱    （进程内，用法接近 ROS1）
工程设施：评测与精度基线 · 录制回放 · 配置
```

五个顶层组件通过**数据流协作**，不是层叠依赖：

- **SensorIO** — 对接外部数据源，统一转换为内部类型
- **Odometry** — 前端里程计，输出实时位姿（ESIKF / 滑窗 BA / ICP 求解）
- **Backend** — 一组可独立启停的后端服务（回环检测、位姿图优化、重定位等）
- **Resources** — 被动的共享数据容器（关键帧、位姿图、外参等）
- **Infrastructure** — 通信运行时、配置、评测、录制回放等非算法功能

一个算法由三类东西组成：**模块**负责线程、总线和生命周期；**插件**是按帧调用的纯算法，比如一个完整的里程计方案；**算法库**里是插件内部可以替换的步骤，比如扫描滤波和配准地图。

## 核心设计法则

1. **模块化边界跟随数学接缝** — 在真正存在多种实现的地方设可替换边界，紧耦合的地方拒绝拆分
2. **经 profile 确认的热路径必须单态** — 逐元素循环保证内联和向量化；跨模块插件边界每帧仅常数次虚分派
3. **永不"停止世界"** — 后端优化不持有前端需要的锁，前端不等全局优化，校正在帧边界生效
4. **故障隔离** — 后端故障不传播到前端，Odometry 始终继续输出位姿
5. **设计先行** — 先研究清楚，经过充分分析得到 v1 方案再施工；执行中只做小的优化（v2），不先草草设计、再大量返工
6. **冗余只为应急和必要的安全** — 队列为突发留足余量；持续跟不上要优化算法，不靠框架兜底
7. **简洁与功能统一** — 功能用简洁的结构实现，不为了功能堆出难以维护、难以开发的复杂架构

## 三层使用深度

| 层级 | 用户画像 | 使用方式 |
|------|---------|---------|
| L1 — YAML 驱动 | 算法评测、快速验证 | 写一份任务 YAML：哪个系统、哪份数据、离线还是在线、做实验还是只出结果 |
| L2 — 手工装配 | 系统工程师 | 写 `main.cpp`，直接构造组件、按接口组装 |
| L3 — 原始组件 | 算法研究者 | 直接 `#include` 单个组件当独立库用，比如 `algo/` 里的滤波与配准地图 |

一次运行是一个**任务**：用哪个 SLAM 系统、处理哪份数据、离线还是在线、做实验还是只出结果。它由两份 YAML 描述，系统写一次，换数据、换运行方式只改任务（简化自 `configs/systems/my_lio.yaml` 与 `configs/tasks/kitti_odometry.yaml`）：

```yaml
# 系统：管线、模块与插件。带 type 的节点选一个实现，其余键是它的参数，槽可以嵌套
pipeline: odometry
modules:
  odometry:
    plugin: {type: my_lio, keyframe_distance_m: 1.5}
```

```yaml
# 任务：在 KITTI 00 上离线跑这个系统，并对真值评测
task: {name: kitti-00, purpose: experiment, mode: offline}
system: ../systems/my_lio.yaml
data:
  source: {type: kitti_odometry, path: /data/kitti/odometry/sequences/00}
evaluation: {alignment: se3}
```

命令行上的 `路径=值` 覆盖任意字段，例如 `system.modules.odometry.plugin.keyframe_distance_m=2`。结果写进一个任务档案目录：还原后的完整任务、身份与哈希、运行结局、三条轨迹、评测报告；同一个任务原样重跑，得到同一个任务哈希。

写错了在运行之前就报，带完整路径和可选项，例如
`system.modules.odometry.plugin.map.type = voxel_downsample：这里要求 LinearizableMap，voxel_downsample 是 ScanFilter 的实现；满足的实现有 voxel_point_map`。

## 当前状态

版本号分两级 vX.Y：大改动升大版本，小改动升小版本，每个版本都有对应的 git 标签。

仓库里有两套代码，迁移是施工项，两套不得在同一翻译单元混用：

- **新栈**（2026-09 起）：进程内通信运行时、框架契约、模块层，以及评测、录制回放、配置三层工程设施。目标是写算法的人只实现算法接口，其余工程事务由框架承担。
  - **插件槽与目录**（`config/plugin.hpp`）：每个接口一张目录，YAML 里按类型名选实现。未知参数、类型不符、取值不合法都在装配时报错；内置实现由框架登记，用户实现在应用入口追加，不改框架代码。
  - **任务与执行器**（`app/`）：一份任务 YAML 引用一份系统 YAML；执行器还原任务、打开数据、按管线装配、离线跑完、写任务档案、评测并过基线门禁。一个任务也可以列出一个数据集的多个序列（如 KITTI 00–10），每个序列在一个子进程里跑，最后汇总成一张表。用户的入口只有几行：登记自己的实现，再交给执行器。任务也能用 Python 写，方便按传感器参数计算配置、批量生成扫参任务（`configs/tasks/example.py`）。
  - **算法库**（`algo/`）：可替换步骤的接口与第一批实现。扫描滤波有 `voxel_downsample` 和 `none`；配准地图有 `voxel_point_map`（体素哈希、点到点，对标 KISS-ICP）。每个接口带契约测试模板，内置实现逐个自动受测。

  **目前还没有真实算法跑过**：示例里只有占位的 `MyLio`，第一个方案还在写。
- **旧栈**：基础设施、点云原语与纯 LO（`configs/lo_kissicp.yaml` 可跑），精度基线验收待做。

接下来：

1. LO 方案 `lo_icp`：用上面的滤波与地图，在 KITTI 上和旧栈 LO 对比轨迹。
2. 第二种地图（面元地图）：通过契约测试后，只改 YAML 就能让 LO 换用它；选一个不满足接口的实现，装配时报错。
3. LIO：复用同一个地图接口。
4. 执行器：在线运行。

手写算法从 `examples/lio_skeleton/my_lio.hpp` 的 `track()` 开始；不下载数据集也能冒烟测试整条链路：

```bash
python3 scripts/make_kitti_fixture.py --out /tmp/fx          # 合成最小 KITTI 夹具
build/examples/simpleslam-lio-skeleton configs/tasks/kitti_odometry.yaml \
    data.source.path=/tmp/fx/odometry/sequences/00 outputs.dir=/tmp/fx/run
build/examples/simpleslam-lio-skeleton dump configs/tasks/kitti_odometry.yaml   # 还原后的完整任务
PYTHONPATH=python python3 -m simpleslam --tool build/examples/simpleslam-lio-skeleton \
    config render configs/tasks/example.py -o /tmp/fx/example.yaml          # Python 写的任务
```

## 快速开始

### 环境要求

- Docker（推荐）或 Ubuntu 24.04+
- GCC 13+（C++20）

### Docker 构建

```bash
# 1. 构建开发镜像（Ubuntu 24.04；源码依赖由镜像的 deps-builder 阶段自行从 GitHub 拉取，
#    网络受限时加 --network=host）
docker build -t simpleslam-dev .

# 2. 编译 + 测试
docker run --rm -v $(pwd):/workspace -w /workspace simpleslam-dev \
    bash -c "cmake -B build && cmake --build build -j && cd build && ctest --output-on-failure"
```

### 原生构建

```bash
# 安装依赖
bash scripts/install_deps.sh

# 编译
cmake -B build -DBUILD_TESTING=ON
cmake --build build -j

# 测试
cd build && ctest --output-on-failure
```

依赖不全时可以只构建新栈：`cmake -B build -DSIMPLESLAM_NEW_STACK_ONLY=ON`。

## 目录结构

```
include/SimpleSLAM/   # 公共头文件
  旧栈： core/（types、concepts、math、infra）、odometry/、backend/、resources/、sensor_io/、runner/、adapters/
  新栈： comm/（通信运行时）、framework/（框架契约与资源）
         algo/（可替换的步骤：扫描滤波、配准地图）
         plugins/（算法插件：里程计、位置识别、图优化，数据集读取器）
         modules/（模块层：把插件接到总线与资源上）
         eval/（评测）、record/（录制回放）、config/（配置、插件槽与目录）
         app/（入口层：目录、任务还原、执行器与任务档案）
src/                  # 各模块库实现（纯头模块仅 CMakeLists）
apps/                 # simpleslam_run（旧栈的 L1 入口，YAML 驱动离线回放）
tools/                # simpleslam（任务入口，只带内置实现）、simpleslam-config（schema / check / dump / hash）、simpleslam-eval（精度与基线门禁）
examples/lio_skeleton # 用户入口的范例：登记 my_lio，再交给执行器；算法在 my_lio.hpp
python/simpleslam/    # Python 侧：任务渲染与扫参、轨迹读写、指标（schema 读自入口，不重写）
configs/systems/      # 系统 YAML：管线、模块、插件与参数
configs/tasks/        # 任务 YAML 与 Python 写的任务
configs/*.yaml        # 旧栈的配置
tests/  docker/  scripts/
```

## 依赖

| 库 | 用途 | 类型 | 谁需要 |
|---|------|------|---------|
| Eigen 3.4+ | 矩阵运算、四元数 | header-only | 新旧栈 |
| manif | SE3d/SO3d 李群操作 | header-only | 新旧栈 |
| yaml-cpp | 配置、精度基线、回放包 | 编译库 | 新栈工程设施；旧栈硬依赖 |
| spdlog、fmt | 日志与格式化 | 编译库 / header-only | 旧栈 |
| nanoflann | KD-Tree 近邻搜索 | header-only | 旧栈 |
| tsl::robin_map | 体素哈希 | header-only | 旧栈 |
| Catch2 3.x | 单元测试 | 编译库 | 测试 |

按阶段引入的可选依赖：GTSAM 4.3+（因子图优化）、small_gicp（回环验证 ICP）、Ceres 2.1+（滑窗 BA）、OpenCV 4.5+（视觉前端）。

## 编码规范

| 类目 | 规范 | 示例 |
|------|------|------|
| 文件名 | `snake_case`，一个类型一个文件 | `voxel_point_map.hpp` |
| 类型 | `PascalCase` | `VoxelPointMap` |
| 函数 / 方法 | `camelCase` | `linearize()` |
| 变量 | `snake_case`，成员加尾下划线 | `voxel_size_m`、`config_` |
| 常量 | `kPascalCase` | `kTypeName` |
| 具体类 | 标注 `final` | `class VoxelPointMap final` |
| 接口 | 公开入口非虚，先查输入输出再调私有虚函数（NVI） | `ScanFilter::apply()` |
| 头文件 | `#pragma once`；首两行是 SPDX 许可证与版权 | |
| 格式 | clang-format：Google 基线、4 空格、100 列 | `.clang-format` |
| 语言标准 | C++20 | |

## 技术栈

```
Eigen + manif — 基础数学层（李群、矩阵运算）
    │
    ├── ESIKF / IMU 预积分 — 手写，只依赖 Eigen + manif
    │   (纯 LIO 路径在此层完全自洽)
    │
    ├── Ceres（可选）— 局部滑窗 BA / VIO
    │
    └── GTSAM（可选）— 全局因子图 / 回环优化
```

## Star History

<a href="https://www.star-history.com/?repos=Michael-Jetson%2FSimpleSLAM&type=date&legend=top-left">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/image?repos=Michael-Jetson/SimpleSLAM&type=date&theme=dark&legend=bottom-right" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/image?repos=Michael-Jetson/SimpleSLAM&type=date&legend=bottom-right" />
   <img alt="Star History Chart" src="https://api.star-history.com/image?repos=Michael-Jetson/SimpleSLAM&type=date&legend=bottom-right" />
 </picture>
</a>

## License

MIT
