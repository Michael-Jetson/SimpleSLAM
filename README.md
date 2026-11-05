# SimpleSLAM

面向视觉-激光-惯性三模态的**模块化 C++20 SLAM 框架**，兼顾定位与建图。

## 项目定位

SimpleSLAM 不是又一个 SLAM 算法实现，而是一个让 SLAM 工程师能快速验证算法组合的构建框架。通过 YAML 配置切换 LIO / VIO / LIVO 方案，在每种方案内部替换子算法，无需修改框架代码。

**SimpleSLAM 不是什么**：
- 不是万能机器人框架——专注 SLAM，不做导航规划和运动控制
- 不重新发明优化器——后端使用 GTSAM / Ceres 等成熟因子图库
- 不依赖 ROS 编译——核心库纯 C++20，ROS 2 集成是可选适配层
- 不追求"任意模块自由替换"——模块化边界由 SLAM 的数学结构决定

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
- **Infrastructure** — 日志、配置、计时、时钟等非算法功能

## 核心设计法则

1. **模块化边界跟随数学接缝** — 在真正存在多种实现的地方设可替换边界，紧耦合的地方拒绝拆分
2. **经 profile 确认的热路径必须单态** — 逐元素循环保证内联和向量化；跨模块插件边界每帧仅常数次虚分派
3. **永不"停止世界"** — 回环校正通过 FUTURE_ONLY + per-submap 变换实现，不阻塞前端
4. **故障隔离** — 后端故障不传播到前端，Odometry 始终继续输出位姿
5. **架构是重构出来的** — 先让算法跑通，再根据实际需求引入抽象

约束性全文见 `docs/architecture.md`。

## 三层使用深度

| 层级 | 用户画像 | 使用方式 |
|------|---------|---------|
| L1 — YAML 驱动 | 算法评测、快速验证 | 编辑 YAML，运行 `simpleslam_run` |
| L2 — 手工装配 | 系统工程师 | 写 `main.cpp`，自由组装组件 |
| L3 — 原始组件 | 算法研究者 | 直接 `#include` 单个组件当独立库用 |

## 当前状态

仓库里有两套代码，迁移是施工项，两套不得在同一翻译单元混用：

- **新栈**（`feat/comm-runtime` 分支，2026-09 起）：进程内通信运行时、框架契约、模块层，以及评测、录制回放、配置三层工程设施，设计见 `docs/design/README.md`。目标是写算法的人只实现算法接口，其余工程事务由框架承担。**目前只有占位算法，还没有真实算法跑过。**
- **旧栈**：v0 基础设施、v0.5 点云原语、v1.0 纯 LO（`configs/lo_kissicp.yaml` 可跑）。精度基线验收待做，验收后版本号才进位 1.0.0。

手写算法从 `examples/lio_skeleton/my_lio.hpp` 的 `track()` 开始；不下载数据集也能冒烟测试整条链路：

```bash
python3 scripts/make_kitti_fixture.py --out /tmp/fx          # 合成最小 KITTI 夹具
build/examples/simpleslam-lio-skeleton configs/new_stack/kitti_odometry.yaml \
    dataset.path=/tmp/fx/odometry/sequences/00
```

路线图、版本与不做的事见 `docs/future/roadmap.md`，未决问题见 `docs/backlog.md`。

## 快速开始

### 环境要求

- Docker（推荐）或 Ubuntu 24.04+
- GCC 13+（C++20 支持；基线见 `docs/engineering.md` 第 3.11 节）

### Docker 构建

```bash
# 1. 构建开发镜像（Ubuntu 24.04；源码依赖由镜像的 deps-builder 阶段自行从 GitHub 拉取，
#    受限网络用 --network=host，详见 docs/guides/docker.md）
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

## 目录结构

```
include/SimpleSLAM/   # 公共头文件
  旧栈： core/（types、concepts、math、infra）、odometry/、backend/、resources/、sensor_io/、runner/、adapters/
  新栈： comm/（通信运行时）、framework/（框架契约与资源）、plugins/（算法插件——写算法只碰这里）
         modules/（模块层：把插件接到总线与资源上）、eval/（评测）、record/（录制回放）、config/（配置）
src/                  # 各模块库实现（纯头模块仅 CMakeLists）
apps/                 # simpleslam_run（L1 入口，YAML 驱动离线回放）
tools/                # simpleslam-config（配置 schema / 校验 / 哈希）、simpleslam-eval（精度与基线门禁）
examples/lio_skeleton # 能跑的新栈入口：数据集 → 装配 → 跑完 → 导出 → 评测；算法在 my_lio.hpp
python/simpleslam/    # Python 侧：配置渲染、轨迹读写、指标（schema 读自 C++，不重写）
configs/  tests/  docker/  scripts/  docs/
```

每个新栈代码目录的设计与文件表，在 `docs/design/` 下同名的文档里。

## 文档

从 [`docs/README.md`](docs/README.md) 开始：它按你要做的事指路，并说明文档怎么组织。

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

依赖不全时可以只构建新栈：`cmake -B build -DSIMPLESLAM_NEW_STACK_ONLY=ON`。按版本引入的可选依赖（GTSAM、small_gicp、Ceres、OpenCV）见 `docs/engineering.md` 第 7.6 节，镜像见 `docs/guides/docker.md`。

编码规范与完整工程规范（版本与发布、变更控制、代码、构建与 CI、测试、文档、依赖、安全、完成定义）见 `docs/engineering.md`。

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
