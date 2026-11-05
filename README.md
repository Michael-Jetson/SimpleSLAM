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
+----------------------------------------------------------------------+
|  SimpleSLAM                                                          |
|                                                                      |
|  SensorIO --> Odometry --> Topic<T> + CallbackSlots                  |
|  (数据接入)    (LIO/VIO/       +----------+----------+              |
|                 LVIO/LO)        v          v          v              |
|                              核心后端   基础后端   扩展后端          |
|                             (回环/PGO) (子图管理) (可视化...)        |
|                                                                      |
|  资源层:     KeyframeStore | CurrentState | PoseGraph                |
|  基础设施:   Logger | Config | Timing | Clock                        |
|  Runner:     组装全部组件 + 管理生命周期 + 四种运行模式              |
+----------------------------------------------------------------------+
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

代码完成：v0 基础设施、v0.5 点云原语、v1.0 纯 LO（`configs/lo_kissicp.yaml` 可跑；精度基线验收待做，验收后版本号才进位 1.0.0）。下一步 v1.5 LIO 核心。

| 版本 | 内容 | 状态 |
|------|------|------|
| v1.0 | 纯 LO（VoxelHashTarget、IcpSolver、LoIcpOdometry） | 代码完成，精度基线验收待做 |
| v1.5 | LIO 核心（ESIKF、IMU 预积分、ikd-Tree、iVox） | 计划中 |
| v2.0 | 回环 + PGO（ScanContext、GTSAM iSAM2） | 计划中 |
| v2.5 | 回环扩展 + 基础稠密地图 + 重定位 | 计划中 |
| v3.0–v4.5 | 视觉特征 → VIO → LIVO | 计划中 |
| v5.0+ | Research Backlog（GNSS/持久化/可视化/Python 绑定等） | 单独立项后进入主线 |

版本轴唯一：路线图里程碑即软件版本，完整路线图与阶段 Not-Doing 清单见 `docs/blueprint.md` §17。

精度目标：v1.0 KITTI ATE 达 KISS-ICP 级，v1.5 达 FAST-LIO2 级（EuRoC ATE < 0.12m）

## 快速开始

### 环境要求

- Docker（推荐）或 Ubuntu 24.04+
- GCC 13+（C++20 支持；基线见 `docs/blueprint.md` §14.4）

### Docker 构建

```bash
# 1. 构建开发镜像（Ubuntu 24.04；源码依赖由镜像的 deps-builder 阶段自行从 GitHub 拉取，
#    受限网络用 --network=host，详见 docs/Docker.md）
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
include/SimpleSLAM/   # 公共头文件：core/（types、concepts、math、infra）、odometry/、backend/、resources/、sensor_io/、runner/、adapters/
src/                  # 各模块库实现（纯头模块仅 CMakeLists）
apps/                 # simpleslam_run（L1 入口，YAML 驱动离线回放）
configs/  tests/  docker/  scripts/  docs/
```

概念到头文件、测试、架构锚点的逐条映射见 `docs/map.md`；文档体系（约束性架构 / ADR / 提案 / 调研 / 课程）见 `docs/README.md`。

## 依赖

### 核心依赖（simpleslam_core）

| 库 | 用途 | 类型 | 引入版本 |
|---|------|------|---------|
| Eigen 3.4+ | 矩阵运算、四元数 | header-only | v0 |
| manif | SE3d/SO3d 李群操作 | header-only | v0 |
| spdlog | 异步日志 | 编译库 | v0 |
| yaml-cpp | YAML 配置 | 编译库 | v0 |
| fmt | 字符串格式化 | header-only | v0 |
| nanoflann | KD-Tree 近邻搜索 | header-only | v0.5 |
| tsl::robin_map | 高性能哈希表（体素哈希） | header-only | v0.5 |
| Catch2 3.x | 单元测试 | 编译库 | v0 |

### 可选依赖（按版本引入）

| 库 | 用途 | 引入版本 |
|---|------|---------|
| GTSAM 4.3+ | 因子图优化 | v2.0 |
| small_gicp | 回环验证 ICP | v2.0 |
| Ceres 2.1+ | 滑窗 BA | v3.5 |
| OpenCV 4.5+ | 视觉前端 | v3.0 |

## 编码规范

| 类目 | 规范 | 示例 |
|------|------|------|
| 文件名 | `snake_case.hpp` / `.cpp` | `timing.hpp` |
| 类名 | `PascalCase` | `TimingManager` |
| 函数/变量 | `camelCase` | `processFrame` |
| 成员变量 | `snake_case_` | `root_` |
| 常量 | `kPascalCase` | `kMaxSamples` |
| 所有具体类 | 标注 `final` | `class SystemClock final` |
| 头文件保护 | `#pragma once` | |
| 技术标准 | 语言锁 C++20；库特性允许 C++23 的 `std::expected` | 政策见蓝图 §14.4 |

完整工程规范（版本与发布、变更控制、代码、构建与 CI、测试、文档、依赖、安全、完成定义）见 `docs/engineering.md`。

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
