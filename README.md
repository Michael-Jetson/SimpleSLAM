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
2. **热路径必须单态** — C++20 concept 约束的模板保证内联和向量化，运行时多态只在启动期使用
3. **永不"停止世界"** — 回环校正通过 FUTURE_ONLY + per-submap 变换实现，不阻塞前端
4. **故障隔离** — 后端故障不传播到前端，Odometry 始终继续输出位姿
5. **架构是重构出来的** — 先让算法跑通，再根据实际需求引入抽象

## 三层使用深度

| 层级 | 用户画像 | 使用方式 |
|------|---------|---------|
| L1 — YAML 驱动 | 算法评测、快速验证 | 编辑 YAML，运行 `run_slam_offline` |
| L2 — 手工装配 | 系统工程师 | 写 `main.cpp`，自由组装组件 |
| L3 — 原始组件 | 算法研究者 | 直接 `#include` 单个组件当独立库用 |

## 当前状态

| 版本 | 内容 | 状态 |
|------|------|------|
| **v0** | 基础设施（日志、配置、计时、时钟、几何类型） | **已完成** |
| **v0.5** | 纯激光里程计（点云数据结构、RegistrationTarget、VoxelHashTarget、KITTI） | 进行中 |
| v1.0 | LIO + 回环（ESIKF、IMU 预积分、ScanContext、GTSAM iSAM2） | 计划中 |
| v1.5 | LIVO + 三地图族（相机前端、Ceres 滑窗 BA） | 计划中 |
| v2.0 | 子图持久化 + 资源分页 + Python 绑定 | 计划中 |

精度目标：v1.0 达到 FAST-LIO2 级（EuRoC ATE < 0.12m，KITTI ATE < 3m）

## 快速开始

### 环境要求

- Docker（推荐）或 Ubuntu 22.04+
- GCC 11+（C++20 支持）

### Docker 构建

```bash
# 1. 预下载依赖源码（容器内 GitHub 访问可能不稳定）
git clone --branch v3.4.0 --depth 1 https://github.com/catchorg/Catch2.git docker/deps/catch2
git clone --branch master --depth 1 https://github.com/artivis/manif.git docker/deps/manif

# 2. 构建开发镜像
docker build -f docker/Dockerfile.ubuntu2204 -t simpleslam-dev .

# 3. 编译 + 测试
docker run --rm -v $(pwd):/workspace -w /workspace simpleslam-dev \
    bash -c "cmake -B build -DBUILD_TESTING=ON && cmake --build build -j && cd build && ctest --output-on-failure"
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
simpleslam/
├── core/                           # simpleslam_core 库（纯 C++20）
│   ├── include/simpleslam/
│   │   ├── convention.hpp          # 框架公约常量
│   │   ├── types/                  # 核心数据类型
│   │   │   ├── common.hpp          #   Timestamp, PointXYZI
│   │   │   ├── geometry.hpp        #   SE3d, SO3d, Vec3d (manif 别名)
│   │   │   └── sensor_data.hpp     #   LidarScan, ImageFrame, ImuSample
│   │   ├── concepts/               # C++20 concept 接口定义
│   │   │   └── registration_target.hpp
│   │   ├── math/                   # 数学工具（点云操作、李群工具）
│   │   └── infra/                  # 基础设施
│   │       ├── logger.hpp          #   spdlog 封装（per-module、async）
│   │       ├── config.hpp          #   YAML 层级加载 + schema 校验
│   │       ├── timing.hpp          #   RAII 计时 + 统计（p50/p95/p99）
│   │       └── clock.hpp           #   时钟抽象（系统/数据集）
│   └── src/
├── registration/                   # RegistrationTarget 实现
├── odometry/                       # Odometry 实现
├── backend/                        # 后端服务
├── resources/                      # 共享资源
├── sensor_io/                      # 传感器接入
├── runner/                         # 系统组装 + 运行入口
├── apps/                           # 可执行文件
├── configs/                        # 参考配置
├── tests/                          # 单元测试 + 契约测试 + 集成测试
├── docker/                         # Docker 开发环境
├── scripts/                        # 工具脚本
└── docs/                           # 架构文档
```

## 依赖

### 核心依赖（simpleslam_core）

| 库 | 用途 | 类型 |
|---|------|------|
| Eigen 3.4+ | 矩阵运算、四元数 | header-only |
| manif | SE3d/SO3d 李群操作 | header-only |
| spdlog | 异步日志 | 编译库 |
| yaml-cpp | YAML 配置 | 编译库 |
| fmt | 字符串格式化 | header-only |
| Catch2 3.x | 单元测试 | 编译库 |

### 可选依赖（按版本引入）

| 库 | 用途 | 引入版本 |
|---|------|---------|
| nanoflann | KD-Tree 近邻搜索 | v0.5 |
| tsl::robin_map | 高性能哈希表 | v0.5 |
| GTSAM 4.3+ | 因子图优化 | v1.0 |
| small_gicp | 回环验证 ICP | v1.0 |
| Ceres 2.1+ | 滑窗 BA | v1.5 |
| OpenCV 4.5+ | 视觉前端 | v1.5 |

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
| 技术标准 | C++20（不使用 C++23） | |

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
