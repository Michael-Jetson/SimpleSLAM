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

| 版本 | 内容 | 状态 | 测试 |
|------|------|------|------|
| **v0** | 基础设施（日志、配置、计时、时钟、几何类型） | **已完成** | 16 |
| **v0.5** | 点云工具 + Sensor IO（KDTree、VoxelGrid、降采样、KittiSource） | **已完成** | 54 |
| **v1.0 基础设施** | Topic 通信、共享资源、Concept 定义、PCD IO、EuRoC、后端框架 | **已完成** | 113 |
| v1.0 算法 | 纯 LO（VoxelHashTarget、ICP-SVD、Runner） | **待实现** | — |
| v1.5 | LIO 核心（ESIKF、IMU 预积分、ikd-Tree、iVox） | 计划中 | — |
| v2.0 | 回环 + PGO（ScanContext、GTSAM iSAM2、SubMapManager） | 计划中 | — |
| v3.0+ | VIO / LIVO / 传感器扩展 / Python 绑定 | 计划中 | — |

精度目标：v1.5 达到 FAST-LIO2 级（EuRoC ATE < 0.12m，KITTI ATE < 3m）

## 快速开始

### 环境要求

- Docker（推荐）或 Ubuntu 22.04+
- GCC 11+（C++20 支持）

### Docker 构建

```bash
# 1. 预下载依赖源码（容器内 GitHub 访问可能不稳定）
git clone --branch v3.4.0 --depth 1 https://github.com/catchorg/Catch2.git docker/deps/catch2
git clone --branch master --depth 1 https://github.com/artivis/manif.git docker/deps/manif
git clone --branch v1.6.1 --depth 1 https://github.com/jlblancoc/nanoflann.git docker/deps/nanoflann
git clone --branch v1.3.0 --depth 1 https://github.com/Tessil/robin-map.git docker/deps/robin-map

# 2. 构建开发镜像
docker build -f docker/Dockerfile.ubuntu2204 -t simpleslam-dev .

# 3. 编译 + 测试（113 个测试）
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
SimpleSLAM/
├── include/SimpleSLAM/
│   ├── core.hpp                    # 核心模块 umbrella header
│   ├── resources.hpp               # 共享资源 umbrella header
│   ├── core/
│   │   ├── convention.hpp          # 框架公约（ENU、T_world_body、精度约定）
│   │   ├── types/                  # 核心数据类型
│   │   │   ├── common.hpp          #   Timestamp, PointXYZI
│   │   │   ├── geometry.hpp        #   SE3d, SO3d, Vec3d (manif 别名)
│   │   │   ├── sensor_data.hpp     #   LidarScan (SOA), ImageFrame, ImuSample
│   │   │   ├── odometry_result.hpp #   OdometryResult, TrackingStatus
│   │   │   ├── keyframe.hpp        #   KeyframeData
│   │   │   └── event_types.hpp     #   KeyframeEvent, LoopDetectedEvent...
│   │   ├── concepts/               # C++20 concept 接口定义
│   │   │   ├── registration_target.hpp  # RegistrationTarget + MatchResult
│   │   │   ├── loop_detector.hpp        # LoopDetector + LoopCandidate
│   │   │   └── pose_graph_optimizer.hpp # PoseGraphOptimizer + OptimizationResult
│   │   ├── math/                   # 数学工具
│   │   │   ├── kdtree.hpp          #   nanoflann 零拷贝 KDTree3f
│   │   │   ├── voxel_grid.hpp      #   VoxelMap (FNV + Morton)
│   │   │   ├── point_ops.hpp       #   降采样、过滤、变换、SOR
│   │   │   ├── normal_estimation.hpp#  KNN PCA 法向量估计
│   │   │   ├── scan_utils.hpp      #   extractByIndices, RangeImageView
│   │   │   ├── lie_utils.hpp       #   hat/vee/perturb/Jacobian
│   │   │   └── pcd_io.hpp          #   PCD 二进制读写
│   │   └── infra/                  # 基础设施 + 通信
│   │       ├── logger.hpp          #   spdlog 封装
│   │       ├── config.hpp          #   YAML 层级加载 + schema 校验
│   │       ├── timing.hpp          #   RAII 计时 + 统计
│   │       ├── clock.hpp           #   时钟抽象（系统/数据集）
│   │       ├── topic.hpp           #   Topic<T> 发布-订阅（3 种 QoS）
│   │       ├── topic_hub.hpp       #   TopicHub 全局单例 + BFS drain
│   │       ├── topic_names.hpp     #   话题名常量
│   │       └── callback_slot.hpp   #   同步回调槽
│   ├── resources/                  # 共享资源容器
│   │   ├── current_state.hpp       #   T_odom + T_correction 双缓冲
│   │   ├── pose_graph.hpp          #   关键帧节点 + 边（shared_mutex）
│   │   ├── keyframe_store.hpp      #   关键帧存储（写后不可变）
│   │   ├── extrinsics_manager.hpp  #   传感器外参管理
│   │   └── trajectory.hpp          #   位姿历史 + TUM/KITTI 导出
│   ├── sensor_io/                  # 传感器接入
│   │   ├── sensor_source.hpp       #   ISensorSource 接口
│   │   ├── kitti_source.hpp        #   KITTI 数据集读取器
│   │   ├── euroc_source.hpp        #   EuRoC MAV 数据集读取器
│   │   └── sensor_mux.hpp          #   多源时间排序合并
│   ├── adapters/                   # 类型转换桥接
│   │   └── gtsam_bridge.hpp        #   manif ↔ GTSAM（条件编译）
│   └── backend/                    # 后端服务框架
│       ├── service_base.hpp        #   可选生命周期基类
│       └── submap/submap.hpp       #   子图数据容器
├── core/src/                       # 核心库实现
├── resources/src/                  # 资源层实现
├── sensor_io/src/                  # 传感器 IO 实现
├── configs/                        # 参考配置
├── tests/unit/                     # 113 个单元测试
├── docker/                         # Docker 开发环境 + 预下载依赖
├── scripts/                        # 工具脚本
└── docs/                           # 架构文档
```

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
