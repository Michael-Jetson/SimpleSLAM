# SimpleSLAM 教程：从零开始构建 SLAM 程序

> 本教程面向 SLAM 初学者，从环境搭建到自行添加组件，一步步学会使用 SimpleSLAM 框架。
> 前置要求：C++ 基础（类、模板、智能指针）、了解 3D 点云和刚体变换的概念。

---

## 目录

- [第零章：环境搭建与第一次运行](#第零章环境搭建与第一次运行)
- [第一章：理解框架架构](#第一章理解框架架构)
- [第二章：点云数据处理](#第二章点云数据处理)
- [第三章：传感器数据读取](#第三章传感器数据读取)
- [第四章：Topic 通信系统](#第四章topic-通信系统)
- [第五章：实现你的第一个 Odometry](#第五章实现你的第一个-odometry)
- [第六章：使用 Runner 运行完整管线](#第六章使用-runner-运行完整管线)
- [第七章：共享资源层](#第七章共享资源层)
- [第八章：添加后端服务](#第八章添加后端服务)
- [第九章：类型擦除与可配置化](#第九章类型擦除与可配置化)
- [第十章：进阶主题](#第十章进阶主题)

---

## 第零章：环境搭建与第一次运行

### 0.1 获取代码

```bash
git clone https://github.com/Michael-Jetson/SimpleSLAM.git
cd SimpleSLAM
```

### 0.2 Docker 构建（推荐）

Docker 方式不影响宿主机环境，一键搞定所有依赖：

```bash
# 1. 预下载依赖源码（避免容器内 GitHub 访问不稳定）
git clone --branch v3.4.0 --depth 1 https://github.com/catchorg/Catch2.git docker/deps/catch2
git clone --branch master --depth 1 https://github.com/artivis/manif.git docker/deps/manif
git clone --branch v1.6.1 --depth 1 https://github.com/jlblancoc/nanoflann.git docker/deps/nanoflann
git clone --branch v1.3.0 --depth 1 https://github.com/Tessil/robin-map.git docker/deps/robin-map

# 2. 构建并运行
docker run --rm -v $(pwd):/workspace -w /workspace osrf/ros:humble-desktop-full \
    bash -c "bash scripts/install_deps.sh && cmake -B build -DBUILD_TESTING=ON && \
             cmake --build build -j && cd build && ctest --output-on-failure"
```

### 0.3 原生构建

```bash
# Ubuntu 22.04+, GCC 11+
bash scripts/install_deps.sh
cmake -B build -DBUILD_TESTING=ON
cmake --build build -j
cd build && ctest --output-on-failure
```

### 0.4 验证

看到类似输出即成功：

```
100% tests passed, 0 tests failed out of 182
```

---

## 第一章：理解框架架构

### 1.1 五大组件

SimpleSLAM 由五个顶层组件构成，它们之间是**数据流协作**关系：

```
SensorIO ──▶ Odometry ──▶ Topic<T> 通信 ──▶ Backend 服务
                                                  │
                          Resources（共享数据容器）◀─┘
                          Infrastructure（日志、配置、计时）
```

| 组件 | 职责 | 类比 |
|------|------|------|
| **SensorIO** | 读取传感器数据 | 数据源适配器 |
| **Odometry** | 计算每帧位姿 | SLAM 前端 |
| **Backend** | 回环检测、全局优化 | SLAM 后端 |
| **Resources** | 关键帧、位姿图等共享数据 | 共享数据库 |
| **Infrastructure** | 日志、配置、计时 | 基础设施 |

### 1.2 三层使用深度

| 层级 | 方式 | 适用场景 |
|------|------|---------|
| **L3** | `#include` 单个组件当独立库 | 算法研究、快速实验 |
| **L2** | 写 `main.cpp`，手工组装组件 | 系统工程、自定义管线 |
| **L1** | 编辑 YAML，一行命令运行 | 快速验证、对比实验 |

本教程从 L3 开始，逐步进入 L2。

### 1.3 核心数据流

一个完整的离线 SLAM 管线数据流：

```
KittiSource ──读取──▶ LidarScan ──▶ Odometry.processLidar()
                                           │
                                    OdometryResult（位姿）
                                           │
                               ┌───────────┴───────────┐
                               ▼                       ▼
                        Trajectory.append()     Topic<KeyframeEvent>
                        (记录轨迹)                     │
                                              ┌────────┴────────┐
                                              ▼                 ▼
                                       LoopDetector      Visualizer
                                       (回环检测)         (可视化)
```

### 1.4 关键类型

| 类型 | 头文件 | 说明 |
|------|--------|------|
| `SE3d` | `core/types/geometry.hpp` | 6 自由度刚体变换（manif 李群） |
| `LidarScan` | `core/types/sensor_data.hpp` | SOA 点云帧，必选 points + 可选字段 |
| `OdometryResult` | `core/types/odometry_result.hpp` | 一帧位姿估计结果 |
| `KeyframeData` | `core/types/keyframe.hpp` | 关键帧（id + 位姿 + 传感器数据） |

---

## 第二章：点云数据处理

本章学习 SimpleSLAM 的点云工具，不需要理解 SLAM 算法。

### 2.1 LidarScan——点云容器

`LidarScan` 使用 SOA（Structure of Arrays）布局，每个属性是独立数组：

```cpp
#include <SimpleSLAM/core/types/sensor_data.hpp>
using namespace simpleslam;

// 创建一个点云
LidarScan scan;
scan.timestamp = 1.0;
scan.points = {
    {1.0f, 0.0f, 0.0f},
    {2.0f, 1.0f, 0.5f},
    {3.0f, 2.0f, 1.0f},
};

// 可选字段——只有需要时才创建
scan.intensities = std::vector<float>{0.5f, 0.8f, 0.3f};
scan.normals = std::vector<Eigen::Vector3f>{
    {0, 0, 1}, {0, 0, 1}, {0, 0, 1}
};

// 检查一致性（所有可选字段长度 == points.size()）
assert(scan.isConsistent());
```

为什么用 SOA 而不是 AOS？配准热路径只遍历 `points` 数组，连续内存访问，缓存最优。

### 2.2 体素降采样

```cpp
#include <SimpleSLAM/core/math/point_ops.hpp>

// 创建一个包含 10000 个点的点云
LidarScan scan;
scan.points.resize(10000);
// ... 填充点坐标 ...

// 体素降采样：0.5m 体素大小
LidarScan downsampled = voxelDownsample(scan, 0.5f);
// 点数大幅减少，但保留了空间分布
```

### 2.3 其他点云操作

```cpp
#include <SimpleSLAM/core/math/point_ops.hpp>

// 距离过滤：只保留 1m ~ 50m 的点
LidarScan filtered = rangeFilter(scan, 1.0f, 50.0f);

// 包围盒裁剪：只保留 [-10, 10] x [-10, 10] x [-2, 5] 的点
LidarScan cropped = cropBox(scan,
    Eigen::Vector3f(-10, -10, -2),
    Eigen::Vector3f(10, 10, 5));

// 刚体变换：把点云从传感器坐标系变换到世界坐标系
SE3d T_world_sensor(Eigen::Vector3d(1, 0, 0), Eigen::Quaterniond::Identity());
LidarScan transformed = transformScan(scan, T_world_sensor);

// 统计离群点移除：K=10 近邻，标准差阈值 1.0
LidarScan clean = statisticalOutlierRemoval(scan, 10, 1.0f);
```

### 2.4 KD-Tree 近邻搜索

```cpp
#include <SimpleSLAM/core/math/kdtree.hpp>

// 用点云构建 KD-Tree
KDTree3f tree(scan.points);

// K 近邻搜索：找 query 点的 5 个最近邻
Eigen::Vector3f query(1.0f, 0.0f, 0.0f);
auto [indices, distances] = tree.knnSearch(query, 5);

// 半径搜索：找半径 0.5m 内的所有点
auto [r_indices, r_distances] = tree.radiusSearch(query, 0.5f);
```

### 2.5 法向量估计

```cpp
#include <SimpleSLAM/core/math/normal_estimation.hpp>

// 用 KNN PCA 估计法向量（K=20 近邻）
estimateNormals(scan, 20);

// 结果存在 scan.normals 中
assert(scan.hasNormals());
```

### 2.6 PCD 文件读写

```cpp
#include <SimpleSLAM/core/math/pcd_io.hpp>

// 写入 PCD 二进制文件
writePCD("output.pcd", scan);

// 读取 PCD 文件
LidarScan loaded = readPCD("output.pcd");
```

### 2.7 练习

1. 读取一个 PCD 文件，降采样到 0.3m 体素，估计法向量，保存结果
2. 对同一点云做不同体素大小的降采样（0.1m, 0.5m, 1.0m），观察点数变化
3. 用 cropBox 裁剪出车辆前方 20m 范围的点云

---

## 第三章：传感器数据读取

### 3.1 ISensorSource 接口

所有数据源实现统一接口：

```cpp
class ISensorSource {
public:
    virtual bool hasNext() const = 0;           // 还有数据吗？
    virtual std::optional<LidarScan> nextScan() = 0;  // 下一帧 LiDAR
    virtual std::optional<ImuSample> nextImu();       // 下一个 IMU 样本
    virtual std::optional<ImageFrame> nextImage();    // 下一帧图像
    virtual Timestamp currentTimestamp() const = 0;
};
```

### 3.2 读取 KITTI 数据集

```cpp
#include <SimpleSLAM/sensor_io/kitti_source.hpp>

// KITTI 数据集目录结构：
// sequences/00/
//   velodyne/   ← .bin 点云文件
//   times.txt   ← 时间戳

KittiSource source("path/to/sequences/00");

int frame = 0;
while (source.hasNext()) {
    auto scan = source.nextScan();
    if (!scan) continue;

    std::cout << "Frame " << frame++
              << ": " << scan->size() << " points"
              << " @ t=" << scan->timestamp << "\n";
}
```

### 3.3 读取 EuRoC 数据集

```cpp
#include <SimpleSLAM/sensor_io/euroc_source.hpp>

// EuRoC MAV 数据集提供相机 + IMU（无 LiDAR）
EurocSource source("path/to/MH_01_easy/mav0");

while (source.hasNext()) {
    // EuRoC 无 LiDAR，nextScan() 始终返回 nullopt
    auto imu = source.nextImu();
    if (imu) {
        std::cout << "IMU: acc=" << imu->acc.transpose()
                  << " gyro=" << imu->gyro.transpose() << "\n";
    }

    auto image = source.nextImage();
    if (image) {
        std::cout << "Image: " << image->width << "x" << image->height << "\n";
    }
}
```

### 3.4 多源合并

```cpp
#include <SimpleSLAM/sensor_io/sensor_mux.hpp>

SensorMux mux;
mux.addSource(std::make_unique<KittiSource>("path/to/00"));
// 可添加更多数据源...

while (mux.hasNext()) {
    auto scan = mux.nextScan();  // 返回时间戳最早的数据
    // ...
}
```

### 3.5 练习

1. 写一个程序读取 KITTI 数据集，打印每帧的点数和时间戳
2. 对每帧做降采样 + 距离过滤后保存为 PCD 文件
3. 统计整个数据集的总点数和平均每帧点数

---

## 第四章：Topic 通信系统

Topic 是 SimpleSLAM 的进程内发布-订阅通信机制，类似 ROS 2 话题但更轻量。

### 4.1 核心概念

```
Publisher ──publish()──▶ Topic<T> ──回调──▶ Subscriber
                            │
                     QoS: Event / Stream / Latest
```

- **消息不可变**：通过 `shared_ptr<const T>` 传递，多个订阅者安全共享
- **publish 延迟 <100ns**：函数调用级别，无序列化
- **Offline 模式**：消息进入 pending 队列，`drainAll()` 逐轮分发（确定性）
- **Online 模式**：直接在发布者线程中调用回调（低延迟）

### 4.2 第一个发布-订阅示例

```cpp
#include <SimpleSLAM/core/infra/topic_hub.hpp>
#include <iostream>

using namespace simpleslam;

int main() {
    // 创建隔离实例（测试用；生产用 TopicHub::init()）
    TopicHub hub(true);  // true = offline 模式

    // 创建 Publisher
    auto pub = hub.createPublisherImpl<std::string>("chat/message");

    // 创建 Subscriber（lambda 回调）
    auto sub = hub.subscribeImpl<std::string>("chat/message",
        [](MsgPtr<std::string> msg) {
            std::cout << "收到消息: " << *msg << "\n";
        });

    // 发布消息
    pub.publish(std::string("Hello SimpleSLAM!"));
    pub.publish(std::string("Topic 通信真简单"));

    // Offline 模式必须手动 drain
    hub.drainAll();

    return 0;
}
```

输出：

```
收到消息: Hello SimpleSLAM!
收到消息: Topic 通信真简单
```

### 4.3 成员函数回调

```cpp
struct FrameCounter {
    int count = 0;

    // 回调签名：const T&（最推荐）
    void onMessage(const std::string& msg) {
        ++count;
        std::cout << "[" << count << "] " << msg << "\n";
    }
};

// 绑定成员函数——不需要 std::bind 和占位符
FrameCounter counter;
auto sub = hub.subscribeImpl<std::string>("chat/message",
    detail::wrapCallback<std::string>(&FrameCounter::onMessage, &counter));
```

### 4.4 使用全局单例

生产代码使用全局 TopicHub（参考 spdlog 模式）：

```cpp
// Runner 启动时初始化
TopicHub::init(true);

// 任何模块直接使用静态方法
auto pub = TopicHub::createPublisher<int>("sensor/count");
auto sub = TopicHub::createSubscriber<int>("sensor/count",
    [](const int& v) { std::cout << "count=" << v << "\n"; });

pub.publish(42);
TopicHub::instance().drainAll();

// Runner 关闭时清理
TopicHub::shutdown();
```

### 4.5 三种 QoS

```cpp
// Event: 可靠传递，不丢消息（关键帧事件、回环事件）
auto pub_event = hub.createPublisherImpl<KeyframeEvent>("slam/keyframe");

// Stream: 高频数据流，可丢旧帧（传感器数据）
auto pub_stream = hub.createPublisherImpl<LidarScan>("sensor/lidar", QoS::Stream);

// Latest: 只保留最新值（实时位姿）
auto pub_latest = hub.createPublisherImpl<OdometryResult>("slam/odometry", QoS::Latest);

// Latest 模式可主动拉取最新值
auto latest = hub.getLatestImpl<OdometryResult>("slam/odometry");
if (latest) {
    std::cout << "最新位姿时间戳: " << latest->timestamp << "\n";
}
```

### 4.6 BFS 事件链

Offline 模式下，`drainAll()` 按 BFS 逐轮分发——回调中 publish 的消息进入下一轮：

```cpp
TopicHub hub(true);
auto pub_a = hub.createPublisherImpl<int>("topic/a");
auto pub_b = hub.createPublisherImpl<int>("topic/b");

// topic/a 的回调向 topic/b 发布新消息
auto sub_a = hub.subscribeImpl<int>("topic/a",
    [&](MsgPtr<int> msg) {
        std::cout << "A收到: " << *msg << "\n";
        pub_b.publish(*msg + 100);   // 进入下一轮
    });

auto sub_b = hub.subscribeImpl<int>("topic/b",
    [](MsgPtr<int> msg) {
        std::cout << "B收到: " << *msg << "\n";
    });

pub_a.publish(1);
hub.drainAll();  // 第一轮处理 A，第二轮处理 B
// 输出:
// A收到: 1
// B收到: 101
```

这就是 SLAM 管线的事件链模式：Odometry 发布关键帧 → LoopDetector 处理 → PGO 优化 → 校正事件。

### 4.7 练习

1. 创建一个 Topic `"sensor/temperature"`，发布 3 个温度值，订阅者打印每次接收
2. 实现一个 3 节点事件链：A → B → C，每级加工数据后传递给下一级
3. 用 `QoS::Latest` 模式发布 10 个值，验证 `getLatest()` 只返回最后一个

---

## 第五章：实现你的第一个 Odometry

### 5.1 OdometryBase 接口

所有里程计实现继承 `OdometryBase`：

```cpp
class OdometryBase {
public:
    // 必须覆盖
    virtual OdometryResult processLidar(const LidarScan& scan) = 0;
    virtual void reset() = 0;
    virtual std::string_view name() const = 0;

    // 可选覆盖
    virtual void initialize(TopicHub& hub);   // 创建 Publisher
    virtual OdometryResult processLidarImu(   // LIO 模式
        const LidarScan& scan, std::span<const ImuSample> imu);
    virtual void shutdown();

    // 同步回调槽（外部可 hook）
    CallbackSlot<const SE3d&> on_after_prediction;
    CallbackSlot<const OdometryResult&> on_after_update;
    CallbackSlot<const KeyframeData&> on_keyframe;

protected:
    void publishResult(const OdometryResult& result);
    void publishKeyframe(const KeyframeData& kf);
};
```

### 5.2 IdentityOdometry——最简实现

从最简单开始：每帧返回单位位姿（不做任何配准）：

```cpp
#include <SimpleSLAM/odometry/odometry_base.hpp>
#include <SimpleSLAM/odometry/keyframe_selector.hpp>

class IdentityOdometry final : public simpleslam::OdometryBase {
public:
    simpleslam::OdometryResult processLidar(
            const simpleslam::LidarScan& scan) override {
        simpleslam::OdometryResult result;
        result.timestamp = scan.timestamp;
        result.pose = simpleslam::SE3d();  // 单位变换（永远在原点）
        result.status = simpleslam::TrackingStatus::Tracking;
        result.is_keyframe = kf_selector_.shouldSelect(result.pose, result.timestamp);

        if (result.is_keyframe) {
            kf_selector_.update(result.pose, result.timestamp);

            simpleslam::KeyframeData kf;
            kf.id = next_kf_id_++;
            kf.timestamp = result.timestamp;
            kf.pose = result.pose;
            kf.scan = std::make_shared<const simpleslam::LidarScan>(scan);
            publishKeyframe(kf);
        }

        publishResult(result);
        return result;
    }

    void reset() override {
        kf_selector_.reset();
        next_kf_id_ = 0;
    }

    std::string_view name() const override { return "IdentityOdometry"; }

private:
    simpleslam::KeyframeSelector kf_selector_;
    uint64_t next_kf_id_ = 0;
};
```

### 5.3 KeyframeSelector 使用

关键帧选择器基于三个阈值判定：

```cpp
#include <SimpleSLAM/odometry/keyframe_selector.hpp>

// 自定义阈值
simpleslam::KeyframeCriteria criteria;
criteria.min_distance = 2.0;      // 移动 2m 以上
criteria.min_angle_deg = 15.0;    // 旋转 15 度以上
criteria.max_interval = 3.0;      // 最多间隔 3 秒

simpleslam::KeyframeSelector selector(criteria);

// shouldSelect 是纯查询（不修改状态）
bool is_kf = selector.shouldSelect(current_pose, current_time);

// 确认后才调用 update
if (is_kf) {
    selector.update(current_pose, current_time);
}
```

`shouldSelect` 和 `update` 分离的好处：可以先查询、再配准验证、最后确认。

### 5.4 CallbackSlot 扩展点

CallbackSlot 允许外部代码在 Odometry 处理流程中介入：

```cpp
IdentityOdometry odom;

// 监控每帧位姿更新
odom.on_after_update.connect([](const simpleslam::OdometryResult& result) {
    std::cout << "位姿更新: t=" << result.timestamp
              << " status=" << static_cast<int>(result.status) << "\n";
});

// 监控关键帧产出
odom.on_keyframe.connect([](const simpleslam::KeyframeData& kf) {
    std::cout << "关键帧 #" << kf.id
              << " 点数=" << (kf.scan ? kf.scan->size() : 0) << "\n";
});
```

### 5.5 IncrementalOdometry 骨架——为 ICP 做准备

下一步你会实现真正的配准算法。这是骨架：

```cpp
// 伪代码——展示 Odometry 内部的典型处理流程
class SimpleLoOdometry final : public simpleslam::OdometryBase {
    simpleslam::OdometryResult processLidar(
            const simpleslam::LidarScan& scan) override {
        // 1. 预处理
        auto processed = simpleslam::voxelDownsample(scan, 0.5f);

        // 2. 配准（你的算法在这里）
        // SE3d T_delta = yourIcpSolver(target_map_, processed, current_pose_);
        // current_pose_ = current_pose_ * T_delta;

        // 3. 更新地图
        // target_map_.update(processed, current_pose_);

        // 4. 填充结果
        simpleslam::OdometryResult result;
        result.timestamp = scan.timestamp;
        result.pose = current_pose_;
        result.status = simpleslam::TrackingStatus::Tracking;

        // 5. 关键帧判定
        result.is_keyframe = kf_selector_.shouldSelect(current_pose_, scan.timestamp);
        if (result.is_keyframe) {
            kf_selector_.update(current_pose_, scan.timestamp);
            // ... 创建 KeyframeData 并发布 ...
        }

        publishResult(result);
        return result;
    }

    // ... reset(), name() ...
};
```

### 5.6 练习

1. 实现 IdentityOdometry，用 TopicHub 验证 on_keyframe 回调触发
2. 修改 IdentityOdometry 让每帧位姿沿 X 轴匀速前进（模拟直线运动）
3. 调整 KeyframeCriteria 的三个阈值，观察关键帧数量变化

---

## 第六章：使用 Runner 运行完整管线

### 6.1 OfflineRunner

OfflineRunner 是离线模式的主循环编排器：

```
构造 → initialize → while(hasNext) { nextScan → processLidar → drainAll } → shutdown
```

```cpp
#include <SimpleSLAM/runner/offline_runner.hpp>
#include <SimpleSLAM/sensor_io/kitti_source.hpp>
// #include "identity_odometry.hpp"  // 你的实现

int main() {
    auto source = std::make_unique<simpleslam::KittiSource>("path/to/kitti/00");
    auto odom = std::make_unique<IdentityOdometry>();

    simpleslam::OfflineRunner runner(std::move(source), std::move(odom));

    // 运行！
    auto result = runner.run();

    std::cout << "处理帧数: " << result.frames_processed << "\n"
              << "关键帧数: " << result.keyframes << "\n"
              << "耗时: " << result.elapsed_seconds << "s\n";

    // 导出轨迹供 evo 评测
    runner.trajectory().exportTUM("estimated_trajectory.txt");

    return 0;
}
```

### 6.2 OfflineRunner 做了什么

1. **构造时**：初始化 `TopicHub`（离线模式），调用 `odometry->initialize()`
2. **run() 中**：初始化所有 Service → 主循环 → 关闭所有 Service
3. **主循环**：`nextScan()` → `processLidar()` → `trajectory.append()` → `drainAll()`
4. **析构时**：`odometry->shutdown()` → `TopicHub::shutdown()`

### 6.3 添加帧数限制

```cpp
// 只处理前 100 帧（调试用）
auto result = runner.run(100);
```

### 6.4 轨迹评测

```bash
# 安装 evo
pip install evo

# 使用 SimpleSLAM 自带评测脚本
python scripts/evaluate.py ground_truth.txt estimated_trajectory.txt \
    --format tum --metric both --plot
```

轨迹文件格式（TUM）：

```
# timestamp tx ty tz qx qy qz qw
0.000000 0.0 0.0 0.0 0.0 0.0 0.0 1.0
0.100000 0.1 0.0 0.0 0.0 0.0 0.0 1.0
...
```

### 6.5 练习

1. 用 IdentityOdometry + KittiSource 跑 KITTI 00 的前 50 帧
2. 导出 TUM 轨迹，用 `evo_traj tum estimated.txt -p` 可视化
3. 对比 IdentityOdometry（静止）和匀速前进的轨迹差异

---

## 第七章：共享资源层

Resources 是框架的共享数据容器，提供并发安全的读写接口。

### 7.1 CurrentState——实时位姿双缓冲

```cpp
#include <SimpleSLAM/resources/current_state.hpp>

simpleslam::CurrentState state;

// Odometry 线程写入里程计位姿
state.publishOdom(odom_pose);

// 任意线程读取
auto snapshot = state.readOdom();

// PGO 写入校正变换
state.publishCorrection(correction_pose);

// 获取校正后的位姿：T_correction * T_odom
simpleslam::SE3d corrected = state.correctedPose();
```

### 7.2 KeyframeStore——写后不可变

```cpp
#include <SimpleSLAM/resources/keyframe_store.hpp>

simpleslam::KeyframeStore store;

// 插入关键帧（插入后核心字段不可变）
simpleslam::KeyframeData kf;
kf.id = 1;
kf.timestamp = 1.0;
kf.pose = simpleslam::SE3d();
store.insert(kf);

// 读取（shared_ptr<const> 零拷贝）
auto kf_ptr = store.get(1);
if (kf_ptr) {
    std::cout << "关键帧 #" << kf_ptr->id << "\n";
}

// 扩展属性（运行时灵活添加）
store.setExtension<std::string>(1, "description", "走廊入口");
auto desc = store.getExtension<std::string>(1, "description");
```

### 7.3 PoseGraph——位姿图

```cpp
#include <SimpleSLAM/resources/pose_graph.hpp>

simpleslam::PoseGraph graph;

// 添加节点
graph.addNode({0, 0.0, simpleslam::SE3d()});
graph.addNode({1, 1.0, some_pose});

// 添加里程计边
simpleslam::PoseGraphEdge edge;
edge.from_id = 0;
edge.to_id = 1;
edge.T_from_to = relative_pose;
edge.type = simpleslam::EdgeType::Odometry;
graph.addEdge(edge);

// 全局优化后批量更新位姿
std::unordered_map<uint64_t, simpleslam::SE3d> optimized;
optimized[0] = corrected_pose_0;
optimized[1] = corrected_pose_1;
graph.updatePoses(optimized);

// 深拷贝快照（不持有锁的情况下安全读取）
auto snapshot = graph.snapshot();
```

### 7.4 Trajectory——轨迹记录与导出

```cpp
#include <SimpleSLAM/resources/trajectory.hpp>

simpleslam::Trajectory traj;
traj.append(0.0, pose_0);
traj.append(0.1, pose_1);
traj.append(0.2, pose_2);

// 插值查询
auto interpolated = traj.poseAt(0.15);  // 在 pose_1 和 pose_2 之间插值

// 导出
traj.exportTUM("trajectory.txt");
traj.exportKITTI("trajectory_kitti.txt");
```

### 7.5 ExtrinsicsManager——传感器外参

```cpp
#include <SimpleSLAM/resources/extrinsics_manager.hpp>

simpleslam::ExtrinsicsManager extrinsics;
extrinsics.registerSensor("lidar", T_body_lidar);
extrinsics.registerSensor("camera", T_body_camera);

// 查询
simpleslam::SE3d T = extrinsics.getExtrinsic("lidar");
```

---

## 第八章：添加后端服务

后端服务通过 Topic 话题订阅事件，独立于 Odometry 运行。

### 8.1 ServiceBase 模式

```cpp
class ServiceBase {
public:
    explicit ServiceBase(std::string service_name);
    virtual void initialize(TopicHub& hub);  // 订阅 Topic
    virtual void shutdown();                  // 释放资源
};
```

### 8.2 实现一个 KeyframeLogger

最简单的后端服务——打印每个关键帧的信息：

```cpp
#include <SimpleSLAM/backend/service_base.hpp>
#include <SimpleSLAM/core/infra/topic_hub.hpp>
#include <SimpleSLAM/core/infra/topic_names.hpp>
#include <SimpleSLAM/core/types/event_types.hpp>
#include <iostream>

class KeyframeLogger final : public simpleslam::ServiceBase {
public:
    KeyframeLogger() : ServiceBase("KeyframeLogger") {}

    void initialize(simpleslam::TopicHub& hub) override {
        sub_ = hub.subscribeImpl<simpleslam::KeyframeEvent>(
            std::string(simpleslam::topic_names::kSlamKeyframe),
            [this](simpleslam::MsgPtr<simpleslam::KeyframeEvent> msg) {
                onKeyframe(*msg);
            });
    }

private:
    void onKeyframe(const simpleslam::KeyframeEvent& kf) {
        std::cout << "[KeyframeLogger] KF #" << kf.keyframe_id
                  << " t=" << kf.timestamp
                  << " points=" << (kf.scan ? kf.scan->size() : 0)
                  << "\n";
    }

    simpleslam::SubscriptionHandle sub_;
};
```

### 8.3 注册到 Runner

```cpp
simpleslam::OfflineRunner runner(std::move(source), std::move(odom));
runner.addService(std::make_unique<KeyframeLogger>());
runner.run();
```

KeyframeLogger 会在每个关键帧产出时被 TopicHub 的 BFS 分发自动调用。

### 8.4 LoopDetector Concept

要实现回环检测，你的类需要满足 LoopDetector concept：

```cpp
#include <SimpleSLAM/core/concepts/loop_detector.hpp>

// concept LoopDetector = requires(T& d, const KeyframeData& kf) {
//     { d.addKeyframe(kf) } -> same_as<void>;
//     { d.detect(kf) } -> convertible_to<optional<LoopCandidate>>;
// };

class MyLoopDetector final {
public:
    void addKeyframe(const simpleslam::KeyframeData& kf) {
        // 添加到描述子数据库
    }

    std::optional<simpleslam::LoopCandidate> detect(
            const simpleslam::KeyframeData& kf) {
        // 搜索回环候选
        // 如果找到，返回 LoopCandidate{match_id, T_match_query, score}
        return std::nullopt;
    }
};

// 编译期验证
static_assert(simpleslam::LoopDetector<MyLoopDetector>);
```

### 8.5 使用框架内置的 LoopClosureService

框架已提供 `LoopClosureService`——一个完整的回环检测后端服务骨架。你只需实现满足 `LoopDetector` concept 的检测器，用 `AnyLoopDetector` 类型擦除包装后传入：

```cpp
#include <SimpleSLAM/backend/loop_closure_service.hpp>
#include <SimpleSLAM/core/concepts/any_loop_detector.hpp>

// 用你的检测器创建服务
auto svc = std::make_unique<simpleslam::LoopClosureService>(
    simpleslam::AnyLoopDetector(MyLoopDetector{}));

// 注册到 Runner
runner.addService(std::move(svc));
```

LoopClosureService 内部自动完成：
1. 订阅 `slam/keyframe` 话题
2. 将 `KeyframeEvent` 转换为 `KeyframeData` 并传给检测器
3. 检测到回环时发布 `LoopDetectedEvent` 到 `slam/loop` 话题

### 8.6 使用框架内置的 PgoService

类似地，`PgoService` 是位姿图优化的服务骨架。你实现满足 `PoseGraphOptimizer` concept 的优化器：

```cpp
#include <SimpleSLAM/core/concepts/pose_graph_optimizer.hpp>

class MyPGOptimizer final {
public:
    void addOdometryEdge(uint64_t from, uint64_t to,
                         const simpleslam::SE3d& rel,
                         const simpleslam::Mat6d& info) {
        // 添加里程计边到因子图
    }

    void addLoopEdge(uint64_t from, uint64_t to,
                     const simpleslam::SE3d& rel,
                     const simpleslam::Mat6d& info) {
        // 添加回环边
    }

    simpleslam::OptimizationResult optimize() {
        // 求解因子图
        simpleslam::OptimizationResult result;
        result.converged = true;
        // result.optimized_poses = ...;
        return result;
    }

    void reset() { /* 清空因子图 */ }
};

static_assert(simpleslam::PoseGraphOptimizer<MyPGOptimizer>);
```

然后用 `AnyPoseGraphOptimizer` 包装传入 PgoService：

```cpp
#include <SimpleSLAM/backend/pgo_service.hpp>
#include <SimpleSLAM/core/concepts/any_pose_graph_optimizer.hpp>

auto pgo = std::make_unique<simpleslam::PgoService>(
    simpleslam::AnyPoseGraphOptimizer(MyPGOptimizer{}));
runner.addService(std::move(pgo));
```

PgoService 自动完成：
1. 订阅 `slam/keyframe`（构建里程计边）和 `slam/loop`（添加回环边）
2. 每检测到回环立即优化
3. 收敛时发布 `CorrectionEvent` 到 `slam/correction`

### 8.7 完整回环管线示例

```cpp
auto source = std::make_unique<simpleslam::KittiSource>("path/to/00");
auto odom = std::make_unique<MyOdometry>();

simpleslam::OfflineRunner runner(std::move(source), std::move(odom));

// 添加回环检测服务
runner.addService(std::make_unique<simpleslam::LoopClosureService>(
    simpleslam::AnyLoopDetector(MyScanContextDetector{})));

// 添加 PGO 服务
runner.addService(std::make_unique<simpleslam::PgoService>(
    simpleslam::AnyPoseGraphOptimizer(MyGtsamOptimizer{})));

// 运行——事件链自动串联：
// Keyframe → LoopClosureService → LoopDetectedEvent → PgoService → CorrectionEvent
auto result = runner.run();
```

### 8.8 SubMapManager——子图生命周期

`SubMapManager` 管理子图的创建、关键帧分配、冻结和 PGO 校正：

```cpp
#include <SimpleSLAM/backend/submap_manager.hpp>

simpleslam::SubMapManagerConfig config;
config.max_keyframes_per_submap = 50;  // 每 50 个关键帧切换子图

simpleslam::SubMapManager mgr(config);

// 创建第一个子图
uint64_t submap_id = mgr.createSubmap(initial_pose);

// 每个关键帧添加到活跃子图
mgr.addKeyframe(keyframe_id);

// 检查是否需要冻结
if (mgr.shouldFreezeActive()) {
    uint64_t new_id = mgr.freezeAndCreateNew(current_pose);
}

// PGO 校正后更新锚点位姿
mgr.applyCorrections(optimized_submap_poses);
```

### 8.9 练习

1. 实现 KeyframeLogger，接入 OfflineRunner，观察关键帧输出
2. 实现一个 FrameCounter 服务，统计总帧数并在 shutdown 时打印
3. 实现一个回环检测的"假"实现（每 20 个关键帧报告一次假回环），用 LoopClosureService + PgoService 验证完整事件链
4. 使用 SubMapManager 管理子图，观察冻结和切换行为

---

## 第九章：类型擦除与可配置化

### 9.1 RegistrationTarget Concept

配准目标的核心接口：

```cpp
template <typename T>
concept RegistrationTarget = requires(T& target,
    const LidarScan& scan, const SE3d& pose, MatchResult& result) {
    { target.match(scan, pose, result) } -> std::same_as<void>;
    { target.update(scan, pose) } -> std::same_as<void>;
    { target.empty() } -> std::convertible_to<bool>;
    { target.size() } -> std::convertible_to<std::size_t>;
};
```

### 9.2 实现一个简单的 RegistrationTarget

```cpp
class NaiveNearestTarget final {
public:
    void match(const simpleslam::LidarScan& scan,
               const simpleslam::SE3d& pose,
               simpleslam::MatchResult& result) {
        result.clear();
        if (map_points_.empty()) return;

        auto transformed = simpleslam::transformScan(scan, pose);
        // 暴力最近邻（仅供教学，实际用 KDTree）
        for (const auto& p : transformed.points) {
            float min_dist = std::numeric_limits<float>::max();
            for (const auto& mp : map_points_) {
                float d = (p - mp).norm();
                if (d < min_dist) min_dist = d;
            }
            if (min_dist < 1.0f) {
                result.residuals.push_back(min_dist);
                ++result.num_valid;
            }
        }
    }

    void update(const simpleslam::LidarScan& scan,
                const simpleslam::SE3d& pose) {
        auto transformed = simpleslam::transformScan(scan, pose);
        map_points_.insert(map_points_.end(),
            transformed.points.begin(), transformed.points.end());
    }

    bool empty() const { return map_points_.empty(); }
    size_t size() const { return map_points_.size(); }

private:
    std::vector<Eigen::Vector3f> map_points_;
};

// 编译期验证
static_assert(simpleslam::RegistrationTarget<NaiveNearestTarget>);
```

### 9.3 AnyRegistrationTarget——运行时多态

当你有多种 RegistrationTarget 实现，想通过配置选择时：

```cpp
#include <SimpleSLAM/core/concepts/any_registration_target.hpp>

// 包装具体实现
simpleslam::AnyRegistrationTarget target(NaiveNearestTarget{});

// 通过统一接口使用（一次虚分派，开销可忽略）
simpleslam::MatchResult result;
target.match(scan, pose, result);
target.update(scan, pose);

// AnyRegistrationTarget 自身也满足 RegistrationTarget concept
static_assert(simpleslam::RegistrationTarget<simpleslam::AnyRegistrationTarget>);
```

### 9.4 AnyLoopDetector——回环检测器类型擦除

与 AnyRegistrationTarget 完全同构：

```cpp
#include <SimpleSLAM/core/concepts/any_loop_detector.hpp>

// 包装你的检测器
simpleslam::AnyLoopDetector detector(MyScanContextDetector{});

// 通过统一接口使用
detector.addKeyframe(kf);
auto candidate = detector.detect(kf);

// AnyLoopDetector 自身满足 LoopDetector concept
static_assert(simpleslam::LoopDetector<simpleslam::AnyLoopDetector>);
```

### 9.5 AnyPoseGraphOptimizer——优化器类型擦除

```cpp
#include <SimpleSLAM/core/concepts/any_pose_graph_optimizer.hpp>

simpleslam::AnyPoseGraphOptimizer opt(MyGtsamOptimizer{});

opt.addOdometryEdge(from, to, relative, info);
opt.addLoopEdge(from, to, relative, info);
auto result = opt.optimize();

static_assert(simpleslam::PoseGraphOptimizer<simpleslam::AnyPoseGraphOptimizer>);
```

### 9.6 三种类型擦除的统一模式

SimpleSLAM 的三种类型擦除包装（AnyRegistrationTarget、AnyLoopDetector、AnyPoseGraphOptimizer）都遵循 Sean Parent 模式：

| 类型擦除 | 包装的 Concept | 用于 |
|---------|---------------|------|
| `AnyRegistrationTarget` | `RegistrationTarget` | Odometry 内部地图选择 |
| `AnyLoopDetector` | `LoopDetector` | LoopClosureService 回环算法选择 |
| `AnyPoseGraphOptimizer` | `PoseGraphOptimizer` | PgoService 优化器选择 |

所有三种都是 move-only、单虚分派、自身满足对应 concept（static_assert 验证）。

### 9.7 练习

1. 实现 NaiveNearestTarget，配合 IdentityOdometry 使用（只做地图构建，不做配准）
2. 把 NaiveNearestTarget 包装到 AnyRegistrationTarget 中使用
3. 实现另一个 RegistrationTarget（如基于体素的），通过 AnyRegistrationTarget 切换
4. 实现一个 MockLoopDetector（固定频率返回假回环），通过 AnyLoopDetector 包装后传入 LoopClosureService

---

## 第十章：进阶主题

### 10.1 HealthMonitor——系统健康监控

```cpp
#include <SimpleSLAM/core/infra/health_monitor.hpp>

simpleslam::HealthThresholds thresholds;
thresholds.degraded_frames_threshold = 5;   // 连续 5 帧坏 → Degraded
thresholds.lost_frames_threshold = 20;      // 连续 20 帧坏 → Lost
thresholds.recovery_frames = 3;             // 连续 3 帧好 → 降级恢复

simpleslam::HealthMonitor monitor(thresholds);

// 每帧更新
simpleslam::HealthMetrics metrics;
metrics.tracking_status = odom_result.status;
metrics.covariance_trace = odom_result.covariance.trace();
monitor.update(metrics);

// 检查状态
if (monitor.state() == simpleslam::SystemHealth::Lost) {
    // 触发重定位...
}
```

状态转移：`OK → Degraded → Lost → Failed（终态）`，恢复是阶梯式（不跳级）。

### 10.2 ImuBuffer——IMU 数据管理

LIO 模式需要管理高频 IMU 数据流。`ImuBuffer` 提供线程安全的缓冲区，支持时间窗查询和插值：

```cpp
#include <SimpleSLAM/odometry/imu_buffer.hpp>

simpleslam::ImuBuffer buffer(10000);  // 最多保留 10000 个样本

// 传感器线程写入
buffer.addSample(imu_sample);
buffer.addBatch(imu_batch);  // 批量添加

// 里程计线程查询：获取 [t_start, t_end] 内的 IMU 数据
auto samples = buffer.query(scan_start_time, scan_end_time);

// 精确时间点插值（线性插值 acc 和 gyro）
auto interpolated = buffer.interpolateAt(exact_timestamp);

// 清理旧数据
buffer.trimBefore(oldest_needed_timestamp);
```

`query()` 支持边界插值——如果 t_start/t_end 落在两个样本之间，返回的结果包含插值的边界样本，确保 IMU 预积分获得精确的起止数据。

### 10.3 Submap 架构

```cpp
#include <SimpleSLAM/backend/submap/submap.hpp>

simpleslam::Submap submap;
submap.id = 0;
submap.T_world_submap = simpleslam::SE3d();  // 锚点位姿
submap.frozen = false;                        // 可写
submap.addKeyframe(1);
submap.addKeyframe(2);

// 冻结后变为只读
submap.frozen = true;

// 回环校正只更新锚点位姿，不修改子图内部点坐标（FUTURE_ONLY）
// submap.T_world_submap = corrected_pose;  // O(1) 操作
```

### 10.4 GTSAM 适配器

```cpp
// 条件编译：只有启用 GTSAM 时才可用
#ifdef SIMPLESLAM_HAS_GTSAM
#include <SimpleSLAM/adapters/gtsam_bridge.hpp>

// manif SE3d <-> gtsam::Pose3 转换
simpleslam::SE3d pose = ...;
gtsam::Pose3 gtsam_pose = simpleslam::toGtsam(pose);
simpleslam::SE3d back = simpleslam::fromGtsam(gtsam_pose);

// 信息矩阵顺序转换（manif [t,r] <-> GTSAM [r,t]）
Eigen::Matrix<double, 6, 6> info_manif = ...;
auto info_gtsam = simpleslam::toGtsamInfoOrder(info_manif);
#endif
```

### 10.5 添加新的 SensorSource

实现 `ISensorSource` 接口即可支持新数据源：

```cpp
class MyRosbagSource final : public simpleslam::ISensorSource {
public:
    explicit MyRosbagSource(const std::string& bag_path) {
        // 打开 rosbag...
    }

    bool hasNext() const override {
        // 检查 bag 是否还有消息
    }

    std::optional<simpleslam::LidarScan> nextScan() override {
        // 从 bag 读取下一帧 PointCloud2，转换为 LidarScan
    }

    std::optional<simpleslam::ImuSample> nextImu() override {
        // 从 bag 读取下一个 IMU 消息
    }

    simpleslam::Timestamp currentTimestamp() const override {
        // 当前消息时间戳
    }
};
```

### 10.6 下一步

框架层已就绪。要构建真正的 SLAM 系统，下一步实现：

1. **VoxelHashTarget**：体素哈希地图 + 点到点残差（对标 KISS-ICP）
2. **ICP-SVD 求解器**：基于残差和雅可比求解位姿增量
3. **LoIcpOdometry**：组合 VoxelHashTarget + ICP-SVD 的完整 LO
4. **run_slam_offline.cpp**：可执行入口，KITTI 数据集上评测

精度目标：KITTI 00-10 ATE 达到 KISS-ICP 论文水平。

---

## 附录：常用头文件速查

| 功能 | 头文件 |
|------|--------|
| 几何类型（SE3d, Vec3d） | `<SimpleSLAM/core/types/geometry.hpp>` |
| 点云容器 | `<SimpleSLAM/core/types/sensor_data.hpp>` |
| 点云操作 | `<SimpleSLAM/core/math/point_ops.hpp>` |
| KD-Tree | `<SimpleSLAM/core/math/kdtree.hpp>` |
| 体素网格 | `<SimpleSLAM/core/math/voxel_grid.hpp>` |
| 法向量估计 | `<SimpleSLAM/core/math/normal_estimation.hpp>` |
| PCD 读写 | `<SimpleSLAM/core/math/pcd_io.hpp>` |
| Topic 通信 | `<SimpleSLAM/core/infra/topic_hub.hpp>` |
| 话题名常量 | `<SimpleSLAM/core/infra/topic_names.hpp>` |
| OdometryBase | `<SimpleSLAM/odometry/odometry_base.hpp>` |
| 关键帧选择 | `<SimpleSLAM/odometry/keyframe_selector.hpp>` |
| OfflineRunner | `<SimpleSLAM/runner/offline_runner.hpp>` |
| 关键帧存储 | `<SimpleSLAM/resources/keyframe_store.hpp>` |
| 位姿图 | `<SimpleSLAM/resources/pose_graph.hpp>` |
| 轨迹导出 | `<SimpleSLAM/resources/trajectory.hpp>` |
| 后端服务基类 | `<SimpleSLAM/backend/service_base.hpp>` |
| 回环检测服务 | `<SimpleSLAM/backend/loop_closure_service.hpp>` |
| PGO 服务 | `<SimpleSLAM/backend/pgo_service.hpp>` |
| 子图管理 | `<SimpleSLAM/backend/submap_manager.hpp>` |
| 健康监控 | `<SimpleSLAM/core/infra/health_monitor.hpp>` |
| IMU 缓冲区 | `<SimpleSLAM/odometry/imu_buffer.hpp>` |
| 类型擦除（配准） | `<SimpleSLAM/core/concepts/any_registration_target.hpp>` |
| 类型擦除（回环） | `<SimpleSLAM/core/concepts/any_loop_detector.hpp>` |
| 类型擦除（PGO） | `<SimpleSLAM/core/concepts/any_pose_graph_optimizer.hpp>` |
| 全部核心 | `<SimpleSLAM/core.hpp>` |
| 全部资源 | `<SimpleSLAM/resources.hpp>` |
| 全部里程计 | `<SimpleSLAM/odometry.hpp>` |
| 全部后端 | `<SimpleSLAM/backend.hpp>` |
