SimpleSLAM设计概览
我现在是一个SLAM算法工程师，MMDet是一个基于Pytorch二次封装的框架，可以模块化的构建神经网络并且训练，我想写一个类似功能的SLAM C++软件框架来通用地完成SLAM算法，目标是搭积木一样构建SLAM系统，以此来实现SLAM项目的快速开发与验证，目前给出一个初步的设计架构方案：

整体框架设计：整个系统分为七个核心算法模块：跟踪（前端的完整里程计）、地图（地图表示、查询与新增）、位置（回环检测与重定位）、后端（长期一致性约束）、传感器IO（数据读取与预处理）和可视化（与ROS交互并可视化）、全局资源（管理长期的共享数据，如地图、状态）七大模块，每个模块实现一种功能，每个模块可以使用不同算法插件来实现功能的组合，如前端可以使用ORB描述子提取插件、点面残差提取插件等，这种做法将SLAM pipeline重新构造成最小可复用组件，开发者可以像搭乐高一样通过连接组件并设置参数来构建完整系统，类似于深度学习框架中组合网络层的方式，整个SLAM项目面向激光/视觉/惯性的多传感器融合方案
暂时无法在飞书文档外展示此内容
1. 使用功能模块和算法插件两种概念来构建SLAM系统，模块对应一个具体任务功能，算法对应一种具体实现方法（比如说视觉前端可以使用ORB描述子也可以使用光流法），以此来实现解耦和模块化，并且便于对单个算法或者模块进行测试
2. 为实现上述插件机制，可以为每类功能模块定义一个抽象基类，内部定义模块需要的主要接口函数，例如，TrackingBase定义接口Track(Frame& frame)来处理新帧位姿估计；MapBase定义接口UpdateMap(KeyFrame& kf)等，具体算法插件模块继承相应的基类并实现接口方法。通过C++的多态机制，框架核心部分只依赖基类接口编程，无需了解具体实现细节，以此便于实现模块替换
3. 使用注册表或者工厂模式来实现从字符串到模块的创建，维护一个全局注册表，便于通过YAML或其他配置文件中的参数来进行查表和实现实例化，并且便通过一个总的构建器对象搭建起来一个slam系统，如此用户只需编辑配置文件即可切换算法，同时通过C++17的内联函数来实现注册的唯一性，也就是可以在创建新模块/插件后直接注册和使用
4. 基于 Eventpp 实现轻量级进程内 TopicBus，替代 ROS 的模块通信，且进程内共享同一个管理者。各模块只需按 topic 与消息类型对齐即可发布/订阅，降低耦合、提升可移植性、避免序列化/反序列化带来的时间开销。发布非阻塞，泵线程负责分发，回调默认异步投递到线程池，避免耗时处理拖慢系统；回调既支持类内成员函数也支持类外函数，便于维护与测试。
5. 日志系统基于 spdlog 构建全局 LogManager，进程内共享同一组 sinks（终端/文件可选）与异步线程池，统一落盘。模块/插件通过 Module/Plugin 获取带层级标签的 logger（如 [Front][ORB]），自定义前缀格式化实现自动对齐。支持对象与指针两种调用（log.info / log->warn）及互转，并可按 sink 分级控制输出级别与终端颜色。
6. 使用Builder构建器作为装配线与拓扑调度器，把YAML等配置文件中的内容组织为一个可执行的SLAM系统，通过模块/插件创建、依赖注入插件指针、共享上下文的方式，构建出完整的SLAM模块化架构
7. 模块使用主被动混合的方式进行执行，关键模块如传感器IO、跟踪等有自己的线程和循环来主动执行，其他的模块如地图、后端等模块使用事件驱动+线程池方法被动执行，并且统一使用 TopicBus和Context进行交互，模块之间不存在相互调用
8. 框架将地图、状态、关键帧/约束图、回环库等长期共享数据统一封装为 Resources，由 Builder 创建并管理生命周期与持久化。各模块仅通过 Query/Write/Snapshot 能力接口访问资源，配合版本/快照与读写锁实现并发安全；写权限尽量收敛到后端/地图维护线程，前端高频仅读，以支持异步融合、大地图分区加载与可替换实现。




综合上面的要求、我给出的设计点和我思考的问题，我应该怎么设计一个很好很合理的软件架构来完成这样一个SLAM项目呢？比如说我可以很方便的定制一个模块来替换掉原slam项目中的某一个模块进行应用，可以很方便地使用不同的模块构建出一个新的slam项目，亦或者很方便新算法的加入与后续开发。请你仔细思考，如果有更好的设计方案请给出，如果有可优化的点也请给出，如果我的方案中有会导致问题的地方也指出
模块
核心作用
主要需要做的事（概览）
SensorIO（传感器IO）

统一接入多传感器数据并提供稳定数据流
读取（ROS1/2、rosbag、dataset、驱动）；时间戳标准化/缓存；基础预处理（去畸变、滤波、降采样等可选）；封装统一消息并发布到内部总线
Tracking（前端里程计）

实时估计位姿（里程计），保证稳定高频输出
特征提取/匹配或点云配准（ICP/NDT等）；IMU预积分/融合（可选）；局部优化（轻量、实时）；关键帧策略；输出高频里程计、关键帧事件与里程计约束
Mapping（地图子系统）

提供共享地图表示与查询，并负责地图演进（区域/子图）
地图数据结构（点/体素/高斯等）与索引；并发控制与快照（可选）；对外提供地图查询与更新接口；（可选）新区域建图、剔除压缩、子地图管理、保存/加载与区域缓存
Localization & Loop（位置：重定位/回环）
发现回环与提供重定位能力，向后端输出约束/先验
全局描述子构建与检索（DBoW/NetVLAD/ScanContext等）；候选验证（PnP/ICP/RANSAC一致性）；输出闭环约束与重定位先验/初值；维护检索数据库/索引资源
Backend（后端一致性）

负责长期一致性：管理约束、调度优化、写回地图
汇总前端里程计约束+闭环约束+GNSS/IMU等因子；构建/维护位姿图或局部BA窗口；选择何时做局部多帧BA/全局优化；调用求解器插件求解；将优化结果写回地图并发布全局校正/优化结果
Visualization & External IO（可视化/对外接口）
对外发布结果并支持交互控制（ROS/GUI）
ROS1/2 TF/Path/Odom/Map 发布；GUI可视化（可选）；订阅外部命令（reset/save/load/mode）；输出系统状态与性能指标；必要时提供服务接口
Global Context（全局上下文容器）
管理共享句柄与运行时能力，支持按需依赖注入
保存共享资源句柄（map、calib、loop_db、store等）与运行设施（bus、executor、clock、logger、metrics、config快照）；提供只读 ContextView；能力获取与启动时校验（Query/Write接口拆分）；避免一把全局大锁（锁应在资源内部）
模块架构细则
横向上，整个系统分为七个核心模块：跟踪（前端的完整里程计）、地图（地图表示、查询与新增）、位置（回环检测与重定位）、后端（长期一致性约束）、传感器IO（数据读取与预处理）和可视化（与ROS交互并可视化）、全局上下文（管理共享数据的读写）七大模块，模块在运行上基于主被动混合，保持作为一个纯粹逻辑单一的功能
纵向上，使用功能模块和算法插件两种概念来构建SLAM系统的具体功能，功能模块对应特定子任务，而算法插件则是该任务中的一部分的具体实现方法。例如，跟踪模块完成视觉里程计任务，不同算法插件可以实现不同跟踪方法（基于特征点的ORB-SLAM前端或基于直接法的DSO前端等）；又如地图模块提供地图数据结构，不同插件可实现不同形式的地图表示（点云地图、体素栅格、概率栅格等）。通过这种策略模式的设计，我们可以将算法与功能解耦，方便替换和扩展
另外需要其他的辅助模块：
- 基础设施：内存管理、日志、配置解析 (YAML)、插件注册表、事件总线。
- 执行层：构建器和执行器等，前者读取 YAML，把插件实例化并连接起来；后者分配线程，决定是串行跑（调试用）还是多线程流水线跑（实际运行）。
日志系统——spdlog
最终选用了spdlog日志方案，而不是Google Glog系统，spdlog是现代C++（C++11/14）标准的标杆。Header-only（易集成），速度极快（支持异步模式），语法类似Python的f-string。
PS：大概只是看到spdlog的Github Star数量远多于glog
而spdlog的核心架构策略为：
必须使用异步模式 (Async Mode)
- 原因：前端（Tracking）处理每一帧的时间通常只有几毫秒（例如30ms）。如果写入硬盘发生抖动耗时10ms，你的系统就会丢帧。异步模式会将日志扔到一个内存队列（Ring Buffer）里，由后台线程慢慢写盘。
多Sink分流 (Multi-sink)
- Console Sink：输出到终端，带颜色，只显示 INFO 及以上级别（给你看）。
- File Sink：输出到文件，包含 DEBUG 级别（给事后分析看）。
封装为单例 (Singleton Wrapper)
- 不要在每个cpp文件里初始化logger，做一个全局的工具类。
安装方法很简单
sudo apt install libspdlog-dev
或者
git clone https://github.com/gabime/spdlog.git
cd spdlog && mkdir build && cd build
cmake .. && cmake --build .
然后在Cmake中进行使用
# 寻找 spdlog
find_package(spdlog REQUIRED)

# 定义预编译宏，这决定了LOG_DEBUG等宏在编译时是否被剔除
# 在Release模式下，你可能想把TRACE去掉以提升性能
add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

add_executable(my_slam_system main.cpp)
target_link_libraries(my_slam_system PRIVATE spdlog::spdlog)
在 target_link_libraries 中，PRIVATE、PUBLIC 和 INTERFACE 决定了 “谁能看到这个库的头文件路径和链接库”。 PRIVATE 意味着 spdlog 只是你这个项目（my_slam_system）内部的实现细节。如果以后有别的程序引用了你的项目，它们不会自动获得 spdlog 的头文件路径或链接。
因为 my_slam_system 是一个 Executable (可执行程序)，而不是一个库。 通常没有其他东西会去链接一个可执行程序。处于依赖链的最末端，所以对于可执行程序来说，用 PRIVATE 是最规范、最安全的写法（虽然对于可执行程序，写 PUBLIC 通常也不会报错，但语义上不对）。
日志系统概览
日志系统的本质是一个 “分流管道”。你的代码产生数据，日志系统负责把数据异步地、安全地输送到不同的目的地。而其中的核心概念有
- Logger（日志器）：这是你代码直接交互的对象（即前端）。你告诉它：“我要记录这句话”。
- Sink（水槽/输出端）：这是数据的目的地（即后端）。
  - stdout_sink：输出到屏幕（控制台）。
  - file_sink：输出到 .txt 文件。
总体上的数据流向必须是异步的，否则写硬盘会卡死你的算法线程，具体流程大概是由SLAM线程写入到内存环形缓冲区也就是Ring Buffer，然后后台日志线程在空闲时读取到分发器Dispatcher中，分发器会根据设置将信息输出到终端和硬盘中；
在 spdlog 以及大多数现代日志系统中，日志级别通常分为 6 个等级。它们按严重程度从低到高排列。
级别
屏幕显示 (Console)
文件记录 (File)
编译版本 (Release)
作用
TRACE
❌ 关
❌ 关 (除非为了抓Bug特意开)
编译期剔除
查死锁、查数值溢出
DEBUG
❌ 关
✅ 开
保留
查逻辑、分析算法性能
INFO
✅ 开
✅ 开
保留
确认流程正常
WARN
✅ 开
✅ 开
保留
发现潜在风险
ERROR
✅ 开
✅ 开
保留
报警
CRITICAL
✅ 开
✅ 开
保留
崩溃前遗言
在 CMakeLists.txt 中，你可以通过宏定义，让编译器在 Release 模式下直接把 TRACE 和 DEBUG 代码删掉，从而实现 “零性能损耗”：
# 如果是 Release 模式，最低日志级别设为 INFO (0=TRACE, 1=DEBUG, 2=INFO...)
# 这样 LOG_DEBUG(...) 在编译后就是空行，代码中低于 INFO 级别的宏会被直接删除，不占 CPU
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO)
else()
    add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
endif()
日志管理设计
日志管理的想法很简单，每次运行时创建一个日志管理器对象，在特定位置下创建日志文件，使用时间戳等命名，然后不同的模块和插件会在输出日志时打上自己的标签，以便于区分不同的模块输出，最终的日志系统有三层组成：
1. 全局 LogManager（单例）
  - 负责初始化 sinks（终端/文件）、formatter（pattern）、async 线程池、缓存所有 logger
  - 保证：同一进程里，所有日志最终写到同一套 sinks（同一个文件/终端）
2. spdlog::logger（真正干活的对象）
  - 每个模块/插件对应一个 logger（名字编码为 Front 或 Front|ORB）
  - 多线程安全写入（使用 _mt sinks）
3. Log（你用的句柄）
  - 本质是一个 shared_ptr<spdlog::logger> 的封装，让你同时支持：
    - log.info("...")（值对象语义）
    - log->warn("...")（像指针一样用）
    - auto p = log.shared();（转回 shared_ptr）
一般在 main() 或 Runtime 初始化最早期调用：
SimpleSLAM::LogConfig cfg;
cfg.enable_console = true;        // 是否输出终端
cfg.enable_file    = true;        // 是否输出文件
cfg.file_path      = "logs/slam.log";
cfg.console_level  = spdlog::level::info;   // 终端只看 info+
cfg.file_level     = spdlog::level::trace;  // 文件写全量（trace/debug也写）
cfg.async = true;                 // 强烈建议：SLAM里避免IO卡算法
cfg.custom_console_colors = false;// 一般不用手配颜色，靠 %^%$ 即可
SimpleSLAM::LogManager::Instance().Init(cfg);
初始化后会发生什么？
- 创建一个或多个 sinks（终端 sink + 文件 sink）
- 给每个 sink 设置各自的 level 和 formatter
- 如果 async=true：创建 spdlog 的线程池，logger 变成 async logger
- LogManager 内部准备一个 map 缓存："Front"、"Front|ORB" 等对应 logger
注意：建议只 Init 一次（我们的 inited_ 已经保证重复调用会直接 return）。
然后可以自定义输出格式：
你配置里有：
cfg.console_pattern = "[%H:%M:%S.%e][%^%l%$][%t]%P %v";
cfg.file_pattern    = "[%Y-%m-%d %H:%M:%S.%e][%l]%P %v";
这里：
- %H:%M:%S.%e：时间（含毫秒）
- %l：日志级别（info/warn/error…）
- %^ 和 %$：仅对彩色 sink 生效，用于“开启/关闭颜色区域”
- %t：线程 id
- %v：日志正文
- %P：我们自定义的“标签前缀输出”
  - 会把 logger name（比如 "Front|ORB"）变成 "[Front][ORB]"
  - 并根据全局最大标签宽度自动补空格，实现对齐
在最终使用上

内存管理
内存管理的核心不是“如何申请内存”，而是如何安全地申请、共享数据和避免内存碎片。以及避免以下情况：
- 泄漏（malloc/new 了不释放）
- 重复释放（double free）
- 悬空指针（释放后还在用）
- 越界访问（写坏别人的内存）
- 无意义拷贝（性能被复制/分配拖垮）
- 碎片化（频繁分配释放导致性能抖动）
- 多线程竞争（共享对象生命周期不清晰）
首先是RAII与所有权的问题：SLAM系统中有大量复杂对象，Frame（帧）, MapPoint（地图点）, KeyFrame（关键帧）。
- 问题：后端优化线程正在使用一个关键帧，而前端线程可能因为这是一个“坏帧”想要删除它。如果使用裸指针（new/delete），必然导致 Segfault (段错误)。
现代C++的解决方法就是智能指针，应该严格遵循 RAII (资源获取即初始化) 原则，全面放弃 new/delete，改用智能指针。
std::unique_ptr (独占所有权)
- 场景：激光雷达的一帧原始点云数据（Raw Scan）。它只属于当前处理它的那个函数或类，处理完就丢弃，不需要共享。
- 优势：零开销，出作用域自动释放。
std::shared_ptr (共享所有权)
- 场景：SLAM中的绝大多数核心数据结构。
- 例子：一个 MapPoint 同时被多个 KeyFrame 观测到，也被 Map 持有。只有当所有观测它的帧都消失，且地图也剔除它时，它才真正被销毁。
- 注意：小心“循环引用”（例如 Frame 指向 MapPoint，MapPoint 又指回 Frame）。如果有这种需求，反向指针必须使用 std::weak_ptr。
Eigen 的内存对齐陷阱
作为一个算法工程师，你肯定会大量使用 Eigen 库。
- 问题：Eigen 为了利用CPU指令集加速（SSE/AVX），要求矩阵内存地址必须是16或32字节对齐的。
- 现象：如果你在 std::vector 中存放包含 Eigen 成员的类，或者使用 std::make_shared 创建包含定长 Eigen 矩阵的对象，程序会直接 Crash。
- 解决：
  1. 在类定义中添加宏：EIGEN_MAKE_ALIGNED_OPERATOR_NEW。
  2. 或者使用支持对齐的分配器（C++17已部分优化此问题，但为了稳健通常仍需注意）。

项目架构

目录职责概览
目录
定位/职责
应包含内容（示例）
不应包含内容
依赖建议（允许依赖）
include/simpleslam/interfaces/（推荐单独存在）
框架稳定接口层：定义模块/插件/资源的抽象接口与契约

IModule（生命周期、端口绑定）、IPlugin、IResource、能力接口如 IPointNearestQuery、IOccupancyQuery、ISerializer、（可选）IMap
任何具体实现；YAML/Builder；ROS
core, utils（尽量少）
include/simpleslam/core/ + src/core/
领域模型与核心数据定义：与具体算法解耦的 SLAM 基本类型，工厂方法与注册器
Timestamp/FrameId、Transform(SE3)、Pose/State、Frame/Keyframe（元信息+数据句柄）、Landmark、坐标系/单位定义、MeasurementHeader
YAML/Registry/Executor；模块逻辑；ROS；具体算法实现（ORB/ICP/BA）
utils（谨慎控制），Eigen/Sophus
include/simpleslam/runtime/ + src/runtime/
运行时系统能力：装配、调度、生命周期、通信、观测
ConfigLoader(YAML)、Registry/Factory、Builder/GraphBuilder、Executor/ThreadPool、MessageBus/EventBus、Lifecycle、Metrics/Tracing
具体 SLAM 算法；地图实现细节；ROS（除非专门桥接层）
interfaces, core, utils
modules/
系统级功能模块实现（可调度执行单元）
tracking/*、loop_closure/*、backend/*、sensor_io/*、visualization/*；模块内部编排调用插件；端口输入输出处理；模块状态机
Builder/Registry 的实现（避免反向依赖 runtime）；通用算法库实现（放 plugins）；共享资源存储细节（放 resources）
interfaces, core, utils, resources(接口), plugins(接口/实现)
plugins/
算法插件实现（模块内部可替换算法）
feature/orb、matcher/icp、solver/ceres、loop_retrieval/dbow、preintegration/imu、map_ops/*（可选）
线程调度/队列；ROS；跨模块逻辑
interfaces（可选）、core, utils，第三方算法库
resources/
共享资源与数据存储（被动对象 + 并发控制 + 能力接口实现）
map/*（Point/Voxel/Gaussian MapResource）、并发控制、索引结构、storage/*（保存/加载、region cache）、calibration/*、vocabulary/*
具体回环/跟踪流程；调度线程逻辑（放 runtime/modules）；ORB/ICP 等算法（放 plugins）
interfaces, core, utils
include/simpleslam/utils/ + src/utils/
通用工具与基础设施（小而通用）
logging、timer/profiler、Status/Error、断言与参数校验工具、文件系统小工具、并发小工具（如 SPSC queue 包装）
领域核心类型（Frame/Map 等）；Builder/模块逻辑；具体算法
标准库为主，可依赖第三方基础库

数学运算/几何相关：放置规则（建议写进架构文档）

类型
放置目录
例子
原因
领域核心几何类型（稳定 API）
core/geometry
SE3/Transform、Pose、Frame 中使用的位姿/坐标系、单位系统
属于领域模型的一部分，外部使用者也会依赖，必须稳定、可安装
通用数学工具（非领域特有）
utils/math（可选子目录）
插值、统计量、随机采样、数值稳定小工具、通用 clamp/normalize
不应污染 core 的公共 API；属于可复用通用工具
算法级数学实现（随算法变化）
plugins/*
ICP 的残差/雅可比、BA 的核函数、IMU 预积分计算细节
这是算法实现细节，应该随插件走，便于替换与单测
资源数据结构的数学/索引
resources/*
KD-tree/hash-grid 查询、体素索引、概率更新公式
这些与资源表示强绑定，避免散落在 utils
依赖关系红线（建议强制执行）

层级
允许依赖
禁止依赖（红线）
utils
（尽量无）标准库/基础三方
任何上层目录
core
utils、Eigen/Sophus
runtime/modules/plugins/resources
interfaces
core、utils
runtime、任何具体实现
plugins
interfaces（可选）、core、utils
runtime、modules
resources
interfaces、core、utils
runtime、modules
modules
interfaces、core、utils、plugins、resources
runtime（避免循环依赖；由 runtime 创建/调度模块）
runtime
interfaces、core、utils
具体模块/插件/资源实现头（通过注册+工厂解耦）
ros_bridge（如有）
modules/runtime（视设计）
把 ROS 头引入到 core/interfaces
暂时无法在飞书文档外展示此内容
可安装（installable）视角下的头文件布局建议

路径
含义
建议
include/simpleslam/...
对外公开 API（安装后供下游 #include <simpleslam/...>）
只放稳定接口与稳定数据结构头；避免重依赖（尤其 ROS）
src/...
内部实现
不安装；可以包含更多第三方、实现细节头
modules/ plugins/ resources/ 的 include/ 子目录
具体实现的公开头（如果需要）
初期可以不对外公开，只在内部使用；对外公开会增加 ABI 负担
模块设计分析
模块的构建分为两个阶段，第一阶段是实例化，这个阶段会通过字符串访问全局注册表中的Creator，并且将模块命名和模块参数传入，获取对象的unique_ptr指针；第二阶段则是使用类似的方法创建出该模块所依赖的插件并且返回其指针，然后将指针打包；第三阶段则是构建资源句柄，使用配置文件中的，送入模块的Init方法中实现初始化
构建器/工厂模式分析
设计概览
目标是支持以“搭积木”的方式组装 SLAM 系统：系统由多个模块（Module）与模块内插件（Plugin）组成，模块/插件之间通常是并行关系，基本不需要相互引用。系统的实例化由一份“配置表”驱动，框架需满足以下要求：
1. 按类型字符串创建对象：配置中给定 type（字符串索引）即可创建对应“积木”对象，也就是根据字符串将对应的类创建为实例。
2. 实例名与类型解耦：所创建的实例具备外部指定的 name作为模块/插件名，可与 type（类名）不一致，用于多实例区分与后续引用。
3. 配置源解耦：不强绑定 YAML，未来可切换 JSON/TOML/命令行/网络下发等；外部解析模块负责将配置转换为统一的数据结构，目前分为：字符串，布尔值，整数/浮点数，对象类型（Object）。
4. 可扩展：新增模块/插件无需修改 Builder 或工厂主体代码，只需定义类并完成注册。
5. Header-only 友好：允许模块/插件为 header-only；即便被多个编译单元 include/链接多次，也应确保“只注册一次”。
核心思想是将模块化对象创建的部分分为三部分：解析器、装配器（构建器，Builder）、工厂
外部解析器（Parser / Loader）
- 输入：YAML/JSON/TOML/命令行/网络配置等任意来源，将数据“整理成 Params map”
- 输出：模块与插件的规格描述，保证基本类型归一化（数值/布尔/字符串），包含：
  - type：字符串索引（默认与类名一致）
  - name：实例名（外部指定）
  - params：参数表（std::unordered_map<std::string, Node>）
  - plugins：插件列表（同结构）
Builder（装配器）：Builder 不关心配置来源与格式，仅负责：
1. 调用工厂创建模块实例：Create(type, name, params)
2. 创建模块下插件并注入模块
3. 完成系统级装配
工厂（Registry Factory）：负责根据字符串和参数创建实例
- 保存 type -> creator 映射
- 提供统一创建入口 Create(type, instance_name, params)
- 新类型通过宏注册进入工厂（自注册），实现开闭原则
参数模型
第一步就是构建一个相对通用的参数模型，使得准确的配置可以在三者之间顺利流通，最终选用的是std::unordered_map作为最终的统一参数格式，这种格式非常适合作为顶层的配置表，其在算法上为键值对，键是参数名，值是具体内容（可以是基本类型或者复杂类型）
主要是因为在SLAM中，不止有各种基础类型，还存在大量的复杂结构配置，如数组（四元数）和二维数组矩阵，甚至可以存在结构化对象，因此需要单独进行处理
使用的时候也非常简单，外部解析器整理成 unordered_map<string, Node>（即 SimpleSLAM::Params，也就是起了一个别名）
SimpleSLAM::Params p = SimpleSLAM::Obj({
  {"max_features", 800},
  {"q", SimpleSLAM::Node(SimpleSLAM::Arr({1.0, 0.0, 0.0, 0.0}))},              // 四元数
  {"K", SimpleSLAM::Node(SimpleSLAM::Arr({                                     // 3x3 矩阵
          SimpleSLAM::Node(SimpleSLAM::Arr({500.0, 0.0, 320.0})),
          SimpleSLAM::Node(SimpleSLAM::Arr({0.0, 500.0, 240.0})),
          SimpleSLAM::Node(SimpleSLAM::Arr({0.0, 0.0, 1.0}))
        }))},
});
也可以对逐个进行使用和赋值
std::unordered_map<std::string, SimpleSLAM::Node> params;
params["max_features"] = 800;
params["q"] = SimpleSLAM::Node(SimpleSLAM::Arr({1.0, 0.0, 0.0, 0.0})); // 四元数
总的来说，对于基础数据类型如数值/字符串/列表，与std::unordered_map标准用法一致，直接赋值即可，但是对于复合类型就需要进行组织，组织成 Node::Array（或 Node::Object）再赋值；通常用一个小的 helper（例如 Arr(...)）即可完成，不需要特别复杂，其中对于结构化对象有如下嵌套表示
// 将一个名为 "camera" 的配置对象写入 params（params 是 unordered_map<string, Node>）
// 这里 "camera" 对应的 Node 类型是 Object（键值对），用于表达相机相关的嵌套配置。
params["camera"] = SimpleSLAM::Node(SimpleSLAM::Obj({
  // 相机模型类型：字符串标量
  // 常见取值例如 "pinhole"（针孔模型）、"fisheye"（鱼眼）、"omni" 等（由你的框架约定）
  {"model", "pinhole"},

  // 相机内参：数组类型（Node::Array）
  // 约定为 [fx, fy, cx, cy]，分别为：
  // fx, fy：焦距（像素单位）；cx, cy：主点坐标（像素单位）
  {"intrinsics", SimpleSLAM::Node(SimpleSLAM::Arr({500.0, 500.0, 320.0, 240.0}))},

  // 畸变参数：数组类型（Node::Array）
  // 这里示例为 4 维畸变参数（具体含义取决于畸变模型约定）
  // 常见针孔畸变可用 [k1, k2, p1, p2]（径向 k1,k2 + 切向 p1,p2），也可能扩展更多参数
  {"distortion", SimpleSLAM::Node(SimpleSLAM::Arr({0.1, -0.05, 0.0, 0.0}))}
}));

工厂与注册机制
工厂的目标是：配置表里写一个 type 字符串，就能拿到对应的“积木类”并创建实例，同时 Builder 不需要知道具体类的名字、更不需要 if/else 分支。因此工厂提供两件事：
1. 注册（Register）：告诉系统，“当 type = 'Frontend' 时，用 Frontend 这个类来创建对象”。
2. 创建（Create）：运行时输入 type + instance_name + params，工厂返回一个 unique_ptr<Base>。
在具体实现操作上，工厂内部有一张注册表（哈希表）：
unordered_map<string, Creator> creators_;
- key：type（如 "Frontend"），实际上是字符串，充当索引
- value：Creator（如何构造 Frontend 的函数）
工厂逻辑是：
1. 在 creators_ 里查找 "Frontend"
2. 找到则取出 Creator
3. 调用 Creator(instance_name, params)，返回 unique_ptr<Base>
如果 "Frontend" 不存在，就抛 unknown type（典型原因：未注册或注册代码未进入最终链接产物）。
创建的前提是注册表中存在这个对象，因此工厂类中存在对应的注册函数，可以将新的模块和插件类注册进去，并且为了尽可能区分开不同模块与插件，对外使用了两套注册表：
- ModuleFactory = RegistryFactory<ModuleBase>
- PluginFactory = RegistryFactory<PluginBase>
这样模块和插件即便同名也不会冲突，因为它们存在不同的 registry 中。
注册机制部分相对复杂一点，因为我们希望提供一个简单的注册逻辑，可以在定义了一个新模块/插件之后直接注册，并且尽可能不出现冲突的情况，如果注册机制是静态函数：
static AutoRegister reg("Frontend", ...);
并且 reg 具有“每个编译单元一份”的语义，那么它会被执行多次，造成重复注册。
因此利用了C++17的inline变量的关键性质，保证了只注册一次
inline const bool reg_xxx = (执行注册逻辑, true);
并且跨多个编译单元只会在最终程序镜像里合并为同一个实体（ODR 合并），不会导致生成多个实体
同时为了进一步化简，使用注册宏对注册方法进行了封装，实际上有三种类型的注册宏，代表着不同的注册逻辑，这里以模块的注册宏为例进行解析，如果想注册插件模块，将注册宏中的MODULE改成PLUGIN即可
- SIMPLESLAM_REGISTER_MODULE(CLASS)
  - 用途：注册一个“模块类型”，用于 ModuleFactory 创建。
  - 注册 key（type）：#CLASS（类名字符串化），例如 Frontend → "Frontend"。
  - 适用条件：CLASS 必须是未限定标识符（不能写 ns::Frontend），也就是参数必须是一个纯类名，不能带任何命名空间或作用域限定符。
    - 原因在于宏内部需要“拼接”一个变量名来实现注册，如果你传入 slam::Frontend，拼接后会变成类似reg_module_slam::Frontend ，这不是合法的 C++ 变量名（包含 ::），因此编译会直接报错
    - 相当于将类名变成字符串（用预处理器的 字符串化运算符 # 变成字符串），然后作为注册索引加入工厂的注册表中
- SIMPLESLAM_REGISTER_MODULE_AS(TYPE_STR, QUALIFIED_CLASS, TAG)
  - 用途：注册模块，用于解决默认宏无法处理 ns::Class 的问题，允许：
    - 类在命名空间中（QUALIFIED_CLASS 可带 ::）
    - 或显式指定注册 key（TYPE_STR）
  - 参数含义：
    - TYPE_STR：注册 key（配置里写的 type 必须匹配它），相当于索引
    - QUALIFIED_CLASS：真实类型，例如 slam::Frontend
    - TAG：用于生成注册变量名的标识符（必须是合法标识符，不含 ::），其只是“变量名标签”，不影响注册 key，并且TAG也不能重复（最少在同一个编译单元内不能重复），或者直接建议不能重复、全局唯一
- SIMPLESLAM_REGISTER_MODULE_OVERRIDE(CLASS)
  - 用途：注册模块时采用“覆盖（override）策略”，即：
    - 如果同 key 已注册，则用新 creator 覆盖旧 creator（last-wins）。
  - 注册 key（type）：#CLASS。
  - 适用条件：同默认宏，CLASS 必须是未限定标识符。
下面给出使用方法，这里假设我新建了一个名为Frontend的模块，以及在命名空间中创建了一个插件ORBExtractor
class Frontend : public SimpleSLAM::ModuleBase { ... };
namespace feat {
class ORBExtractor : public SimpleSLAM::PluginBase { ... };
}

SIMPLESLAM_REGISTER_MODULE(Frontend); // 注册 type="Frontend"

// 希望配置里仍写 "ORBExtractor"，而不是 "feat::ORBExtractor"
SIMPLESLAM_REGISTER_PLUGIN_AS("ORBExtractor", feat::ORBExtractor, ORBExtractor);

SIMPLESLAM_REGISTER_MODULE_OVERRIDE(Frontend);

现在的注册依赖“某个 TU 的静态初始化执行”。如果未来你把插件/模块编译进 静态库，并且该 .o 没有被链接器拉进最终程序，就会出现：
- 代码编译通过，但运行时报 “unknown type”
这是最常见的注册表方案坑。
后续可以考虑如何修改和优化
创建函数
只需要提供 type 字符串 + instance_name + 已整理好的 Params(unordered_map)，函数直接调用对应 RegistryFactory<Base> 完成实例化；不对参数表做任何检查，仅在失败时给出更可定位的错误信息（例如 unknown type、构造异常等），对应的代码在runtime/Creator.h中
使用方法如下
SimpleSLAM::Params params;
params["max_features"] = 800;
params["enable_loop"]  = false;

auto m = SimpleSLAM::CreateByType<SimpleSLAM::ModuleBase>(
    "Frontend", "frontend0", std::move(params));

SimpleSLAM::Params pparams;
pparams["n_features"] = 1200;

auto p = SimpleSLAM::CreateByType<SimpleSLAM::PluginBase>(
    "ORBExtractor", "orb0", std::move(pparams));

资源模块分析
在SLAM中，有些数据满足下面的情况：
- 多模块共享（Tracking/Backend/Loop/Viz 都会用）
- 生命周期长（跨很多帧/分钟/需要存盘）
- 需要并发控制（读写锁/快照/版本）
- 需要持久化/区域加载
典型的数据有：状态信息、地图、回环数据、外参标定信息、关键帧数据，并且这些数据总体上分为两类
- 对象类型数据：一个数据就是单个对象，如地图、状态，无法继续细分
- 序列类型数据：一个数据实际上是一个序列，如关键帧序列，实际上是由一系列关键帧组成
每个 Resource 一定要拆成“能力接口”而不是暴露全能对象，具体来说，对每个资源至少拆成三类接口（这是后续“权限控制 + 并发”最省心的做法）：
- Query：只读查询（多模块共享）
- Write：写入/结构更新（尽量收敛到单写者模块）
- Snapshot：快照（给高频读/可视化，减少锁争用）
在具体使用上，约定如下
- ResourceHub：全局资源仓库（可全局单例语义，但建议由 Builder 持有并注入），负责 key -> 资源实例 的注册与查询。
- 模块只依赖能力接口（capability）：模块不识别具体资源类型，只声明自己需要哪些接口能力。
- Builder 装配期校验：读取 YAML，创建资源、创建模块、按 deps 绑定 slot→key，并校验资源是否实现所需接口；不满足则启动失败并报清晰错误。
- 字符串 key 只出现在装配期：运行期模块持有的是接口句柄（如 std::shared_ptr<IStateRead>），不再做字符串查找。
资源的最薄元信息建议统一成一个接口（或等价能力）：name()（资源 key）、kind()（五类枚举）、type_name()（稳定实现类型标识）。可选能力 IVersioned 仅用于诊断与调试（版本号/时间戳）。
资源分类
StaticConfig（静态配置类）
设计思想：管理“启动期写一次、运行期只读”的全局配置与标定参数，作为系统的常量来源，保证可复现与稳定。
典型内容：
- 相机/IMU/LiDAR 内外参、畸变模型、时间偏移
- 噪声参数、固定超参（阈值上限、鲁棒核等）
- 词袋字典/模型路径、资源路径、传感器模型描述
能力接口（模块依赖）：
- IConfigView：只读访问（标定/噪声/模型路径等）
- 可选 IConfigUpdate：在线标定或热更新（仅当你确实需要“运行期变配置”）
装配与实现要点：
- YAML 中声明 kind: StaticConfig 与 type（具体实现类），Builder 解析 params 构造后注册到 ResourceHub。
- 模块通过 slot 依赖注入 IConfigView，不关心具体实现类与资源 key 的字符串细节。

---
RuntimeState（运行期单例状态类）
设计思想：表达“此刻的最新估计与系统状态机”，体量小但更新频繁，作为跨模块协同的共享当前态；只放“当前值”，不放历史。
典型内容：
- 当前位姿/速度/IMU bias/重力方向（VIO/LIO）
- tracking 状态机（未初始化/跟踪/丢失/重定位中）
- active map id、最近关键帧 id、模式开关（暂停后端、开启回环等）
能力接口（模块依赖）：
- IStateRead：读取当前状态（Get() 或 Snapshot()）
- IStateWrite：更新状态（Update(new_state)）
- 可选 UpdateIfNewer(ts/version)：防止乱序覆盖（接口仅表达语义）
装配与实现要点：
- Builder 装配时可对不同模块注入不同权限（只读/可写），例如可视化只拿 IStateRead，前端拿 IStateWrite。
- YAML 中通常只描述 RuntimeState 的实现类型与初始参数（初始模式、阈值等）。

---
MapStore（空间场地图本体类）
设计思想：管理以空间组织为核心的地图本体（field/grid/volume/dense structure），强调局部空间裁剪与增量更新；它是“稠密/空间场”层面的权威数据。
类型边界（关键约定）：
- MapStore 专指空间场/稠密表示：体素/TSDF/高斯/占据栅格/稠密点云分块等。
- Keyframe/Landmark 等“按 ID 管理的离散实体”不属于 MapStore，归入 Graph/HistoryStore。
典型内容：
- Voxel/Hash-Voxel Map、Occupancy Grid、Octree
- TSDF/ESDF 体积场
- Gaussian Map（3D Gaussian Splatting 类型）
- 稠密点云（按空间分块/体素 bucket 组织）
能力接口（模块依赖）：
- IMapQuery：空间裁剪/局部区域提取（AABB/radius/frustum 等概念上的局部视图）
- IMapWrite：融合/更新入口（插入、融合观测、更新子块）
- IMapSnapshot：导出用于可视化/外部接口的地图视图
- 可选 ISubmapManage：子地图分区管理（切换、加载/卸载、active submap）
装配与实现要点：
- 建议将“通用检索/候选生成/近邻检索”更多交给 IndexStore（除非你明确 MapStore 自带同类能力且能解释清楚两者关系）。
- YAML 中声明 kind: MapStore 与具体 type（VoxelMap/TsdfMap/GaussianMap…），Builder 注册；后端/建图模块注入 IMapWrite，可视化注入 IMapSnapshot。

---
Graph/HistoryStore（稀疏实体 + 关系 + 历史类）
设计思想：作为稀疏 SLAM 的权威数据本体，统一承载按 ID 管理的实体、实体关系/约束，以及随时间追加的历史序列，关键词是 by-id + append + range。
典型内容：
- 实体：Keyframe、Landmark/MapPoint、Submap（可选）
- 关系/约束：共视图、生成树、位姿图/因子图、回环约束集合
- 历史：轨迹序列、关键帧创建序列、回环事件时间线
能力接口（模块依赖）：
- IEntityQuery / IEntityWrite：按 ID 获取、增删实体（Keyframe/Landmark/Submap）
- IGraphQuery / IGraphWrite：边/因子增删与邻接查询（PoseGraph/FactorGraph/Covisibility）
- ITrajectoryQuery / IHistoryAppend：时间序列读取与追加（Trajectory/Timeline/EventLog）
- 可选 IWriteBatch：批量写入/事务式提交语义（一次插入关键帧+路标+边等）
装配与实现要点：
- 你可以先用一个资源实现把实体/关系/历史都塞在同一个 Graph/HistoryStore 内；未来若拆成多个资源 key（EntityStore、GraphStore、HistoryStore）也不影响模块接口，只是 YAML 绑定不同 key。
- 前端通常依赖实体写入 + 约束写入 + 历史追加；后端依赖实体/图的读写；回环依赖实体查询 + 图写入。

---
IndexStore（可重建索引类）
设计思想：管理用于检索/候选生成/近邻查询的加速结构，可由本体数据推导重建，不是权威数据源，允许弱一致与重建。
典型内容：
- BoW 倒排索引（DBoW2 类）
- ScanContext 库
- 全局描述子/embedding 的检索结构（向量索引）
- 可选：独立空间近邻索引层（如果你决定把 NN 从 MapStore 剥离）
能力接口（模块依赖）：
- IIndexQuery：检索、候选生成、top-k 查询
- IIndexUpdate：增量更新（插入/删除条目）
- IIndexRebuild：从 Map/Graph 重建或 reset
装配与实现要点：
- YAML 中可用 depends_on 声明索引依赖的本体资源（通常是 Graph/HistoryStore 或 MapStore），Builder 据此排序初始化或在资源内部完成“从本体构建”的参数绑定。
- 回环模块依赖 IIndexQuery（可选 IIndexUpdate），并通过 Graph/HistoryStore 获取必要的关键帧/描述子等本体信息。
配置文件设计
接下来就是设计对应的yaml文件实现，以此方便实现模块化的构建，第一步的思想就是将资源部分显式列出，定义其中存在哪些资源，并且给出资源的类型和参数信息，第二步就是在模块的定义中指出其所依赖的资源
- Resources：资源块清单（key=资源名），每个资源包含 kind/type/params。
- Modules：模块清单（key=模块槽位名），每个模块包含 id/type/params/deps/plugins。
- Runtime：全局运行时配置（日志、线程池、TopicBus 等）。
下面给出一个简单的YAML文件实现
Resources:
  Map:
    Kind: MapStore
    Type: VoxelMap
    Params: { resolution: 0.2 }
  State:
    Kind: RuntimeState
    Type: VioState
    Params: { }

Modules:
  Frontend:
    Name: LIOFrontend
    Type: LIOFrontend
    Params: { }
    Depends:
      state: State
      map: Map
    Plugins:
      Extractor: { Type: ORBExtractor, Params: { n_features: 1200 } }
      Matcher:
        Type: SuperPoint
        Params: { }

Runtime:
  TopicBus: { CallbackThreads: 4 }
  Logging: { Level: info, Async: true }

首先在 Resources 中注册了 Map 与 State 两个资源，并为每个资源指定了 Kind/Type/Params。其次在 Modules 中定义了 Frontend 模块，模块的 Type 与实例 ID 都是 LIOFrontend，用于明确该前端属于 LIO 类型，并指定了 Params（此处为空）。随后是 Depends 映射：例如 state: State，其中 state 是模块内部的槽位名，State 是资源清单中的 key；装配时先用槽位名在模块内取句柄，再通过该映射定位到具体资源。最后是 Plugins 段，逻辑与 Depends 类似：插件槽位名映射到具体插件配置，实现模块内部按槽位获取插件实例。
数据流分析
传感器的数据是即时产生的、类型各异的，并且需要在不同的模块之间跨线程通信，IMU等高频数据的频率普遍在200Hz甚至更高，点云等数据的频率只有10Hz，同时现需要类型安全和低延时，因此需要一种不依赖ROS的实现，因为ROS传递消息的机制存在序列化/反序列化的情况，会导致信息的拖慢，这一点尤其是在面对大型数据结构时尤为明显，因此我们选用了eventpp作为通讯机制，其有如下优点
- 本身就是为 callbacks / event dispatcher / event queue 设计的库，可直接实现 signal-slot、pub-sub、observer 等模式
- 并且明确支持 多线程安全、嵌套事件、同步分发与异步队列
- header-only、只要求 C++11、无额外依赖，无需编译安装，并且在 vcpkg/conan 等包管理器里可用，工程集成成本很低
- 社区使用量较高
不过该机制更适合同进程内跨线程通信和事件分发，然后我们再使用mutex实现无锁缓冲队列，二者进行组合
标准库的 std::vector / std::deque 在并发场景下不提供“一个线程 push、另一个线程 pop”这样的安全保证。因此在这种情况下，如果没有同步机制，结果是数据竞争（未定义行为），可能表现为偶发崩溃、读到脏数据、迭代器失效、内部状态损坏等，因为直觉上虽然这种操作并没有操作同一个元素，但是对 deque/vector 的内部实现来说，push/pop 会修改共享的内部元数据（例如大小 size、起始/结束索引、块指针数组、迭代器边界等）。这些元数据是共享的；两个线程同时写它们，或者一个写一个读，都属于数据竞争。尤其是vector，push_back 可能触发扩容，导致整个底层数组搬迁，deque可能在 push 时分配新的块、更新映射表指针等
理想的一种通讯方式实际上是类似于ROS的通讯，只需要给出话题和数据类型，同时设置好回调函数类型，即可成功实现类似ROS的通讯，并且因为更加轻量化的原因，可以避免序列化/反序列化的通信延迟，但是想实现这种方法本质上需要一个进程内的“中心 Broker（消息总线/注册表）”，没有中心 Broker，就一定需要显式把发布者对象传给订阅者（你说的“中间对接”）；想要“话题一致即可通信”，就必须让双方都去同一个地方登记/查找
想使用Eventpp的话需要进行编译安装
git clone https://github.com/wqking/eventpp.git
cd eventpp
mkdir build
cd build
cmake ..
sudo make install
整体思想上与ROS非常一致，单个话题依赖于特定消息格式，并且一个发布对应多个订阅（至于多个发布的情况暂时未考虑），这个机制暂时命名为TopicBus，其把 topic 字符串 视作事件 ID（EventType），并且绑定消息类型，实现话题与消息类型的uiqi，避免错误
通讯机制
TopicBus 由一个进程内唯一 Runtime 管理（全局单例，似乎ROS1也是类似的机制，也就是主节点）。消息流分为三段：
- 发布阶段（Publish）：Publisher<T>::Publish(msg) 将消息封装为 _TBEnvelope 并入队到 TopicBus 内部队列。
 此阶段目标：尽快返回、绝不阻塞发布线程。
- 泵线程（Pump Thread）出队与分发（Dispatch）：Runtime 内部有一个泵线程：持续从内部队列取出消息，并调用 eventpp 的 EventDispatcher::dispatch(topic, env) 将消息同步分发给该 topic 的 listeners。
- 回调执行阶段（默认异步通知式）：每个 listener 收到消息后，默认不在泵线程里做重活，而是把回调任务投递到线程池执行，无论。这样泵线程保持轻量，避免被任意订阅者拖慢。
总体上，TopicBus 在进程内维护一个全局唯一 Runtime。各模块在任意位置创建 Publisher/Subscriber（通常作为类成员以保证生命周期）。发布时，Publisher 仅将消息以共享指针形式压入 TopicBus 的全局队列（若队列满则丢弃并返回 false），不阻塞发布线程。Runtime 的泵线程持续从队列取出消息并按 topic 分发给已注册的订阅者 listener。默认情况下（async_callback=true），listener 只负责做类型检查与限流，并将“执行用户回调”的任务投递到线程池后立即返回；用户回调的实际执行和完成情况独立于分发流程，不会反向阻塞发布线程或泵线程。
上述过程是异步处理过程，也可以设置为同步处理，也就是在订阅者创建时显示声明async_callback = false，监听和回调的过程就会变成 listener 在泵线程直接执行用户回调，此时：
- 泵线程会被回调耗时阻塞；
- 分发会被拖慢。
在具体使用方法上，因为是单进程场景，因此只有一个主函数，所以需要在主函数中执行
SimpleSLAM::TopicBus::Init({ .callback_threads = 4 });
SimpleSLAM::TopicBus::Shutdown()
这是初始化与关闭的函数，初始化中可以指定回调线程池线程数，关闭函数会关闭泵线程与线程池，建议在进程退出前调用，当然实现也支持懒初始化（首次 Advertise/Subscribe 时自动创建 Runtime），但工程上更推荐显式 Init，便于控制线程池规模与生命周期。
auto pub = SimpleSLAM::TopicBus::Advertise<MsgType>("/topic", topic_queue_limit);
pub.Publish(std::make_shared<MsgType>(...));
对发布者，有
- Advertise<T>(topic, topic_queue_limit=200)：创建发布者句柄。topic_queue_limit 为入口队列上限
- Publish(...) -> bool：非阻塞；队列满则返回 false（表示消息被丢弃）。
对于订阅者，分为两类接口：Subscribe 与 SubscribeRef
Subscribe<T>：回调参数为 std::shared_ptr<const T>这是推荐的默认形式：