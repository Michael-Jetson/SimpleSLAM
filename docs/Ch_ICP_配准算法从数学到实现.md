# ICP 配准算法：从数学原理到 C++ 实现

> **章节定位**：本章属于"算法实现"教学，衔接 Ch11（Eigen 基础）、Ch23（李群 manif）、Ch16（Concepts），并为 Mini-LIO 累积项目提供配准核心。
> **前置知识**：矩阵运算（Ch11）、李群 SE(3)/SO(3) 基础（Ch23）、SVD 分解（Ch11.4）
> **累积项目进度**：本章完成后，Mini-LIO 将拥有完整的 ICP 配准模块，可在 KITTI 数据集上输出里程计轨迹。

---

## 前置自测

📋 **答不出 ≥ 2 题 → 先回对应章节复习**

1. SVD 分解 $A = U\Sigma V^T$ 中，$U$、$\Sigma$、$V$ 分别是什么矩阵？$U$ 和 $V$ 有什么性质？（Ch11）
2. $\text{SE}(3)$ 的切空间 $\mathfrak{se}(3)$ 是几维的？指数映射 $\text{Exp}: \mathbb{R}^6 \to \text{SE}(3)$ 做了什么？（Ch23）
3. 给定一个 $3 \times 3$ 反对称矩阵 $[\omega]_\times$，$[\omega]_\times v$ 等价于什么向量运算？（Ch23）
4. `Eigen::JacobiSVD` 的 `ComputeFullU | ComputeFullV` 标志是什么意思？不加会怎样？（Ch11）
5. manif 库中 `SE3d::rplus(tangent)` 和 `SE3d::lplus(tangent)` 的区别是什么？（Ch23）

---

## 本章目标

学完本章后，你将能够：

1. **从数学上完整推导** SVD 闭式解和 Gauss-Newton 迭代两种 ICP 求解方法
2. **理解**点到点、点到面两种配准范式的数学差异与工程权衡
3. **独立实现** `VoxelHashTarget`（体素哈希地图 + 配准）和 `IcpSolver`（Gauss-Newton 求解器）
4. **在 KITTI 数据集上运行** `LoIcpOdometry`，输出里程计轨迹并用 EVO 评测

---

## 1. 配准问题的本质 ⭐

### 1.1 动机：每一帧点云都是"孤岛"

LiDAR 传感器每 100ms 扫描一帧点云，每帧包含数万个三维点。但每帧点云都在**传感器当前坐标系**下——如果不知道传感器两帧之间移动了多少，这些点云就是散落的孤岛，无法拼成完整地图。

**配准（Registration）**就是回答这个问题：**给定两帧部分重叠的点云，求连接它们的刚体变换 $T \in \text{SE}(3)$。**

```
帧 k-1 的点云 Q（目标）        帧 k 的点云 P（源）
    ·  ·                           ·  ·
   ·    ·        T = ?            ·    ·
  ·      ·   ←————————————       ·      ·
   ·    ·                         ·    ·
    ·  ·                           ·  ·
```

在里程计中，P 是当前帧，Q 是局部地图（由历史帧累积而成）。求出 T 后，当前帧的全局位姿就是 $T_{\text{world\_body}} = T_{\text{world\_map}} \cdot T$。

### 1.2 数学定义

给定源点集 $\{p_i\}_{i=1}^N$（当前帧）和目标点集 $\{q_j\}$（局部地图），求刚体变换 $T = (R, t)$，$R \in \text{SO}(3)$，$t \in \mathbb{R}^3$，使得：

$$
\min_{R, t} \sum_{i=1}^{N} \| q_{c(i)} - (R p_i + t) \|^2
$$

其中 $c(i)$ 是源点 $p_i$ 在目标点集中的**对应点索引**（correspondence）。

| 符号 | 含义 | 维度 |
|------|------|------|
| $p_i$ | 源点（当前帧，传感器坐标系） | $\mathbb{R}^3$ |
| $q_{c(i)}$ | $p_i$ 的对应目标点 | $\mathbb{R}^3$ |
| $R$ | 旋转矩阵 | $3 \times 3$，$R^T R = I$，$\det(R) = 1$ |
| $t$ | 平移向量 | $\mathbb{R}^3$ |
| $N$ | 有效匹配对数 | 标量 |

### 1.3 如果不迭代会怎样

**一次性求解的前提是已知正确的对应关系 $c(i)$。** 但在实际场景中，我们不知道哪个源点对应哪个目标点——只能用"最近点"作为近似。

问题是：最近点取决于当前位姿估计。位姿估计又取决于对应关系。这是一个**鸡生蛋**的循环依赖。

ICP 的解法是**交替优化**：

```
重复直到收敛：
  Step 1: 固定 T，找每个 p_i 变换后在 Q 中的最近点 → 对应关系 c(i)
  Step 2: 固定 c(i)，求最优 T → 位姿更新
```

这就是 **Iterative Closest Point (ICP)** 的核心循环，由 Besl & McKay 在 1992 年提出。

### 1.4 ICP 的历史脉络

| 年份 | 里程碑 | 贡献 |
|------|--------|------|
| 1987 | Arun, Huang & Blostein | SVD 闭式解求最优刚体变换（给定已知对应） |
| 1991 | Chen & Medioni | 点到面 ICP——利用表面法向量加速收敛 |
| 1992 | Besl & McKay | 经典 ICP 框架——迭代最近点 + 四元数求解（后人常改用 SVD） |
| 2001 | Rusinkiewicz & Levoy | ICP 变体六维度分类法（选点/对应/权重/拒绝/误差/求解） |
| 2009 | Segal et al. | Generalized-ICP——统一点到点和点到面 |
| 2019 | Rusinkiewicz | Symmetric ICP——对称目标函数，二次收敛 |
| 2023 | Vizzo et al. (KISS-ICP) | "为点到点 ICP 正名"——工程优化后的 p2p 达到 SOTA |

### 1.5 ⚠️ 常见陷阱

**💡 概念误区：认为 ICP 能全局收敛**

新手想法："ICP 会迭代直到找到正确位姿，所以只要跑足够多次就行。"

实际上：ICP 是一个**局部优化器**，只能收敛到**距初始猜测最近的局部极小值**。如果初始位姿偏差超过扫描空间范围的 10-20%，ICP 几乎必然收敛到错误的局部最小值。

根本原因：最近点对应关系在真实位姿的远处是完全错误的——大量源点匹配到了错误的目标点，形成一个"看起来合理但实际错误"的能量极小值。

正确做法：
- 在里程计中使用**恒速模型**提供良好的初始猜测（两帧间运动通常很小）
- 在回环检测中使用全局描述子（ScanContext、FPFH）先做粗匹配
- 在初始化时使用 IMU 预积分提供旋转先验

**🧠 思维陷阱：迭代次数越多精度越高**

新手想法："把 `max_iterations` 设成 1000 肯定比 20 更精确。"

实际上：ICP 通常在 5-20 次迭代内就收敛了（$\|\delta\xi\| < 10^{-4}$）。之后的迭代只是在数值精度的噪声中来回跳动，不增加精度，反而浪费算力。KISS-ICP 的默认上限是 500，但实测平均只用 3-5 次。

正确做法：用**收敛判据**（更新量范数小于阈值）控制终止，而非固定次数。

### 1.6 练习

1. **概念题**：解释为什么里程计中 ICP 的初始猜测通常足够好（提示：考虑 LiDAR 的帧率 10Hz 和车辆的最大速度）。如果车速 30 m/s，两帧间最大位移是多少米？与 100m 感知范围相比是什么量级？

2. **编程方向**：给定两组已知对应的 3D 点 $(p_i, q_i)$，$i = 1..10$，手动构造 $R = R_z(5°)$，$t = (0.1, 0.2, 0.3)^T$，令 $q_i = R p_i + t + \epsilon_i$（$\epsilon_i$ 为小噪声）。验证直接用最小二乘求解 $T$ 能否还原出 $R$ 和 $t$。

3. **思考题**：如果场景中有大量动态物体（行人、车辆），ICP 会面临什么问题？提出至少两种应对策略。

---

ICP 的框架清楚了：交替进行"找最近点"和"求最优变换"。下面先解决第一个子问题——如何高效地"找最近点"。

## 2. 最近邻搜索策略 ⭐⭐

### 2.1 动机：对应关系搜索是 ICP 的瓶颈

ICP 每次迭代需要为每个源点 $p_i$（经当前位姿变换后）在目标点集中找到最近邻。如果当前帧有 $N = 50000$ 个点，地图有 $M = 500000$ 个点：

- **暴力搜索**：$O(N \times M)$ 次距离计算 = $2.5 \times 10^{10}$。按每次距离计算 10ns，需要 250 秒。每帧 100ms 的实时约束完全无法满足。

因此最近邻搜索的加速结构是 ICP 能否实时运行的**决定性因素**。

### 2.2 哈希表基础——从"以空间换时间"到具体实现 ⭐

你已经知道哈希表是"以空间换时间"的方案。现在我们把这句话拆开，搞清楚**换了什么空间**、**换来了什么时间**、**怎么换的**。

#### 2.2.1 数组查找的本质

最快的查找是**数组直接索引**——给定编号 $i$，`arr[i]` 一步到位，$O(1)$。

```
索引：  0    1    2    3    4    5    6    7
数组： [__] [__] [__] [42] [__] [__] [__] [__]

查找 key=3 → arr[3] = 42 → 一步完成
```

**问题**：如果 key 的范围很大（如 0 到 $10^9$），你不可能开一个 $10^9$ 大小的数组——这就是"空间"的代价。

#### 2.2.2 哈希函数——把大范围映射到小范围

**核心思想**：用一个函数 $h(key)$ 把任意 key 映射到有限范围 $[0, B)$（$B$ 是桶数/数组大小）。

```
key = 73856093  →  h(key) = 73856093 % 1024 = 285
                   ↓
  桶 285 存储 value
```

| 概念 | 含义 |
|------|------|
| 桶（bucket） | 数组的一个位置 |
| 哈希函数 $h(key)$ | 把 key 映射到桶编号的函数 |
| 负载因子 $\alpha = n/B$ | 已存元素数 / 桶总数 |

**"以空间换时间"的精确含义**：开一个大小为 $B$ 的数组（空间），通过 $h(key)$ 一步定位到桶（时间 $O(1)$）。$B$ 越大，$\alpha$ 越小，冲突越少，越快。$B$ 越小，省空间但冲突多，变慢。

#### 2.2.3 冲突处理——两种主流策略

不同的 key 可能映射到同一个桶——这叫**冲突（collision）**。例如 $h(100) = h(1124) = 285$。

**策略 A：链地址法（Chaining）**——`std::unordered_map` 的做法

```
桶 283: → [empty]
桶 284: → [empty]
桶 285: → [key=100, val=A] → [key=1124, val=B] → null    ← 链表
桶 286: → [empty]
```

每个桶是一个链表（或 `vector`）。冲突时在链表尾部追加。查找时先算 $h(key)$ 定位桶，再在链表中线性查找 key。

**优点**：简单，$\alpha > 1$ 也能工作。
**缺点**：链表节点散布在堆上，**缓存不友好**——每次跳转链表指针都可能触发 cache miss。

**策略 B：开放寻址法（Open Addressing）**——`tsl::robin_map` 的做法

```
桶 283: [empty]
桶 284: [empty]
桶 285: [key=100, val=A]   ← 首选位置
桶 286: [key=1124, val=B]  ← 100 占了 285，1124 往后挪到 286
桶 287: [empty]
```

所有元素直接存在数组里（不用链表）。冲突时向后**探测**下一个空位。查找时从 $h(key)$ 开始逐个检查，直到找到 key 或遇到空位。

**优点**：数据连续存储在数组中，**缓存极其友好**——CPU 一次缓存行（64 字节）能预取多个桶。
**缺点**：$\alpha$ 不能超过 1（数组满了就不能再插入），通常维持 $\alpha < 0.7$。

> **Robin Hood 哈希**是开放寻址的改进版——当新元素的"探测距离"大于已占位元素时，交换两者位置。效果是让所有元素的探测距离趋于均匀，减少最坏情况。`tsl::robin_map` 就是这种实现。

**KISS-ICP 选择开放寻址（`tsl::robin_map`）而非链地址（`std::unordered_map`）**，原因就是缓存性能——ICP 每帧要做 50000+ 次哈希查找，缓存命中率直接决定运行速度。

#### 2.2.4 时间复杂度分析

| 操作 | 平均 | 最坏 | 条件 |
|------|------|------|------|
| 插入 | $O(1)$ | $O(n)$ | 所有 key 冲突到同一桶 |
| 查找 | $O(1)$ | $O(n)$ | 同上 |
| 删除 | $O(1)$ | $O(n)$ | 同上 |

**"均摊 $O(1)$"的前提**：哈希函数要好（分布均匀）、$\alpha$ 不能太高。好的哈希函数 + $\alpha < 0.7$ 时，平均探测次数 $< 2$。

#### 2.2.5 一个完整的具体例子

假设要存储 4 个整数 key：10, 22, 31, 4。桶数 $B = 8$，哈希函数 $h(k) = k \% 8$。

```
h(10) = 10 % 8 = 2
h(22) = 22 % 8 = 6
h(31) = 31 % 8 = 7
h(4)  =  4 % 8 = 4

桶：  [0]  [1]  [2]   [3]  [4]  [5]  [6]   [7]
      [ ]  [ ]  [10]  [ ]  [4]  [ ]  [22]  [31]

查找 key=22：h(22)=6 → 桶6 → 找到 ✅ （1 步）
查找 key=99：h(99)=3 → 桶3 → 空 → 不存在 ✅ （1 步）
```

无冲突时完美 $O(1)$。现在插入 key=18：$h(18) = 18 \% 8 = 2$，桶 2 已被 10 占据——冲突！

链地址法：桶 2 的链表追加 18：`[10] → [18]`。
开放寻址：往后探测，桶 3 空，放入桶 3。

### 2.3 从一维哈希到三维空间哈希——概念跃迁 ⭐⭐

#### 2.3.1 关键洞察：三维空间天然可以离散化

普通哈希表的 key 是任意类型（字符串、整数）。**空间哈希的 key 是三维整数坐标 $(v_x, v_y, v_z)$**——通过将连续的三维空间离散化为体素网格得到。

```
连续空间中的点 (3.7, 1.2, -0.5)
  ↓ 除以体素边长 s=1.0 并取 floor
体素坐标 (3, 1, -1)
  ↓ 哈希函数
桶编号 h(3, 1, -1) = 某个整数
  ↓ 查表
该体素内存储的所有点
```

**和普通哈希表的关键区别**：

| 维度 | 普通哈希表 | 空间哈希表 |
|------|-----------|-----------|
| key 的类型 | 字符串/整数（无几何意义） | 三维整数坐标（有几何意义） |
| key 之间的关系 | 无序（"apple" 和 "banana" 无关） | **有序**（$(3,1,-1)$ 和 $(3,1,0)$ 是邻居） |
| value 的含义 | 任意数据 | 该体素内的点集合 |
| 查询模式 | 精确匹配（key 存在吗？） | **邻域搜索**（附近有什么？） |

**这个区别至关重要**：普通哈希表只做精确查找——"key=X 的值是什么？"。空间哈希表做**邻域查找**——"距离 $(3.7, 1.2, -0.5)$ 最近的点是哪个？"。后者需要检查 key 及其**邻居**的桶。

#### 2.3.2 为什么不用三维数组？

最朴素的做法是开一个 3D 数组 `grid[vx][vy][vz]`。

**问题**：LiDAR 的感知范围是 100m，如果 $s = 1.0\text{m}$，需要 $200 \times 200 \times 200 = 8 \times 10^6$ 个体素。但实际上**大部分体素是空的**——一帧点云只覆盖其中几千个体素（点云是稀疏的，不是每个立方米都有点）。

| 方案 | 内存 | 查询速度 | 适用条件 |
|------|------|---------|---------|
| 3D 数组 | $O(V^3)$，大量浪费 | $O(1)$ 直接索引 | 场景小、点云稠密 |
| 哈希表 | $O(n)$，只存非空体素 | $O(1)$ 均摊 | 场景大、点云稀疏 |

**空间哈希 = 稀疏 3D 数组**。只有包含点的体素才在哈希表中分配内存。这就是"空间换时间"中"空间"的精确含义——不是浪费空间，而是**只为有用的体素分配空间**，用哈希函数跳过空体素。

#### 2.3.3 三维哈希函数为什么用三个大素数

将三维整数坐标 $(v_x, v_y, v_z)$ 映射为一个标量桶编号。需要满足：

1. **分散性**：相邻体素（如 $(3,1,0)$ 和 $(3,1,1)$）的哈希值应该差异大，避免聚集冲突
2. **快速**：只用整数乘法和异或，不用浮点数

三素数方案（Teschner et al. 2003）：

$$
h(v_x, v_y, v_z) = (v_x \cdot 73856093) \oplus (v_y \cdot 19349669) \oplus (v_z \cdot 83492791)
$$

**为什么用素数？** 乘以素数相当于在整数域上做"搅拌"——素数的特殊性质使得不同的 $(v_x, v_y, v_z)$ 组合不太可能产生相同的哈希值。

**为什么用异或 $\oplus$ 而非加法 $+$？** 异或是按位独立操作——每一位的输出只取决于该位的两个输入，不会像加法那样产生进位传播。这使得哈希值的每一位都充分利用了三个坐标的信息。

**具体例子**——相邻体素的哈希值：

```
h(3, 1, 0)  = (3×73856093) ^ (1×19349669) ^ (0×83492791)
            = 221568279 ^ 19349669 ^ 0
            = 205765538

h(3, 1, 1)  = 221568279 ^ 19349669 ^ 83492791
            = 154869251

差异巨大！两个相邻体素映射到完全不同的桶位置——避免了聚集冲突。
```

**C++ 实现**：

```cpp
struct VoxelHash {
    size_t operator()(const Eigen::Vector3i& v) const {
        return static_cast<size_t>(
            (v.x() * 73856093) ^ (v.y() * 19349669) ^ (v.z() * 83492791));
    }
};
```

> **注意**：上面的三素数方案用于教学示例。SimpleSLAM 实际代码（`voxel_grid.hpp`）使用 FNV-1a 64-bit 哈希——逐字节混合每个坐标，雪崩特性更好。两者功能等价，只是碰撞率和分布特性有差异。实际项目中推荐使用已有的 `VoxelHash`（FNV-1a）而非自己实现三素数版本。

#### 2.3.4 从连续坐标到体素坐标

将一个三维浮点坐标 $(x, y, z)$ 转换为体素坐标 $(v_x, v_y, v_z)$：

$$
v_x = \lfloor x / s \rfloor, \quad v_y = \lfloor y / s \rfloor, \quad v_z = \lfloor z / s \rfloor
$$

其中 $s$ 是体素边长。$\lfloor \cdot \rfloor$ 是向负无穷取整（floor）。

**几何含义**：将三维空间切割成边长为 $s$ 的立方体网格，每个立方体有唯一的整数坐标。

```
s = 1.0m 时的 2D 示意（忽略 z 轴）：

         vy=2  │ · ·  │  ·   │      │
         vy=1  │  ·   │ ····· │  ·   │   ← 点聚集在 (1,1) 体素
         vy=0  │ · ·  │  ·   │ ·    │
               ─────────────────────
               vx=0    vx=1    vx=2

点 (1.3, 1.7, 0.2) → 体素 (1, 1, 0)
点 (1.8, 1.1, 0.9) → 体素 (1, 1, 0)  ← 同一个体素！
点 (2.1, 1.5, 0.3) → 体素 (2, 1, 0)  ← 不同体素
```

**关键性质**：同一体素内的任意两个点，距离不超过体素对角线长度 $\sqrt{3} \cdot s$。这是邻域搜索正确性的基础。

### 2.4 体素哈希地图的完整数据结构 ⭐⭐

现在把所有概念组合成 KISS-ICP 使用的完整数据结构。

#### 2.4.1 整体架构

```
VoxelHashMap:
  ├── 数据存储：HashMap<Vector3i, vector<Vector3d>>
  │     key = 体素坐标 (vx, vy, vz)
  │     value = 该体素内的点集（最多 max_points_per_voxel 个）
  │
  ├── 参数：
  │     voxel_size = 1.0m          ← 体素边长
  │     max_points_per_voxel = 20  ← 每个体素最多存多少个点
  │
  └── 操作：
        insert(point)              ← 向地图添加一个点
        getClosestNeighbor(query)  ← 查询距 query 最近的地图点
        removeIfFar(origin, dist)  ← 删除远处的体素（滑窗）
```

#### 2.4.2 插入操作——逐步拆解

**输入**：一个世界坐标系下的三维点 $p = (3.7, 1.2, -0.5)$，$s = 1.0$

```
Step 1: 计算体素坐标
  vx = floor(3.7 / 1.0) = 3
  vy = floor(1.2 / 1.0) = 1
  vz = floor(-0.5 / 1.0) = -1        ← 注意负数用 floor 而非截断！
  voxel = (3, 1, -1)

Step 2: 哈希定位桶
  bucket_id = hash(3, 1, -1) % num_buckets

Step 3: 检查该体素是否已存在
  if 哈希表中不存在 key=(3,1,-1):
    创建新条目 (3,1,-1) → [p]           ← 新体素，第一个点
  else:
    获取现有点列表 points = map[(3,1,-1)]

Step 4: 防重叠检查
  if points.size() >= 20:
    跳过（桶满了）
  else if 任何已有点与 p 的距离 < map_resolution:
    跳过（太近了，会重叠）
  else:
    points.push_back(p)                 ← 追加
```

**为什么限制每个体素 20 个点？**

| 理由 | 解释 |
|------|------|
| 控制查询时间 | 27 邻域 × 20 点 = 540 次距离计算，上界固定 |
| 避免内存爆炸 | 长时间运行后某些热点体素可能积累上千个点 |
| 保证均匀性 | 配合 map_resolution 间距控制，20 个点在 $1\text{m}^3$ 内近似均匀 |

**map_resolution 的计算**：$\text{min\_dist}^2 = s^2 / \text{max\_points}$，即 $\text{min\_dist} = \sqrt{1.0 / 20} \approx 0.22\text{m}$。这是一个**经验启发式阈值**（KISS-ICP 的做法），不是严格的 3D 均匀分布推导（3D 均匀分布的理论间距是 $s / n^{1/3} \approx 0.37\text{m}$）。实际效果是：新点如果离已有点小于这个距离，信息增益不足——跳过。

#### 2.4.3 最近邻查询——27 邻域搜索详解

**输入**：查询点 $q = (3.5, 1.8, -0.3)$

```
Step 1: 计算 q 的体素坐标
  voxel_q = (3, 1, -1)    ← 和上面插入的点在同一个体素

Step 2: 生成 27 个邻居体素坐标
  对 dx, dy, dz ∈ {-1, 0, +1}:
    neighbor = (3+dx, 1+dy, -1+dz)

  具体生成：
    (2,0,-2) (3,0,-2) (4,0,-2)   ← dz=-1 层的 9 个
    (2,1,-2) (3,1,-2) (4,1,-2)
    (2,2,-2) (3,2,-2) (4,2,-2)
    (2,0,-1) (3,0,-1) (4,0,-1)   ← dz=0 层的 9 个（含自身体素）
    (2,1,-1) (3,1,-1) (4,1,-1)   ← (3,1,-1) 就是 q 自身所在体素
    (2,2,-1) (3,2,-1) (4,2,-1)
    (2,0, 0) (3,0, 0) (4,0, 0)   ← dz=+1 层的 9 个
    (2,1, 0) (3,1, 0) (4,1, 0)
    (2,2, 0) (3,2, 0) (4,2, 0)

Step 3: 查表 + 暴力比较
  best_dist = ∞
  best_point = null
  for each neighbor in 27_neighbors:
    if hash_map.contains(neighbor):
      for each point p in hash_map[neighbor]:
        d = ‖q - p‖²                     ← 用平方距离，避免 sqrt
        if d < best_dist:
          best_dist = d
          best_point = p

Step 4: 返回
  return best_point (如果 best_dist < max_correspondence_distance²)
```

**为什么 27 个邻居而不是只查自身体素？**

因为查询点可能位于体素的边缘——最近点可能在相邻体素中。

```
         体素 (3,1,-1)        体素 (4,1,-1)
    ┌──────────────────┐┌──────────────────┐
    │                  ││                  │
    │              q · ││ · p_closest      │
    │                  ││                  │
    └──────────────────┘└──────────────────┘

q 在 (3,1,-1) 的右边缘，但最近点 p 在 (4,1,-1) 中。
如果只查 q 自身体素，就会错过 p。
```

**为什么 27 个就够了，不需要更大范围？**

最近点到 q 的距离不可能超过 $\sqrt{3} \cdot s$（一个体素对角线长度），否则在体素内部一定有更近的点（或该区域没有点，不需要更大范围找）。而 27 邻域恰好覆盖了以 q 所在体素为中心、半径为 $s$ 的立方体邻域——任何距离 $< \sqrt{3} \cdot s$ 的点都必然落在这 27 个体素之一中。

**复杂度分析**：

```
27 次哈希查找（每次 O(1)）
× 每个体素最多 20 个点的线性扫描
= 最多 540 次距离计算

对比暴力搜索 500000 次距离计算 → 加速约 1000 倍
而且 540 是固定常数，与地图大小无关！
```

#### 2.4.4 滑窗地图——删除远处的体素

SLAM 中机器人持续移动，地图不能无限增长。KISS-ICP 的策略是**删除距当前位置过远的体素**：

```
removeIfFar(sensor_origin, max_distance):
  for each (voxel, points) in hash_map:
    if ‖points[0] - sensor_origin‖ > max_distance:
      hash_map.erase(voxel)
```

这维持了一个以传感器为中心的**滑动窗口**——只保留附近的地图，远处的丢弃。

### 2.5 体素哈希 vs 其他空间索引 ⭐⭐

#### 2.5.1 三种主流方案对比

| 方案 | 数据结构 | 查询复杂度 | 插入复杂度 | 适用场景 |
|------|---------|-----------|-----------|---------|
| KD-Tree | 平衡二叉树 | $O(\log M)$ | $O(M \log M)$ 重建 | 静态点集，批量配准 |
| ikd-Tree | 增量 KD 树 | $O(\log M)$ | $O(\log M)$ 增量 | LIO 高频更新（FAST-LIO2） |
| 体素哈希 | 哈希表 | $O(1)$ 均摊 | $O(1)$ 均摊 | LO 里程计（KISS-ICP） |

#### 2.5.2 KD-Tree 为什么不适合在线 SLAM

KD-Tree 通过**递归空间划分**实现 $O(\log M)$ 查询：每个节点用一个超平面将空间一分为二，交替选择 x/y/z 轴。

**问题**：KD-Tree 的平衡性来自**构建时对所有点排序**。如果地图不断增长（每帧加入新点），要么：
- **全量重建**：$O(M \log M)$，当 $M = 500000$ 时需要几十毫秒——太慢
- **不重建**：树逐渐失衡，查询退化到 $O(M)$

ikd-Tree（FAST-LIO2 使用）解决了增量问题——支持逐点插入和局部重平衡。但实现复杂（~2000 行 C++），且 GPLv2 许可证有商用限制。

**体素哈希不存在这个问题**——插入只需算体素坐标 + 写入哈希表，$O(1)$，与地图大小完全无关。

#### 2.5.3 体素哈希的局限性

体素哈希不是万能的。在以下场景中表现不佳：

**场景 1：KNN 查询（K 近邻）**

体素哈希擅长找**最近的 1 个点**（nearest neighbor），但不擅长找**最近的 K 个点**。KD-Tree 天然支持 KNN（沿树回溯即可），而体素哈希需要把 27 邻域的所有点排序——当 K 较大时不够高效。

**场景 2：半径搜索**

查找距离 $r$ 内的所有点。如果 $r \gg s$（搜索半径远大于体素边长），需要遍历大量邻居体素——$(2\lceil r/s \rceil + 1)^3$ 个，可能很多。

**场景 3：非均匀分布**

如果点云极度聚集（如室内一面墙上的密集扫描），少数体素内的点数接近 max_points，而大量体素是空的。此时 map_resolution 间距控制会丢弃大量点，降低地图精度。

### 2.6 SimpleSLAM 已有的基础设施

查看项目已有的 `VoxelGrid`（`core/math/voxel_grid.hpp`）和 `KDTree`（`core/math/kdtree.hpp`）：

- `VoxelGrid`：用于**降采样**（每个体素取一个代表点），不存储多点桶
- `KDTree`：批量构建，不支持增量插入

**VoxelHashTarget 需要新实现**——不能直接复用 `VoxelGrid`，因为需求不同：

| 需求 | VoxelGrid（降采样） | VoxelHashTarget（配准地图） |
|------|--------------------|-----------------------------|
| 每个体素存多少点 | 1 个（代表点） | 最多 20 个 |
| 是否支持增量插入 | 否（一次性） | 是（逐帧追加） |
| 是否支持最近邻查询 | 否 | 是（27 邻域） |
| 是否需要删除旧点 | 否 | 是（滑窗地图） |

### 2.7 完整代码走读——构建你的理解 ⭐⭐

**以下代码是完整可运行的简化版体素哈希地图**，用于帮助你理解原理。你的实现任务在练习中。

```cpp
#include <Eigen/Core>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <vector>

/// 将 3D 点转换为体素坐标
inline Eigen::Vector3i pointToVoxel(const Eigen::Vector3d& point, double voxel_size) {
    // 为什么用 floor 而非 int() 截断？
    // int(-0.3) = 0（向零取整），floor(-0.3) = -1（向负无穷取整）
    // 向零取整会让负坐标区域的体素边界不对称
    return {static_cast<int>(std::floor(point.x() / voxel_size)),
            static_cast<int>(std::floor(point.y() / voxel_size)),
            static_cast<int>(std::floor(point.z() / voxel_size))};
}

/// 三素数空间哈希函数
struct VoxelHash {
    size_t operator()(const Eigen::Vector3i& v) const {
        // 为什么用三个大素数？
        // 素数乘法在整数域上充分"搅拌"每个坐标
        // 异或(^)让每一位独立混合，不会产生进位传播
        return static_cast<size_t>(
            (v.x() * 73856093) ^ (v.y() * 19349669) ^ (v.z() * 83492791));
    }
};

/// 为什么需要自定义相等比较？
/// std::unordered_map 在哈希冲突时需要比较 key 是否真正相等
/// Eigen::Vector3i 默认没有 == 用于 unordered_map 的特化
struct VoxelEqual {
    bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
        return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
};

class SimpleVoxelMap {
public:
    explicit SimpleVoxelMap(double voxel_size, int max_points_per_voxel = 20)
        : voxel_size_(voxel_size)
        , max_points_(max_points_per_voxel)
        // map_resolution 控制同一体素内点的最小间距
        // = sqrt(体素体积 / 最大点数) 的简化版
        , min_dist_sq_(voxel_size * voxel_size / max_points_per_voxel) {}

    void insert(const Eigen::Vector3d& point) {
        auto voxel = pointToVoxel(point, voxel_size_);
        auto& bucket = map_[voxel];     // 不存在时自动创建空 vector

        // 桶满了 → 跳过
        if (static_cast<int>(bucket.size()) >= max_points_) return;

        // 和已有点太近 → 跳过（防重叠）
        for (const auto& existing : bucket) {
            if ((existing - point).squaredNorm() < min_dist_sq_) return;
        }

        bucket.push_back(point);
    }

    /// 27 邻域最近邻搜索
    std::optional<Eigen::Vector3d> closestNeighbor(
            const Eigen::Vector3d& query, double max_dist) const {
        auto voxel = pointToVoxel(query, voxel_size_);
        double best_dist_sq = max_dist * max_dist;
        std::optional<Eigen::Vector3d> best_point;

        // 遍历 3×3×3 = 27 个邻居体素
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    Eigen::Vector3i neighbor = voxel + Eigen::Vector3i(dx, dy, dz);
                    auto it = map_.find(neighbor);
                    if (it == map_.end()) continue;  // 该体素不存在

                    for (const auto& p : it->second) {
                        double d_sq = (query - p).squaredNorm();
                        if (d_sq < best_dist_sq) {
                            best_dist_sq = d_sq;
                            best_point = p;
                        }
                    }
                }
            }
        }
        return best_point;   // nullopt 表示没找到
    }

private:
    double voxel_size_;
    int max_points_;
    double min_dist_sq_;
    std::unordered_map<Eigen::Vector3i, std::vector<Eigen::Vector3d>,
                       VoxelHash, VoxelEqual> map_;
};
```

**对比 KISS-ICP 实现的差异**：

| 方面 | 上面的简化版 | KISS-ICP 实现 |
|------|------------|---------------|
| 哈希表 | `std::unordered_map`（链地址） | `tsl::robin_map`（开放寻址） |
| 性能 | 教学用，足够理解原理 | 工程优化，缓存友好 |
| 并行 | 无 | TBB `parallel_for` 并行查询 |
| 滑窗删除 | 未实现 | `RemovePointsFarFromLocation` |

### 2.8 ⚠️ 常见陷阱

**⚠️ 编程陷阱：体素坐标计算用 `int(x / voxel_size)` 而非 `floor`**

```cpp
// ❌ 错误：int() 截断是向零取整
int vx = int(point.x() / voxel_size);
// 当 point.x() = -0.3, voxel_size = 1.0 时：
// int(-0.3) = 0 ← 错误！应该是 -1

// ✅ 正确：floor() 是向负无穷取整
int vx = static_cast<int>(std::floor(point.x() / voxel_size));
// floor(-0.3) = -1.0 → int(-1.0) = -1 ✅
```

现象：负坐标区域的体素索引全部偏移一格，导致体素边界不对称——传感器左右两侧的配准精度不同，debug 时极难发现。

**💡 概念误区：体素越小配准越精确**

新手想法："把 `voxel_size` 从 1.0m 调到 0.1m，体素更精细，地图分辨率更高，配准肯定更准。"

实际上：体素太小会导致两个问题：
1. **哈希表爆炸**：$0.1\text{m}$ 体素在 $100\text{m} \times 100\text{m}$ 范围内有 $10^6$ 个体素，内存和查找都变慢
2. **稀疏匹配**：每个体素可能只有 0-1 个点，27 邻域搜索经常找不到足够匹配

KISS-ICP 的默认策略：`voxel_size = 0.01 × max_range`。对于 100m 范围的 LiDAR，`voxel_size = 1.0m`。

**⚠️ 编程陷阱：用 `std::unordered_map` 存 `Eigen::Vector3i` 却忘了自定义 Hash 和 Equal**

```cpp
// ❌ 编译错误：Eigen::Vector3i 没有默认的 std::hash 特化
std::unordered_map<Eigen::Vector3i, std::vector<Eigen::Vector3d>> map;

// ✅ 必须提供自定义 Hash 和 Equal
std::unordered_map<Eigen::Vector3i, std::vector<Eigen::Vector3d>,
                   VoxelHash, VoxelEqual> map;
```

**🧠 思维陷阱：认为哈希查找"真的是" $O(1)$**

$O(1)$ 是**均摊**意义上的。在最坏情况下（所有 key 冲突到同一个桶），退化为 $O(n)$。实际中好的哈希函数 + 合理负载因子（$\alpha < 0.7$）时，平均每次查找只需 1-2 步——但这不是**保证**。与之相比，平衡二叉树的 $O(\log n)$ 是**最坏情况保证**。

在 SLAM 中，三素数哈希函数在体素坐标上的分散性已被大量实验验证——实际冲突率极低，$O(1)$ 假设成立。

### 2.9 练习

1. **手动模拟**：在纸上画一个 $4 \times 4$ 的二维网格（$v_x \in [0,3]$，$v_y \in [0,3]$），用 $h(v_x, v_y) = (v_x \cdot 73856093 \oplus v_y \cdot 19349669) \% 8$ 计算每个体素的桶编号。有多少对体素发生了冲突？

2. **编程题**：基于上面的 `SimpleVoxelMap`，添加 `removeIfFar(Vector3d origin, double max_dist)` 方法。实现滑窗功能。在 KITTI 第 00 序列的前 50 帧上运行，每帧插入降采样后的点云并删除 100m 外的体素，打印每帧后的体素总数——验证地图大小是否稳定在一个范围内。

3. **对比实验**：对 KITTI 第 00 序列的第一帧点云（~120000 点），分别用暴力搜索和体素哈希找 1000 个随机查询点的最近邻，对比耗时。`voxel_size` 分别取 0.5m、1.0m、2.0m，观察速度和准确率的变化。

4. **思考题**：在什么场景下体素哈希会退化？提示：考虑一个极度狭长的走廊，宽 2m、长 500m。与 KD-Tree 相比在这个场景下谁更优？为什么？

---

最近邻搜索解决了"找对应"的问题。接下来是 ICP 的第二个子问题——**给定对应关系，如何求最优刚体变换？** §3 先介绍经典的 SVD 闭式解，§4 再升级为工程中实际使用的 Gauss-Newton 方法。

## 3. SVD 闭式解——经典 Arun 方法 ⭐⭐

### 3.1 动机：已知对应时的最优解

假设对应关系 $c(i)$ 已经确定（ICP 外层循环的 Step 1 已完成），现在需要求解：

$$
\min_{R \in \text{SO}(3),\, t \in \mathbb{R}^3} \sum_{i=1}^{N} \| q_i - (R p_i + t) \|^2
$$

这里 $q_i$ 就是 $q_{c(i)}$，为了简洁省略下标。

**如果不用 SVD 会怎样？** 可以暴力展开目标函数对 $R$ 的 9 个元素和 $t$ 的 3 个元素求偏导——但 $R$ 有正交约束 $R^T R = I$，需要用拉格朗日乘子处理，推导极其繁琐。SVD 方法巧妙地绕过了正交约束，直接构造出满足约束的最优解。

### 3.2 完整推导

**Step 1：分离旋转和平移。**

展开目标函数：
$$
E(R, t) = \sum_{i=1}^{N} \| q_i - R p_i - t \|^2
$$

对 $t$ 求导并令其为零：
$$
\frac{\partial E}{\partial t} = -2 \sum_{i=1}^{N} (q_i - R p_i - t) = 0
$$

$$
N t = \sum_{i=1}^{N} q_i - R \sum_{i=1}^{N} p_i
$$

$$
t^* = \bar{q} - R \bar{p}
$$

其中 $\bar{p} = \frac{1}{N}\sum p_i$，$\bar{q} = \frac{1}{N}\sum q_i$ 是两组点的质心。

**为什么这很重要？** 最优平移完全由旋转和两个质心决定。这意味着我们可以先求 $R$，再回代得 $t$——问题从 12 个未知数（$R$ 9个 + $t$ 3个）降为只需求 $R$。

**Step 2：代入 $t^*$，化简为只关于 $R$ 的问题。**

定义**去质心坐标**：$\tilde{p}_i = p_i - \bar{p}$，$\tilde{q}_i = q_i - \bar{q}$。

代入 $t^* = \bar{q} - R\bar{p}$：
$$
q_i - R p_i - t^* = (q_i - \bar{q}) - R(p_i - \bar{p}) = \tilde{q}_i - R \tilde{p}_i
$$

因此：
$$
E(R) = \sum_{i=1}^{N} \| \tilde{q}_i - R \tilde{p}_i \|^2
$$

展开范数的平方：
$$
E(R) = \sum_i \left[ \tilde{q}_i^T \tilde{q}_i - 2 \tilde{q}_i^T R \tilde{p}_i + \tilde{p}_i^T R^T R \tilde{p}_i \right]
$$

由于 $R^T R = I$（正交矩阵），第三项 $= \tilde{p}_i^T \tilde{p}_i$，与 $R$ 无关。第一项也与 $R$ 无关。因此：

$$
\min_R E(R) \iff \max_R \sum_i \tilde{q}_i^T R \tilde{p}_i
$$

**Step 3：用迹（trace）重写。**

利用迹的性质 $a^T B c = \text{tr}(c a^T B)$（标量 = 1×1 矩阵的迹）：

$$
\sum_i \tilde{q}_i^T R \tilde{p}_i = \sum_i \text{tr}(\tilde{p}_i \tilde{q}_i^T R) = \text{tr}\left( \sum_i \tilde{p}_i \tilde{q}_i^T \cdot R \right) = \text{tr}(W R)
$$

其中 **交叉协方差矩阵** $W = \sum_{i=1}^N \tilde{p}_i \tilde{q}_i^T$ 是一个 $3 \times 3$ 矩阵。

> **注意**：有些文献定义 $H = \sum \tilde{p}_i \tilde{q}_i^T$（源在左），有些定义 $H = \sum \tilde{q}_i \tilde{p}_i^T$（目标在左）。两者的 SVD 结果中 $U$ 和 $V$ 的角色互换。本文使用 **源在左** 的约定，与 Arun 1987 原文一致。

**问题变为**：$\max_{R \in \text{SO}(3)} \text{tr}(W R)$。

**Step 4：SVD 求解。**

对 $W$ 做奇异值分解：$W = U \Sigma V^T$，其中 $U, V \in O(3)$，$\Sigma = \text{diag}(\sigma_1, \sigma_2, \sigma_3)$，$\sigma_1 \geq \sigma_2 \geq \sigma_3 \geq 0$。

$$
\text{tr}(W R) = \text{tr}(U \Sigma V^T R) = \text{tr}(\Sigma \cdot V^T R U)
$$

令 $M = V^T R U$。由于 $R, U, V$ 都是正交矩阵，$M$ 也是正交矩阵，满足 $|M_{ii}| \leq 1$。

$$
\text{tr}(\Sigma M) = \sigma_1 M_{11} + \sigma_2 M_{22} + \sigma_3 M_{33}
$$

由于 $|M_{ii}| \leq 1$，当且仅当 $M = I$ 时取最大值 $\sigma_1 + \sigma_2 + \sigma_3$。

$M = I \Rightarrow V^T R U = I \Rightarrow$

$$
\boxed{R^* = V U^T}
$$

**Step 5：反射修正。**

如果 $\det(V U^T) = -1$，说明 SVD 给出的是一个**反射**而非旋转（发生在点集接近共面时）。

修正方法：翻转 $V$ 最后一列的符号：

$$
R^* = V \cdot \text{diag}(1, 1, \det(VU^T)) \cdot U^T
$$

**直觉**：$\det(VU^T) = -1$ 意味着从源到目标的"最优对齐"包含了一次镜像翻转。但物理世界中刚体不会翻转，所以我们放弃在第三个奇异值方向上的优化（代价最小），换取 $\det(R) = +1$。

**最终公式汇总**：

$$
\bar{p} = \frac{1}{N}\sum p_i, \quad \bar{q} = \frac{1}{N}\sum q_i
$$

$$
W = \sum_{i=1}^N (p_i - \bar{p})(q_i - \bar{q})^T, \quad W = U\Sigma V^T
$$

$$
R^* = V \cdot \text{diag}(1, 1, \det(VU^T)) \cdot U^T
$$

$$
t^* = \bar{q} - R^* \bar{p}
$$

### 3.3 C++ 实现引导

**你的任务**：实现一个函数 `solveSVD`，输入配对好的源点和目标点，输出最优 $\text{SE}(3)$ 变换。

**Step 1: 先理解为什么这样写**

SVD 求解只需要一个 $3 \times 3$ 矩阵的 SVD 分解——这比分解一个 $N \times 3$ 矩阵快得多。关键洞察是交叉协方差矩阵 $W$ 将 $N$ 个点压缩成了一个 $3 \times 3$ 矩阵，丢弃了个体信息但保留了旋转信息。

**Step 2: 代码骨架（你来填核心部分）**

```cpp
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <manif/SE3.h>
#include <vector>

/// 给定 N 对匹配点，SVD 闭式解求最优刚体变换
/// @param source 源点（当前帧，传感器坐标系）
/// @param target 目标点（地图坐标系）
/// @return T_target_source，将源点变换到目标坐标系的变换
manif::SE3d solveSVD(const std::vector<Eigen::Vector3d>& source,
                      const std::vector<Eigen::Vector3d>& target) {
    assert(source.size() == target.size());
    assert(source.size() >= 3);  // 至少 3 个不共线的点
    const int N = static_cast<int>(source.size());

    // ── Step 1: 计算质心 ──
    // TODO: 你来实现
    // Eigen::Vector3d p_bar = ...
    // Eigen::Vector3d q_bar = ...

    // ── Step 2: 构建交叉协方差矩阵 W ──
    // TODO: 你来实现
    // Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    // for (int i = 0; i < N; ++i) {
    //     W += ...
    // }

    // ── Step 3: SVD 分解 ──
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // ── Step 4: 计算旋转矩阵（含反射修正）──
    // TODO: 你来实现
    // 提示：检查 det(V * U.transpose())，必要时翻转 V 的最后一列
    // Eigen::Matrix3d R = ...

    // ── Step 5: 计算平移 ──
    // TODO: 你来实现
    // Eigen::Vector3d t = ...

    // ── 构造 SE3d 返回 ──
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    return manif::SE3d(T);
}
```

**Step 3: 常见错误写法**

```cpp
// ❌ 错误 1：W 的构造方向反了
W += (q_i - q_bar) * (p_i - p_bar).transpose();  // 目标在左，源在右
// 结果：R 变成了 U * V^T 而非 V * U^T，求出的是反向变换

// ❌ 错误 2：忘记反射修正
Eigen::Matrix3d R = V * U.transpose();  // 直接用
// 结果：当点接近共面时 det(R) = -1，这不是有效旋转
// 后果：manif::SE3d 构造会断言失败，或者后续运动累积出镜像轨迹

// ❌ 错误 3：SVD 不请求完整 U 和 V
Eigen::JacobiSVD<Eigen::Matrix3d> svd(W);  // 不加 ComputeFullU | ComputeFullV
// 结果：svd.matrixU() 和 svd.matrixV() 返回空矩阵
// Eigen 默认不计算 U 和 V 以节省时间——必须显式请求
```

**Step 4: SVD 方法的局限性**

| 方面 | SVD 闭式解 | Gauss-Newton（下一节） |
|------|-----------|----------------------|
| 支持鲁棒权重 | ❌ 不能——所有点等权 | ✅ 天然支持 IRLS |
| 支持点到面残差 | ❌ 不能 | ✅ 只需换雅可比 |
| 单次求解速度 | 极快（$3 \times 3$ SVD） | 较快（$6 \times 6$ 线性系统） |
| 数值稳定性 | 极好 | 需要 LDLT 等稳定分解 |

**结论**：SVD 方法理论优雅，但实际工程中（KISS-ICP、FAST-LIO2、LIO-SAM）**全部使用 Gauss-Newton**，因为需要鲁棒核函数处理外点。

### 3.4 ⚠️ 常见陷阱

**⚠️ 编程陷阱：Eigen SVD 的 ComputeFullU/ComputeFullV**

```cpp
// ❌ 未请求完整分解
Eigen::JacobiSVD<Eigen::Matrix3d> svd(W);
auto U = svd.matrixU();  // U 是空矩阵！
auto V = svd.matrixV();  // V 也是空矩阵！
// 后续 V * U.transpose() 得到 0×0 矩阵，静默产生错误结果

// ✅ 必须显式请求
Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
```

自检方法：SVD 后立刻检查 `assert(svd.matrixU().rows() == 3 && svd.matrixU().cols() == 3)`。

**💡 概念误区：认为 SVD 是在"分解变换矩阵"**

新手想法："SVD 分解的是 $R$ 或 $T$，从中提取旋转和平移。"

实际上：SVD 分解的是**交叉协方差矩阵** $W = \sum \tilde{p}_i \tilde{q}_i^T$（$3 \times 3$），不是变换矩阵。$W$ 编码了两组去质心点之间的"相关性方向"。$U$ 和 $V$ 分别是目标侧和源侧的主方向，$\Sigma$ 度量各方向上的相关强度。$R^* = VU^T$ 的几何含义是"将源的主方向旋转到目标的主方向"。

### 3.5 练习

1. **手推题**：给定 $p_1 = (1, 0, 0)$，$p_2 = (0, 1, 0)$，$p_3 = (0, 0, 1)$；$q_1 = (0, 1, 0)$，$q_2 = (-1, 0, 0)$，$q_3 = (0, 0, 1)$。手动计算 $W$，进行 SVD（提示：$W$ 是排列矩阵的变体），验证 $R = VU^T$ 是绕 $z$ 轴旋转 $90°$。

2. **编程题**：完成上面的 `solveSVD` 函数骨架，并用以下测试验证：构造 $R = R_x(10°) R_y(20°) R_z(30°)$，$t = (1, 2, 3)^T$，生成 100 个随机源点，计算 $q_i = R p_i + t$，调用 `solveSVD`，验证恢复出的 $R$ 和 $t$ 与原始值相差不超过 $10^{-10}$。

3. **进阶题**：在上面的测试中给 $q_i$ 加上 $\sigma = 0.05\text{m}$ 的高斯噪声，观察恢复精度如何变化。尝试 $N = 10, 100, 1000$ 个点，画出 $N$ vs 旋转误差的关系。你预期是什么趋势？（提示：$\sqrt{N}$ 收敛）

---

## 4. SE(3) 上的 Gauss-Newton ⭐⭐⭐

本节是全章最核心的内容，将分 10 个小节从基础概念逐步推进到完整实现：

```
4.1  为什么不用 SVD          ← 动机
4.2  问题建模                ← 数学框架
4.3  什么是雅可比矩阵        ← 前置数学（可跳过）
4.4  扰动模型                ← 李群上的"微分"
4.5  雅可比推导              ← 核心公式推导
4.6  MatchResult 格式        ← 公式如何落地到代码
4.7  完整 GN 迭代            ← 伪代码
4.8  C++ 实现引导            ← 代码骨架
4.9  常见陷阱                ← 避坑
4.10 练习                    ← 动手
```

如果你对雅可比和李群已经熟悉，可以跳过 §4.3-4.4 直接看 §4.5。

### 4.1 动机：从 SVD 到 Gauss-Newton

§3 的 SVD 闭式解优雅直接，但工程中无法使用——它有两个硬伤：

1. **无法加权**——所有匹配点等权参与。但实际中 10-30% 的对应是外点（动态物体、遮挡），不加权就会被外点带偏。§6 的鲁棒核函数需要**逐点加权**，SVD 无法支持。
2. **只能做点到点**——§5 的点到面 ICP 使用标量残差和不同的雅可比结构，SVD 的交叉协方差矩阵无法表达。

Gauss-Newton 方法天然解决这两个问题：它把配准转化为**加权非线性最小二乘**问题，在 SE(3) 的切空间上线性化求解。残差类型和权重通过 MatchResult 传入，求解器本身不关心具体形式——这就是 §7 中 VoxelHashTarget 和 IcpSolver 能完全解耦的数学基础。

### 4.2 问题建模

**在 $\text{SE}(3)$ 上的最小化**：

$$
\min_{T \in \text{SE}(3)} \sum_{i=1}^{N} \rho\left( \| r_i(T) \| \right)
$$

其中 $r_i(T)$ 是第 $i$ 个匹配的残差函数（取决于配准类型），$\rho(\cdot)$ 是鲁棒核函数（L2 时 $\rho(s) = s^2/2$）。

### 4.3 前置知识：什么是雅可比矩阵 ⭐

在进入 SE(3) 优化之前，先确保你理解雅可比矩阵。如果你对此已经熟悉，可跳到 §4.4（扰动模型）。

#### 4.3.1 从一维导数说起

$$
f(x) = x^2 \quad \Rightarrow \quad f'(x) = 2x
$$

导数 $f'(x)$ 的含义：**$x$ 变化一点点 $\delta x$，$f$ 变化多少？**

$$
f(x + \delta x) \approx f(x) + f'(x) \cdot \delta x
$$

比如 $x=3$：$f'(3) = 6$，意思是 $x$ 增加 0.01，$f$ 大约增加 $6 \times 0.01 = 0.06$。

#### 4.3.2 多输入一输出——梯度

函数有 3 个输入，1 个输出：

$$
f(x, y, z) = x^2 + 2y + z
$$

$$
\nabla f = \begin{bmatrix} \frac{\partial f}{\partial x} \\ \frac{\partial f}{\partial y} \\ \frac{\partial f}{\partial z} \end{bmatrix} = \begin{bmatrix} 2x \\ 2 \\ 1 \end{bmatrix}
$$

这叫**梯度**——一个 3×1 向量，告诉你每个输入方向的变化率。

#### 4.3.3 多输入多输出——雅可比矩阵

函数有 $n$ 个输入，$m$ 个输出：

$$
\mathbf{f}(\mathbf{x}) = \begin{bmatrix} f_1(x_1, x_2, \ldots, x_n) \\ f_2(x_1, x_2, \ldots, x_n) \\ \vdots \\ f_m(x_1, x_2, \ldots, x_n) \end{bmatrix}
$$

**雅可比矩阵**就是把每个输出对每个输入的偏导排成一个 $m \times n$ 矩阵：

$$
J = \begin{bmatrix}
\frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2} & \cdots & \frac{\partial f_1}{\partial x_n} \\
\frac{\partial f_2}{\partial x_1} & \frac{\partial f_2}{\partial x_2} & \cdots & \frac{\partial f_2}{\partial x_n} \\
\vdots & & \ddots & \\
\frac{\partial f_m}{\partial x_1} & \frac{\partial f_m}{\partial x_2} & \cdots & \frac{\partial f_m}{\partial x_n}
\end{bmatrix} \in \mathbb{R}^{m \times n}
$$

**物理含义不变**——输入变化 $\delta\mathbf{x}$（$n$ 维），输出变化多少？

$$
\mathbf{f}(\mathbf{x} + \delta\mathbf{x}) \approx \mathbf{f}(\mathbf{x}) + J \cdot \delta\mathbf{x}
$$

| 概念 | 输入维度 | 输出维度 | "导数"的形状 |
|------|---------|---------|------------|
| 导数 $f'$ | 1 | 1 | 标量 |
| 梯度 $\nabla f$ | $n$ | 1 | $n \times 1$ 向量 |
| 雅可比 $J$ | $n$ | $m$ | $m \times n$ 矩阵 |

**一句话：雅可比矩阵就是多维版的导数。**

#### 4.3.4 具体例子

$$
\mathbf{f}(x, y) = \begin{bmatrix} x + 2y \\ x^2 y \end{bmatrix}
$$

2 个输入 $(x, y)$，2 个输出 $(f_1, f_2)$，雅可比是 $2 \times 2$：

$$
J = \begin{bmatrix} \frac{\partial f_1}{\partial x} & \frac{\partial f_1}{\partial y} \\ \frac{\partial f_2}{\partial x} & \frac{\partial f_2}{\partial y} \end{bmatrix} = \begin{bmatrix} 1 & 2 \\ 2xy & x^2 \end{bmatrix}
$$

在 $(x, y) = (1, 3)$ 处：$J = \begin{bmatrix} 1 & 2 \\ 6 & 1 \end{bmatrix}$

含义：$x$ 增加 0.01 → $f_1$ 增加 $1 \times 0.01 = 0.01$，$f_2$ 增加 $6 \times 0.01 = 0.06$。

#### 4.3.5 ICP 中的雅可比——3 个输出 × 6 个输入

ICP 的残差函数：

- **输出**：$\mathbf{r} = \hat{p} - q \in \mathbb{R}^3$（残差的 $x, y, z$ 三个分量）
- **输入**：$\delta\xi \in \mathbb{R}^6$（位姿微调量——3 个平移 + 3 个旋转）

所以雅可比是 $3 \times 6$——3 行（残差分量）、6 列（位姿微调方向）：

$$
J = \frac{\partial \mathbf{r}}{\partial \delta\xi} = \begin{bmatrix}
\frac{\partial r_x}{\partial \delta t_x} & \frac{\partial r_x}{\partial \delta t_y} & \frac{\partial r_x}{\partial \delta t_z} & \frac{\partial r_x}{\partial \delta\phi_x} & \frac{\partial r_x}{\partial \delta\phi_y} & \frac{\partial r_x}{\partial \delta\phi_z} \\
\frac{\partial r_y}{\partial \delta t_x} & \frac{\partial r_y}{\partial \delta t_y} & \frac{\partial r_y}{\partial \delta t_z} & \frac{\partial r_y}{\partial \delta\phi_x} & \frac{\partial r_y}{\partial \delta\phi_y} & \frac{\partial r_y}{\partial \delta\phi_z} \\
\frac{\partial r_z}{\partial \delta t_x} & \frac{\partial r_z}{\partial \delta t_y} & \frac{\partial r_z}{\partial \delta t_z} & \frac{\partial r_z}{\partial \delta\phi_x} & \frac{\partial r_z}{\partial \delta\phi_y} & \frac{\partial r_z}{\partial \delta\phi_z}
\end{bmatrix}
$$

左扰动下的结果（§4.5 会推导）：

$$
J = \begin{bmatrix} I_3 & -[\hat{p}]_\times \end{bmatrix}
$$

展开后每一列的含义一目了然：

```
        δtx  δty  δtz  δφx    δφy    δφz
  rx: [  1    0    0    0      pz    -py  ]   ← 平移 x → 残差 x 直接 +1
  ry: [  0    1    0   -pz     0      px  ]   ← 绕 x 转 → ry 按 -pz 变化
  rz: [  0    0    1    py    -px     0   ]   ← 绕 z 转 → rx 按 -py 变化
       ├── 平移部分 ──┤  ├── 旋转部分 ──────┤
```

**前 3 列（平移）**是单位矩阵——平移和残差是 1:1 直接关系。

**后 3 列（旋转）**取决于点的位置 $\hat{p}$——离旋转轴越远的点，旋转产生的位移越大。这就是反对称矩阵 $[\hat{p}]_\times$ 的几何含义。

#### 4.3.6 雅可比在 Gauss-Newton 中的角色

有了雅可比，就能把非线性问题**局部线性化**：

$$
\mathbf{r}(\delta\xi) \approx \mathbf{r}_0 + J \cdot \delta\xi
$$

代入最小二乘 $\min \sum r_i^2$，展开求导令其为零，得到**法方程**：

$$
J^T J \cdot \delta\xi = -J^T \mathbf{r}_0
$$

这是一个 $6 \times 6$ 的线性系统——IcpSolver 解的就是它。

**完整链路**：

```
雅可比 = "残差对位姿微调的敏感度矩阵"
  ↓
告诉优化器：往哪个方向调位姿能最快减小残差
  ↓
法方程 JᵀJ δξ = -Jᵀr 求出最优调整量 δξ
  ↓
更新位姿（§4.4 会讲怎么更新——不是简单的加法）
```

现在你知道了雅可比是什么、它在优化中起什么作用。下一个问题是：$\delta\xi$ 是什么？它如何作用到位姿 $T$ 上？这就是**扰动模型**。

### 4.4 扰动模型——李群上的"微分叠加"

#### 4.4.1 为什么需要扰动

不能对 $T$ 的 12 个矩阵元素直接求导（因为有 6 个约束 $R^T R = I$, $\det R = 1$）。正确做法是在**切空间**上参数化微小更新。

**和标量优化的完全类比**：

| | 标量优化 | SE(3) 上的优化 |
|---|---------|---------------|
| 当前值 | $x_k$ | $T_k$ |
| 微小增量 | $\delta x$（标量） | $\delta\xi$（6D 向量） |
| 求导 | $\frac{\partial f}{\partial x}$ | $\frac{\partial r}{\partial \delta\xi}$（雅可比） |
| 更新 | $x_{k+1} = x_k + \delta x$ | $T_{k+1} = \text{Exp}(\delta\xi) \cdot T_k$ 或 $T_k \cdot \text{Exp}(\delta\xi)$ |

标量用**加法**叠加微分，位姿用**乘法**叠加微分。原因：SE(3) 不是向量空间——两个旋转矩阵相加不是旋转矩阵，但两个变换矩阵相乘仍是变换矩阵。$\text{Exp}(\delta\xi)$ 是指数映射，把 6D 微分量"变成"一个 SE(3) 变换矩阵。当 $\delta\xi$ 很小时 $\text{Exp}(\delta\xi) \approx I + \delta\xi^\wedge$，与标量的 $1 + \delta x$ 完全对应。

**"扰动"就是"微分"在李群上的说法。**

#### 4.4.2 左扰动 vs 右扰动——从哪边叠加微分

微分叠加有两种方式——左乘或右乘：

$$
\text{左扰动：} T' = \text{Exp}(\delta\xi) \cdot T \quad \text{（左乘，δξ 在世界坐标系下描述微小运动）}
$$

$$
\text{右扰动：} T' = T \cdot \text{Exp}(\delta\xi) \quad \text{（右乘，δξ 在机体坐标系下描述微小运动）}
$$

**物理直觉**——想象你站在原点，面朝北方（机体 $x$ 轴朝北）：

```
世界坐标系 W          机体坐标系 B
     N (x_W)              ↑ x_B（你面朝的方向）
     ↑                    |
     |                    |
W ───┼──→ E (y_W)    ← ──┼──→ y_B
     |                    |
```

- **左扰动** $\delta t = (0.01, 0, 0)$：在**世界** $x$ 方向移动 0.01m → 不管你面朝哪，都向北走
- **右扰动** $\delta t = (0.01, 0, 0)$：在**机体** $x$ 方向移动 0.01m → 你面朝哪就向哪走

当你面朝北时两者一样。当你面朝东时：左扰动仍然向北，右扰动则向东。

#### 4.4.3 两种扰动的详细对比

| 维度 | 左扰动 | 右扰动 |
|------|--------|--------|
| 更新公式 | $T' = \text{Exp}(\delta\xi) \cdot T$ | $T' = T \cdot \text{Exp}(\delta\xi)$ |
| $\delta\xi$ 的坐标系 | 世界坐标系 | 机体坐标系 |
| 点到点雅可比 | $J = [I_3 \mid -[\hat{p}]_\times]$，$\hat{p} = Rp+t$ | $J = [R \mid -R[p]_\times]$，$p$ 在传感器系 |
| 雅可比复杂度 | 更简洁（不含 $R$） | 含 $R$ 乘法 |
| manif 接口 | `SE3d::lplus(tangent)` | `SE3d::rplus(tangent)` |
| 使用者 | KISS-ICP、本项目 | GTSAM、Ceres（默认） |

**关键约束**：**推导和更新必须使用同一种约定**。用左扰动推导雅可比就必须用 $T \leftarrow \text{Exp}(\delta\xi) \cdot T$ 更新。混用会导致收敛方向错误——不会报错，但结果是错的。

**最终结果一样**：两种扰动收敛到同一个最优位姿——只是每步的 $\delta\xi$ 数值不同，优化路径不同。

#### 4.4.4 本节使用右扰动推导（教学目的）

下面的雅可比推导使用右扰动（便于从 SE(3) 结构直观理解），但**本项目代码使用左扰动**（和 KISS-ICP 一致，雅可比更简洁）。

扭量 $\delta\xi \in \mathbb{R}^6$：

$$
\delta\xi = \begin{pmatrix} \delta t \\ \delta \phi \end{pmatrix} \in \mathbb{R}^6
$$

前 3 个分量 $\delta t$ 是平移增量，后 3 个 $\delta \phi$ 是旋转增量（轴角向量）。

### 4.5 点到点残差的雅可比推导

理解了扰动的物理含义后，现在推导 ICP 中最核心的公式——残差对位姿微调的雅可比矩阵。先用右扰动推导（能看到完整的矩阵结构从何而来），再给出左扰动的结果（代码实际使用）。

#### 4.5.1 残差定义

变换后的源点 $\hat{p}_i = R p_i + t$ 到目标最近点 $q_i$ 的偏差：

$$
r_i = R p_i + t - q_i \in \mathbb{R}^3
$$

这是一个 3D 向量（3 个输出）。我们需要求：**位姿 $T$ 微调 $\delta\xi$（6 个输入）时，$r_i$ 怎么变？** 即求 $3 \times 6$ 的雅可比矩阵。

#### 4.5.2 右扰动推导——逐步展开

**Step 1：写出扰动后的变换矩阵。**

右扰动 $T' = T \cdot \text{Exp}(\delta\xi)$，展开为 $4 \times 4$ 矩阵乘法：

$$
T' = \begin{pmatrix} R & t \\ 0 & 1 \end{pmatrix} \begin{pmatrix} I + [\delta\phi]_\times & \delta t \\ 0 & 1 \end{pmatrix}
$$

按矩阵分块乘法规则（左上 = 左上×左上 + 右上×左下，右上 = 左上×右上 + 右上×右下）：

$$
T' = \begin{pmatrix} R(I+[\delta\phi]_\times) & R\delta t + t \\ 0 & 1 \end{pmatrix}
$$

从中读出扰动后的旋转和平移：
- 新旋转：$R' = R + R[\delta\phi]_\times$
- 新平移：$t' = R \delta t + t$

**为什么新平移是 $R \delta t + t$ 而不是 $t + \delta t$？**

因为右扰动的 $\delta t$ 是在**机体坐标系**下描述的微小平移。要转换到世界坐标系下的实际位移，必须乘以旋转矩阵 $R$：

```
例：你面朝东北方向（机体 x 轴 = 东北方向）

右扰动 δt = (0.01, 0, 0) → "沿机体 x 方向前进 0.01m"

世界系位移 = R · (0.01, 0, 0)
           = (0.007, 0.007, 0)   → 东北方向 ✅

如果不乘 R，直接 t + δt：
           = 原位置 + (0.01, 0, 0) → 向正东走 → 方向错误 ❌
```

**Step 2：代入残差公式。**

$$
r_i' = R' p_i + t' - q_i = (R + R[\delta\phi]_\times) p_i + (R \delta t + t) - q_i
$$

展开括号：

$$
r_i' = Rp_i + R[\delta\phi]_\times p_i + R\delta t + t - q_i
$$

将前三项和后两项重新分组：

$$
r_i' = \underbrace{(Rp_i + t - q_i)}_{\text{原始残差 } r_i} + \underbrace{R\delta t}_{\text{平移微调的影响}} + \underbrace{R[\delta\phi]_\times p_i}_{\text{旋转微调的影响}}
$$

**Step 3：化简旋转项——反对称矩阵的性质。**

$R [\delta\phi]_\times p_i$ 的含义是"旋转微调 $\delta\phi$ 对残差的影响"。要把它写成 $(\text{某个矩阵}) \cdot \delta\phi$ 的形式，才能提取雅可比。

反对称矩阵有一个关键性质——**它乘以向量等于叉积**：

$$
[a]_\times b = a \times b
$$

所以 $[\delta\phi]_\times p_i = \delta\phi \times p_i$。

再利用叉积的**反交换律** $a \times b = -(b \times a) = -[b]_\times a$：

$$
\delta\phi \times p_i = -(p_i \times \delta\phi) = -[p_i]_\times \delta\phi
$$

代入：

$$
R[\delta\phi]_\times p_i = R \cdot (-[p_i]_\times \delta\phi) = -R[p_i]_\times \delta\phi
$$

现在旋转项也写成了"矩阵乘以 $\delta\phi$"的形式。

**Step 4：合并，提取雅可比。**

$$
r_i' - r_i = R \cdot \delta t + (-R [p_i]_\times) \cdot \delta\phi
$$

写成统一的矩阵形式（$\delta\xi = [\delta t^T, \delta\phi^T]^T$）：

$$
r_i' - r_i = \begin{bmatrix} R & -R [p_i]_\times \end{bmatrix} \begin{pmatrix} \delta t \\ \delta\phi \end{pmatrix} = J_i \cdot \delta\xi
$$

所以**右扰动雅可比**：

$$
\boxed{J_i^{\text{右}} = \begin{bmatrix} R & -R [p_i]_\times \end{bmatrix} \in \mathbb{R}^{3 \times 6}}
$$

| 列 | 对应输入 | 雅可比列内容 | 含义 |
|---|---------|------------|------|
| 1-3 | $\delta t$（平移） | $R$ | 机体系平移经 $R$ 转到世界系 |
| 4-6 | $\delta\phi$（旋转） | $-R[p_i]_\times$ | 点 $p_i$ 被旋转后经 $R$ 转到世界系 |

其中 $[p_i]_\times$ 是 $p_i$（**传感器坐标系**下的点）的反对称矩阵：

$$
[p_i]_\times = \begin{bmatrix} 0 & -p_{iz} & p_{iy} \\ p_{iz} & 0 & -p_{ix} \\ -p_{iy} & p_{ix} & 0 \end{bmatrix}
$$

#### 4.5.3 左扰动推导——更简洁的结果

左扰动 $T' = \text{Exp}(\delta\xi) \cdot T$，同样展开 $4 \times 4$ 矩阵乘法：

$$
T' = \begin{pmatrix} I + [\delta\phi]_\times & \delta t \\ 0 & 1 \end{pmatrix} \begin{pmatrix} R & t \\ 0 & 1 \end{pmatrix} = \begin{pmatrix} (I+[\delta\phi]_\times)R & (I+[\delta\phi]_\times)t + \delta t \\ 0 & 1 \end{pmatrix}
$$

- 新旋转：$R' = (I + [\delta\phi]_\times) R = R + [\delta\phi]_\times R$
- 新平移：$t' = t + [\delta\phi]_\times t + \delta t$

**注意平移项的差异**：左扰动的 $\delta t$ 直接在世界系下生效——不需要乘 $R$。这是因为左扰动 $\text{Exp}(\delta\xi)$ 本身就在世界坐标系下施加。

代入残差：

$$
r_i' = R' p_i + t' - q_i = (R + [\delta\phi]_\times R) p_i + t + [\delta\phi]_\times t + \delta t - q_i
$$

$$
= \underbrace{(Rp_i + t - q_i)}_{r_i} + \delta t + [\delta\phi]_\times (Rp_i + t)
$$

令 $\hat{p}_i = Rp_i + t$（世界坐标系下变换后的点），利用 $[\delta\phi]_\times \hat{p}_i = -[\hat{p}_i]_\times \delta\phi$：

$$
r_i' - r_i = \delta t - [\hat{p}_i]_\times \delta\phi
$$

$$
= \begin{bmatrix} I_3 & -[\hat{p}_i]_\times \end{bmatrix} \begin{pmatrix} \delta t \\ \delta\phi \end{pmatrix}
$$

**左扰动雅可比**：

$$
\boxed{J_i^{\text{左}} = \begin{bmatrix} I_3 & -[\hat{p}_i]_\times \end{bmatrix} \in \mathbb{R}^{3 \times 6}}
$$

| 列 | 对应输入 | 雅可比列内容 | 含义 |
|---|---------|------------|------|
| 1-3 | $\delta t$（平移） | $I_3$（单位矩阵） | 世界系平移直接作用（1:1） |
| 4-6 | $\delta\phi$（旋转） | $-[\hat{p}_i]_\times$ | $\hat{p}_i$ 离旋转轴越远，影响越大 |

**比右扰动简洁得多**——平移部分是单位矩阵（无需乘 $R$），旋转部分直接用世界系下的点。

#### 4.5.4 两种结果的对比与本项目的选择

| | 右扰动 | 左扰动 |
|---|--------|--------|
| 雅可比 | $[R \mid -R[p_i]_\times]$ | $[I_3 \mid -[\hat{p}_i]_\times]$ |
| 平移列 | $R$（需要乘旋转矩阵） | $I_3$（单位矩阵，更简洁） |
| 旋转列中的点 | $p_i$（传感器坐标系） | $\hat{p}_i = Rp_i + t$（世界坐标系） |
| 位姿更新 | $T \leftarrow T \cdot \text{Exp}(\delta\xi)$ | $T \leftarrow \text{Exp}(\delta\xi) \cdot T$ |

> **⚠️ 本项目代码使用左扰动**（和 KISS-ICP 一致）。左扰动的雅可比不含 $R$ 因子，每个匹配点只需提取 $\hat{p}$ 的三个分量即可填写全部 18 个雅可比元素。

展开左扰动雅可比的每一行，对照 VoxelHashTarget::match() 中的代码：

```
        δtx  δty  δtz  δφx      δφy      δφz
  rx: [  1    0    0    0        p̂z      -p̂y  ]
  ry: [  0    1    0   -p̂z       0        p̂x  ]
  rz: [  0    0    1    p̂y      -p̂x       0   ]
```

其中 $\hat{p} = Rp + t$ 是世界坐标系下的变换后的点，对应代码中的 `(px, py, pz) = (p_world.x(), p_world.y(), p_world.z())`。

### 4.6 MatchResult 的两种使用模式

上面推导了雅可比的数学形式。接下来看它如何落地到代码的数据结构中。

MatchResult 的 `residuals` 和 `jacobians` 都是 `vector<double>`，**按行存储**——每行一个标量残差 + 一行 6 列雅可比。`num_valid` 是**总行数**（不是匹配点数）。

两种模式：

| 模式 | 每个匹配写入 | num_valid 增量 | 用于 |
|------|------------|---------------|------|
| 3D 残差 | 3 个残差 + 3×6=18 个雅可比 | +3 | **点到点 ICP（本项目使用）** |
| 标量残差 | 1 个残差 + 1×6=6 个雅可比 | +1 | 点到面 ICP |

IcpSolver 不关心哪种模式——它逐行遍历 `num_valid` 行，每行取 1 个残差 + 6 个雅可比，统一处理。

**标量残差模式**：将 3D 残差 $r_i \in \mathbb{R}^3$ 转为标量 $e_i = \|r_i\|$：

$$
e_i = \sqrt{r_i^T r_i}
$$

$$
\frac{\partial e_i}{\partial \delta\xi} = \frac{r_i^T}{\|r_i\|} \cdot J_i \in \mathbb{R}^{1 \times 6}
$$

**但要注意**：当 $\|r_i\| \to 0$ 时分母趋零，雅可比发散。实际实现中可以：
- 使用 $e_i^2 = r_i^T r_i$ 作为残差（平方范数），雅可比变为 $2 r_i^T J_i$，无除零问题
- 或直接用 3D 残差构建 $3N \times 6$ 的大雅可比矩阵

**KISS-ICP 的做法**：直接使用 3D 残差 $r_i \in \mathbb{R}^3$，雅可比 $J_i \in \mathbb{R}^{3 \times 6}$，法方程为 $6 \times 6$：

$$
H = \sum_i w_i J_i^T J_i \in \mathbb{R}^{6 \times 6}, \quad b = \sum_i w_i J_i^T r_i \in \mathbb{R}^{6 \times 1}
$$

$$
H \delta\xi = -b
$$

其中 $w_i$ 是鲁棒核权重（下一节详述）。

### 4.7 完整 Gauss-Newton 迭代

```
输入：源点 {pᵢ}, 目标点 {qᵢ}, 初始位姿 T₀, 最大迭代 max_iter, 收敛阈值 ε
输出：优化后的位姿 T

T = T₀
for k = 0, 1, ..., max_iter-1:
    // Step 1: 变换源点
    for each pᵢ: p̂ᵢ = T ⊕ pᵢ = R·pᵢ + t

    // Step 2: 找对应（最近邻搜索）
    for each p̂ᵢ: qᵢ = nearestNeighbor(p̂ᵢ, map)
    过滤距离 > 阈值的对应

    // Step 3: 构建法方程
    H = 0₆ₓ₆,  b = 0₆ₓ₁
    for each valid (pᵢ, qᵢ):
        rᵢ = p̂ᵢ - qᵢ                          // 3×1 残差
        Jᵢ = [I₃ | -[p̂ᵢ]×]                     // 3×6 雅可比（左扰动简化形式）
        wᵢ = robust_weight(‖rᵢ‖²)              // 鲁棒核权重
        H += wᵢ · Jᵢᵀ Jᵢ
        b += wᵢ · Jᵢᵀ rᵢ

    // Step 4: 解 6×6 线性系统
    δξ = -H.ldlt().solve(b)                     // LDLT 分解

    // Step 5: 更新位姿
    T = Exp(δξ) · T                              // 左更新

    // Step 6: 收敛检查
    if ‖δξ‖ < ε: break

return T
```

### 4.8 C++ 实现引导

以上从数学推导到伪代码，完整走了一遍 Gauss-Newton。现在看它在 C++ 中是什么样——你会发现代码比数学短得多，因为 Eigen 封装了矩阵运算。

**你的任务**：实现 `IcpSolver` 类，核心方法是 `solveOneStep`。

**代码骨架**：

```cpp
#include <Eigen/Dense>
#include <manif/SE3.h>

struct IcpConfig {
    int max_iterations = 20;
    double convergence_threshold = 1e-4;
};

class IcpSolver final {
public:
    explicit IcpSolver(const IcpConfig& config = {}) : config_(config) {}

    /// 对当前 MatchResult 求解一步位姿增量
    /// @param result 由 VoxelHashTarget::match() 填充的残差和雅可比
    /// @return 6D 扭量更新 δξ
    Eigen::Matrix<double, 6, 1> solveOneStep(const MatchResult& result) {
        // TODO: 你来实现
        // 1. 累加 JᵀJ 和 Jᵀr
        // 2. LDLT 求解
        // 3. 返回 δξ
    }

    /// 完整 ICP 迭代（外层循环调用 match + solve 交替）
    /// 注意：此方法需要 target 的引用，用于每轮重新 match
    /// 但在 SimpleSLAM 架构中，match 和 solve 被分离——
    /// match 在 VoxelHashTarget 内，solve 在这里。
    /// 所以 LoIcpOdometry 负责编排外层循环。

private:
    IcpConfig config_;
};
```

**关键实现提示**：

```cpp
// ── 累加法方程的高效写法 ──

// ❌ 低效：逐元素构造 3×6 雅可比矩阵
Eigen::Matrix<double, 3, 6> J;
J.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
J.block<3,3>(0,3) = -hat(p_transformed);  // 每次构造 hat 矩阵开销大

// ✅ 高效：直接用向量外积累加 JᵀJ
// 展开 JᵀJ 的结构，发现它由 I, [p]×, [p]×ᵀ[p]× 的块组成
// 对 6×6 对称矩阵，只需累加上三角的 21 个元素

// ✅ 另一种高效写法：使用 Eigen::Map 直接映射 MatchResult 的 jacobians
Eigen::Map<const Eigen::Matrix<double, 1, 6>> Ji(
    result.jacobians.data() + i * 6);
```

### 4.9 ⚠️ 常见陷阱

**⚠️ 编程陷阱：LDLT 和 LLT 的选择**

```cpp
// ❌ 使用 LLT（Cholesky）——要求严格正定
auto dx = JtJ.llt().solve(-Jtr);
// 当 JtJ 半正定（退化场景，如走廊）时 LLT 分解失败
// Eigen 不报错，静默返回垃圾值！

// ✅ 使用 LDLT——允许半正定
auto dx = JtJ.ldlt().solve(-Jtr);
// LDLT 能处理半正定矩阵，退化方向的更新量自动为零

// ✅ 更安全的做法：检查分解是否成功
Eigen::LDLT<Eigen::Matrix<double,6,6>> ldlt(JtJ);
if (ldlt.info() != Eigen::Success) {
    return Eigen::Matrix<double,6,1>::Zero();  // 分解失败，不更新
}
auto dx = ldlt.solve(-Jtr);
```

**💡 概念误区：混淆左扰动和右扰动**

这个误区在 §4.4.2-4.4.3 已详细讲解。核心规则重复一次：**推导和更新必须使用同一种约定**。混用不会报错，但收敛方向错误。

**🧠 思维陷阱：认为 Gauss-Newton 总是收敛的**

实际上：Gauss-Newton 假设目标函数在当前点附近**近似二次**。当残差很大（远离最优解）时，二次近似很差，更新步长可能过大导致发散。

应对策略：
- **Levenberg-Marquardt**：加阻尼 $(\mathbf{J}^T\mathbf{J} + \lambda I) \delta\xi = -\mathbf{J}^T r$，大 $\lambda$ 退化为梯度下降（安全但慢），小 $\lambda$ 回到 GN（快但可能发散）
- **KISS-ICP 的做法**：用恒速模型提供好的初始猜测，使得残差始终很小（近似二次成立），不需要 LM

### 4.10 练习

1. **推导题**：用**右扰动**推导点到点残差 $r_i = Rp_i + t - q_i$ 的雅可比。验证结果为 $J_i = [R \mid -R[p_i]_\times]$。与左扰动版本 $[I_3 \mid -[\hat{p}_i]_\times]$ 对比，证明两者通过 $R$ 相关联。

2. **编程题**：实现 `solveOneStep` 函数。用 SVD 解（上一节）的测试数据验证：对相同的匹配点对，GN 单次迭代的结果应该与 SVD 解非常接近（因为在已知对应下 GN 一步就能到最优）。

3. **数值实验**：构造一个退化场景——所有点都在一个平面 $z = 0$ 上。打印 $J^T J$ 的 6 个特征值。哪些方向退化了？与你的几何直觉一致吗？（提示：$z$ 方向平移和绕 $x, y$ 轴旋转应该退化）

---

## 5. 点到面 ICP ⭐⭐⭐

§3-4 介绍的都是**点到点**残差 $r = \hat{p} - q$。本节介绍另一种残差定义——**点到面**，它利用了目标表面的法向量信息。v1.0 使用点到点（和 KISS-ICP 一致），但理解点到面对 v1.5（IkdTreeTarget、IVoxTarget）至关重要。

### 5.1 动机：利用表面几何加速收敛

点到点 ICP 惩罚源点到目标点的**欧氏距离**。但在光滑表面上，源点沿表面滑动不改变真实几何关系——只有**法向量方向**的位移才真正度量配准误差。

```
        ← 目标表面 →
   q ────────•──────── q'
              |  ↑ nᵢ（法向量）
              |  |
          p̂ •  正确的误差 = (p̂ - q) · nᵢ
              \
               \ ← 错误的误差 = ‖p̂ - q‖（欧氏距离）
```

点到面残差：$e_i = n_i^T (R p_i + t - q_i)$，是**标量**。

Chen & Medioni（1991）证明：在光滑表面附近，点到面 ICP 的收敛速度是**超线性**的（接近二次），而点到点 ICP 只有**线性**收敛。

### 5.2 残差与雅可比推导

**残差**：

$$
e_i = n_i^T (\hat{p}_i - q_i) \in \mathbb{R}
$$

其中 $\hat{p}_i = R p_i + t$，$n_i$ 是 $q_i$ 处的表面法向量。

**雅可比**（左扰动）：

$$
e_i' = n_i^T \left( (I + [\delta\phi]_\times) \hat{p}_i + \delta t - q_i \right)
$$

$$
= n_i^T (\hat{p}_i - q_i) + n_i^T \delta t + n_i^T [\delta\phi]_\times \hat{p}_i
$$

利用 $a^T [b]_\times c = -a^T [c]_\times b$：

$$
n_i^T [\delta\phi]_\times \hat{p}_i = -n_i^T [\hat{p}_i]_\times \delta\phi
$$

因此：

$$
\frac{\partial e_i}{\partial \delta\xi} = \begin{bmatrix} n_i^T & -n_i^T [\hat{p}_i]_\times \end{bmatrix} \in \mathbb{R}^{1 \times 6}
$$

**和 MatchResult 完美匹配**：一个标量残差 + 一行 6 列雅可比，正好是 MatchResult 的格式。切换点到面只需在 `match()` 内部改残差和雅可比的计算方式，`IcpSolver::solveOneStep()` **完全不变**。

### 5.3 两种范式的全面对比

| 维度 | 点到点 | 点到面 |
|------|--------|--------|
| 残差 | $r_i = \hat{p}_i - q_i \in \mathbb{R}^3$ | $e_i = n_i^T(\hat{p}_i - q_i) \in \mathbb{R}$ |
| 雅可比维度 | $3 \times 6$ | $1 \times 6$ |
| 每个匹配对法方程的贡献 | 3 个方程 | 1 个方程 |
| 最少匹配数 | 2 对 | 6 对（6 个方程 ↔ 6 个未知数） |
| 闭式解 | 有（SVD） | 无（必须迭代） |
| 收敛速率 | 线性（$e_{k+1} \leq c \cdot e_k$） | 超线性（接近二次，$e_{k+1} \leq C \cdot e_k^2$） |
| 收敛域 | 宽 | 窄 |
| 退化场景 | 走廊中条件数好 | 走廊中沿走廊方向不可观 |
| 法向量需求 | 不需要 | **必须**——噪声法向量严重影响精度 |
| 适用场景 | 稀疏/噪声大/初始偏差大 | 稠密光滑表面/初始偏差小 |

### 5.4 ⚠️ 常见陷阱

**⚠️ 编程陷阱：法向量方向不一致**

不同的法向量估计方法可能产生朝内或朝外的法向量——同一个表面上相邻点的法向量可能方向相反（$n$ 和 $-n$）。

```cpp
// ❌ 直接用 PCA 估计的法向量
// PCA 给出的是面元的主方向，但符号不确定
// 结果：部分残差符号翻转，收敛方向混乱

// ✅ 统一法向量方向——朝传感器原点
if (normal.dot(sensor_origin - point) < 0) {
    normal = -normal;
}
```

**💡 概念误区：点到面一定比点到点好**

KISS-ICP（2023，Vizzo et al.）用大量实验证明：**工程优化后的点到点 ICP 在精度上与点到面持平甚至更好**。关键因素不是残差类型，而是对应质量（自适应阈值）和外点处理（鲁棒核）。

GenZ-ICP（2024）的分析发现：在退化走廊场景中，点到面 ICP 的 $J^T J$ 最小特征值趋零（沿走廊方向不可观），而点到点的条件数始终为 1（各方向等权）。

### 5.5 练习

1. **推导题**：用右扰动推导点到面雅可比。验证结果中 $R$ 的位置与左扰动版本的差异。

2. **思考题**：为什么点到面 ICP 在走廊中退化，而点到点不退化？从 $J^T J$ 的结构分析——走廊中所有法向量都平行于一个方向，这对 $J^T J$ 的秩意味着什么？

---

## 6. 鲁棒核函数 ⭐⭐⭐

§4 的 Gauss-Newton 法方程中有一个权重 $w_i$——前面一直默认 $w_i = 1$。本节解释为什么需要非均匀权重，以及如何从核函数导出权重。这是 ICP 从"能用"到"好用"的关键一步。

### 6.1 动机：外点是 ICP 的头号敌人

在实际 SLAM 中，即使用了最佳的最近邻搜索和距离阈值过滤，仍然会有 **10-30% 的对应是错误的**：

- 动态物体（行人、车辆）产生的点与静态地图不匹配
- 遮挡关系变化导致新暴露的表面
- 传感器噪声和多径反射

标准 L2 损失（$\rho(s) = s^2/2$）对外点的惩罚是**二次增长**的——一个距离 10m 的外点对目标函数的贡献是正常 0.1m 内点的 $10000$ 倍。少数外点就能主导整个优化方向。

### 6.2 IRLS——迭代重加权最小二乘

鲁棒核函数通过 **IRLS（Iteratively Reweighted Least Squares）** 集成到 Gauss-Newton 中：

$$
(J^T W J) \delta\xi = -J^T W r
$$

其中 $W = \text{diag}(w_1, w_2, \ldots, w_N)$，$w_i = \frac{\rho'(\|r_i\|)}{\|r_i\|}$ 是从核函数导出的权重。

**直觉**：小残差的 $w_i \approx 1$（和 L2 一样），大残差的 $w_i \to 0$（被压制）。核函数的形状决定了"什么程度的残差算大"以及"大残差被压制到什么程度"。

### 6.3 常用核函数

| 核函数 | $\rho(s)$ | 权重 $w(s) = \rho'(s)/s$ | 性质 |
|--------|-----------|--------------------------|------|
| L2 | $s^2/2$ | $1$ | 无鲁棒性 |
| Huber($k$) | $s^2/2$ if $s \leq k$; $ks - k^2/2$ otherwise | $1$ if $s \leq k$; $k/s$ otherwise | 凸；不重降 |
| Cauchy($c$) | $\frac{c^2}{2}\ln(1 + s^2/c^2)$ | $\frac{1}{1 + s^2/c^2}$ | 非凸；重降 |
| Geman-McClure($c$) | $\frac{s^2}{2(c^2 + s^2)}$ | $\frac{c^2}{(c^2 + s^2)^2}$ | 强重降；**KISS-ICP 使用** |
| Welsch($c$) | $\frac{c^2}{2}(1 - e^{-s^2/c^2})$ | $e^{-s^2/c^2}$ | 最强重降 |

**"重降"（redescending）** 意味着当残差 $s \to \infty$ 时，影响函数 $\psi(s) = \rho'(s) \to 0$——外点的贡献不仅被限制，而且趋于零。Huber 只限制不重降（远处外点仍有恒定影响），Geman-McClure 和 Welsch 让远处外点完全失去影响力。

**参数 $c$ 的意义**：控制"内点和外点的分界线"。通常取 $c \approx \sigma$（期望的内点噪声标准差）。KISS-ICP 使用自适应阈值 $\sigma$ 作为 $c$。

### 6.4 KISS-ICP 的 Geman-McClure 实现

KISS-ICP 并不直接使用 §6.3 表格中的教科书公式，而是做了**重参数化**——将核函数作用在残差的**平方范数** $\|r_i\|^2$ 上，并用 $\sigma$（自适应阈值）替代 $c^2$：

$$
w_i = \frac{\sigma^2}{(\sigma + \|r_i\|^2)^2}
$$

> **和表格的关系**：表格中 $w(s) = c^2/(c^2 + s^2)^2$ 的自变量 $s$ 是距离 $\|r_i\|$，KISS-ICP 的自变量是 $s^2 = \|r_i\|^2$，尺度参数也从 $c^2$ 换成了 $\sigma$。两者在数学上是等价的重参数化——常数因子差异不影响最优解（在法方程 $J^T W J \delta\xi = -J^T W r$ 中，$W$ 的常数缩放消掉了）。

代码非常简洁：

```cpp
auto weight = [&](double residual_sq) {
    return sigma * sigma / ((sigma + residual_sq) * (sigma + residual_sq));
};
```

其中 $\sigma$ 来自自适应阈值（§7.2 详述）。

### 6.5 ⚠️ 常见陷阱

**🧠 思维陷阱：认为鲁棒核越强越好**

新手想法："Welsch 的重降最强，外点影响最小，肯定最好。"

实际上：Babin et al.（2019）的实验表明——**正确调参时 Cauchy 和 Welsch 精度最好，错误调参时它们比 L2 还差**。重降核是非凸的，如果 $c$ 太小，大量正常内点也会被当作外点压制，导致有效匹配数不足而发散。

正确做法：
- **保守策略**：先用 Huber（凸，不会发散），确认效果后再换 Geman-McClure
- **自适应 $c$**：用残差中位绝对偏差（MAD）估计 $\sigma$，令 $c = 1.4826 \cdot \text{MAD}$（正态分布下的一致估计量）
- **KISS-ICP 的做法**：$c = \sigma$ 来自运动模型偏差的历史统计——这是一种隐式自适应

### 6.6 练习

1. **编程题**：对同一组含 20% 外点的配对数据，分别用 L2、Huber($k=1.0$)、Geman-McClure($c=1.0$) 权重做一次 Gauss-Newton。对比三种结果的旋转/平移误差。

2. **可视化题**：画出 Huber、Cauchy、Geman-McClure、Welsch 四种核函数的 $\rho(s)$、$\rho'(s)$、$w(s)$ 曲线。标注它们的行为差异（凸/非凸、重降/不重降）。

3. **思考题**：为什么 IRLS 在非凸核函数下仍能工作？（提示：IRLS 每步解的是当前权重下的**加权最小二乘**，这是凸问题；非凸性体现在权重的更新上。这是一种 MM（Majorization-Minimization）算法。）

---

## 7. 完整 ICP 管道——从原始点云到位姿 ⭐⭐

前面分别讲了体素哈希地图（§2）、SVD 求解（§3）、Gauss-Newton 优化（§4）、点到面扩展（§5）、鲁棒核（§6）。本节把所有零件组装成完整的管道，并映射到 SimpleSLAM 的三个类（VoxelHashTarget、IcpSolver、LoIcpOdometry）。

### 7.1 KISS-ICP 参考管道

以下是 KISS-ICP 的完整处理流程（Vizzo et al. 2023）：

```
输入：原始点云 + 点时间戳（可选）

Step 1: 预处理
  ├─ 运动去畸变（如果有时间戳）
  │   每个点 pᵢ 的时间偏移 sᵢ ∈ [0, 1]
  │   pᵢ* = Exp((sᵢ - 1) · log(Δ_prev)) · pᵢ
  │   用上一帧的相对运动 Δ_prev 做 SE(3) 插值
  │
  └─ 距离过滤
      移除 range < min_range 或 > max_range 的点

Step 2: 两级降采样
  ├─ 细粒度降采样 → frame_ds（voxel_size × 0.5）→ 用于更新地图
  └─ 粗粒度降采样 → source（voxel_size × 1.5）  → 用于配准查询
  原因：配准用稀疏集（快），地图用稠密集（准）

Step 3: 初始猜测（恒速模型）
  T_init = T_prev · Δ_prev
  其中 Δ_prev = T_{k-2}⁻¹ · T_{k-1}

Step 4: ICP 配准
  T_new = AlignPointsToMap(source, map, T_init, 3σ, σ)
  ├─ max_correspondence_distance = 3σ（三西格玛过滤）
  └─ kernel_scale = σ（Geman-McClure 参数）

Step 5: 更新
  ├─ 自适应阈值更新
  │   model_error = ‖trans(T_init⁻¹ · T_new)‖ + 2·max_range·sin(θ/2)
  │   σ = sqrt(Σ model_error² / n)
  │
  ├─ 地图更新
  │   将 frame_ds 用 T_new 变换后插入体素哈希地图
  │   移除距当前位置 > max_distance 的体素（滑窗）
  │
  └─ 运动模型更新
      Δ_new = T_prev⁻¹ · T_new

输出：T_new（全局位姿）
```

### 7.2 自适应阈值的数学

为什么不用固定阈值？因为最佳阈值取决于**运动预测的准确度**——车缓慢行驶时恒速模型很准（阈值可以很小），急刹车/急转弯时模型偏差大（阈值需要放大）。

KISS-ICP 的自适应方案本质是**在线估计恒速模型的预测误差标准差**：

$$
\sigma = \sqrt{\frac{1}{n} \sum_{k} e_k^2}
$$

其中 $e_k$ 是第 $k$ 帧的模型偏差。偏差包含两部分：

| 分量 | 公式 | 含义 |
|------|------|------|
| 平移偏差 | $\delta_t = \|t_{\text{predict}} - t_{\text{actual}}\|$ | 位置预测误差 |
| 旋转偏差 | $\delta_r = 2 \cdot r_{\max} \cdot \sin(\theta/2)$ | 旋转导致的最大点位移 |

旋转偏差的公式 $2 r_{\max} \sin(\theta/2)$ 是一个巧妙的几何界——距传感器 $r_{\max}$ 处的点，因旋转 $\theta$ 产生的弧长位移的精确值（弦长公式）。

### 7.3 收敛判据

```cpp
if (dx.norm() < 1e-4) break;
```

`dx` 是 $\mathfrak{se}(3)$ 中的 6D 向量——前 3 个分量量纲是米，后 3 个是弧度。$\|dx\| < 10^{-4}$ 意味着更新量不足 0.1mm 和 0.006°，已经在传感器噪声以下。

### 7.4 在 SimpleSLAM 中的实现架构

```
LoIcpOdometry (继承 OdometryBase)
  │
  ├── VoxelHashTarget target_     ← 满足 RegistrationTarget concept
  │     .match(scan, pose, result)  → 填充残差+雅可比
  │     .update(scan, pose)         → 向地图插入新点
  │     .empty() / .size()
  │
  ├── IcpSolver solver_            ← 从 MatchResult 求解 δξ
  │     .solveOneStep(result) → δξ
  │
  ├── KeyframeSelector kf_sel_     ← 已有组件
  │
  └── processLidar(scan):
        1. source = voxelDownsample(scan, voxel_size * 1.5)
        2. pose = last_pose_ * delta_  // 恒速模型
        3. if target_.empty():
             target_.update(scan, pose)
             return {pose, Initializing}
        4. for iter in 0..max_iter:
             target_.match(source, pose, result_)
             dx = solver_.solveOneStep(result_)
             pose = Exp(dx) * pose
             if dx.norm() < eps: break
        5. target_.update(scan, pose)  // 用原始扫描更新（不是降采样后的）
        6. delta_ = last_pose_.inverse() * pose
        7. last_pose_ = pose
        8. if kf_sel_.shouldSelect(pose, scan.timestamp):
             kf_sel_.update(pose, scan.timestamp)
             publishKeyframe(...)
        9. publishResult({pose, Tracking, ...})
```

### 7.5 ⚠️ 常见陷阱

**⚠️ 编程陷阱：用降采样后的点云更新地图**

```cpp
// ❌ 用粗粒度降采样的 source 更新地图
target_.update(source_downsampled, pose);
// 地图越来越稀疏，配准质量逐帧退化

// ✅ 用原始扫描（或细粒度降采样）更新地图
target_.update(scan, pose);  // 或 frame_fine_ds
// KISS-ICP 用 0.5×voxel_size 的细粒度降采样更新地图
```

**🧠 思维陷阱：初始化时也用 ICP 配准**

新手想法："第一帧就开始跑 ICP。"

实际上：第一帧没有地图可配准。正确做法是第一帧直接插入地图，标记状态为 `Initializing`，从第二帧开始配准。KISS-ICP 也是如此——第一帧无条件插入。

**⚠️ 编程陷阱：忘记更新恒速模型**

```cpp
// ❌ delta_ 忘记更新
// 结果：每帧的初始猜测都是上一次有效 delta_ 的值
// 如果机器人突然加速，初始猜测偏差越来越大

// ✅ 每帧更新
delta_ = last_pose_.inverse() * pose;
last_pose_ = pose;
```

### 7.6 练习

1. **编程题（累积项目核心）**：实现 `VoxelHashTarget`，使其满足 `RegistrationTarget` concept。在 `match()` 中填充点到点残差和左扰动雅可比。用单元测试验证：构造已知变换的两组点，`match()` + `IcpSolver::solveOneStep()` 后 $\delta\xi$ 应该趋近于零变换。

2. **集成测试**：实现 `LoIcpOdometry::processLidar()`，对 KITTI 序列 00 的前 100 帧运行，用 `Trajectory::exportTum()` 导出轨迹，与 `evaluate.py` 对比 GPS 真值。目标 ATE < 5m/100帧（初始版本，不含鲁棒核时可能更大）。

3. **消融实验**：分别关闭以下组件，观察 ATE 变化：
   - 关闭降采样（用全部点配准）→ 观察速度和精度变化
   - 关闭鲁棒核（用 L2 损失）→ 观察外点影响
   - 关闭恒速模型（初始猜测用上一帧位姿而非恒速外推）→ 观察高速段效果

---

## 本章小结

| 主题 | 核心内容 | 难度 |
|------|---------|------|
| 配准问题本质 | 两帧点云的刚体变换估计，ICP 迭代框架 | ⭐ |
| 最近邻搜索 | 体素哈希 $O(1)$ 查询，27 邻域暴力扫描 | ⭐⭐ |
| SVD 闭式解 | 交叉协方差矩阵 → SVD → $R = VU^T$ | ⭐⭐ |
| Gauss-Newton on SE(3) | 扰动模型（左/右乘）、雅可比推导、法方程 | ⭐⭐⭐ |
| 点到面 ICP | 标量残差 $n^T(p-q)$、更快收敛但退化风险 | ⭐⭐⭐ |
| 鲁棒核函数 | IRLS 框架、Geman-McClure、自适应 $c$ | ⭐⭐⭐ |
| 完整管道 | 预处理 → 降采样 → 恒速预测 → ICP → 地图更新 | ⭐⭐ |

**关键收获**：

1. ICP 是局部优化器——初始猜测决定成败
2. SVD 优雅但不实用——工程中全用 Gauss-Newton
3. 鲁棒核不是可选的——没有它外点会毁掉结果
4. MatchResult 的 residuals + jacobians 设计使得**求解器和配准目标完全解耦**——换残差类型不改求解器

---

## 累积项目：本章新增模块

```
Mini-LIO 项目进度：
  Ch1 ✅ KittiSource 读取点云
  Ch2 ✅ VoxelGrid 降采样
→ Ch3 本章新增：
    ├── VoxelHashTarget（体素哈希配准地图）
    ├── IcpSolver（Gauss-Newton 求解器）
    └── LoIcpOdometry（纯 LO 里程计主循环）
  Ch4 （未来）因子图 PGO
  Ch5 （未来）回环检测
```

**本章完成后**，你应该能跑通以下命令：

```bash
# 编译
cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

# 在 KITTI 序列 00 上运行里程计
./run_slam_offline --config kitti_lo.yaml --dataset /path/to/kitti/00

# 评估轨迹精度
python3 evaluate.py --format tum --ref groundtruth.txt --est trajectory.tum
```

精度目标：KITTI 序列 00 的 ATE 达到 KISS-ICP 级别（~0.5-1.5% 相对误差）。

---

## 附录 A：实现所需 API 速查

本节汇总实现 VoxelHashTarget / IcpSolver / LoIcpOdometry 时用到的全部 API，按库分组。

### A.1 Eigen 矩阵与向量

```cpp
#include <Eigen/Core>
#include <Eigen/Dense>

// ── 类型 ──
Eigen::Vector3f v3f;                           // 3D float 向量（点坐标用）
Eigen::Vector3d v3d;                           // 3D double 向量（位姿计算用）
Eigen::Matrix3d R;                             // 3×3 double 矩阵（旋转矩阵）
Eigen::Matrix<double, 6, 6> JtJ;              // 6×6 固定大小矩阵
Eigen::Matrix<double, 6, 1> Jtr;              // 6×1 固定大小向量

// ── 初始化 ──
Eigen::Matrix3d::Zero();                       // 全零 3×3
Eigen::Matrix3d::Identity();                   // 单位矩阵 3×3
Eigen::Matrix<double,6,6>::Zero();             // 全零 6×6
Eigen::Vector3d(1.0, 2.0, 3.0);               // 直接赋值

// ── 类型转换 ──
Eigen::Vector3f point_f(1.0f, 2.0f, 3.0f);
Eigen::Vector3d point_d = point_f.cast<double>();   // float → double
Eigen::Vector3f back_f  = point_d.cast<float>();    // double → float

// ── 常用运算 ──
v3d.x(); v3d.y(); v3d.z();                    // 分量访问
v3d.norm();                                     // 欧氏范数 √(x²+y²+z²)
v3d.squaredNorm();                              // 范数平方 x²+y²+z²（比 norm() 快，省 sqrt）
(v3d - other).squaredNorm();                    // 两点距离²

// ── 矩阵块操作 ──
Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
T.block<3,3>(0,0) = R;                         // 写入左上 3×3 块
T.block<3,1>(0,3) = t;                         // 写入右上 3×1 块

// ── Eigen::Map（零拷贝映射）──
// 将 vector<double> 的连续内存当做 Eigen 矩阵/向量使用
std::vector<double> data = {1, 0, 0, 0, 0.5, -0.3};
Eigen::Map<const Eigen::Matrix<double, 1, 6>> row(data.data());  // 1×6 行向量
// row 直接指向 data 的内存，不拷贝
// 修改 data → row 同步变化（反之亦然，如果不是 const）
```

### A.2 Eigen SVD 与线性求解

```cpp
#include <Eigen/SVD>

// ── SVD 分解 ──
Eigen::Matrix3d W;  // 3×3 交叉协方差矩阵
Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
//                                        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                                        必须加！不加则 matrixU()/matrixV() 返回空矩阵
Eigen::Matrix3d U = svd.matrixU();         // 3×3 正交矩阵
Eigen::Matrix3d V = svd.matrixV();         // 3×3 正交矩阵
Eigen::Vector3d sigma = svd.singularValues();  // 奇异值（降序排列）

// ── LDLT 线性求解（推荐用于法方程）──
Eigen::Matrix<double, 6, 6> JtJ;
Eigen::Matrix<double, 6, 1> Jtr;
// 求解 JtJ * x = -Jtr
auto ldlt = JtJ.ldlt();
if (ldlt.info() == Eigen::Success) {
    Eigen::Matrix<double, 6, 1> dx = ldlt.solve(-Jtr);
}
// 为什么用 LDLT 而非 LLT：LDLT 允许半正定（退化场景），LLT 要求严格正定

// ── 特征值分解（退化检测用）──
Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eig(JtJ);
Eigen::Matrix<double, 6, 1> eigenvalues = eig.eigenvalues();  // 升序排列
// eigenvalues(0) 最小 → 最退化的方向
```

### A.3 manif SE3d 李群操作

```cpp
#include <manif/SE3.h>

// ── 类型（SimpleSLAM 中的别名）──
// SE3d = manif::SE3d     → 刚体变换（旋转 + 平移）
// SO3d = manif::SO3d     → 纯旋转

// ── 创建 ──
SE3d identity{};                               // 单位变换（无旋转无平移）
SE3d from_matrix(Eigen::Matrix4d::Identity()); // 从 4×4 矩阵构造

// ── 分量提取 ──
SE3d pose;
auto R = pose.rotation();                      // 返回 3×3 旋转矩阵（Eigen::Matrix3d）
auto t = pose.translation();                   // 返回 3×1 平移向量（Eigen::Vector3d）

// ── 变换点 ──
Eigen::Vector3d p_body(1.0, 2.0, 3.0);
Eigen::Vector3d p_world = pose.act(p_body);    // p_world = R * p_body + t
// 等价于手动写 R * p_body + t，但更简洁
// 注意：act() 只接受 Vector3d，不接受 Vector3f

// ── 位姿组合（乘法）──
SE3d T_ab, T_bc;
SE3d T_ac = T_ab * T_bc;                      // 变换链：先 bc 再 ab
// 等价于 T_ab.compose(T_bc)

// ── 逆变换 ──
SE3d T_inv = pose.inverse();                   // T * T_inv = Identity
// 如果 pose = T_world_body，那么 T_inv = T_body_world

// ── 指数映射（切向量 → 变换）──
Eigen::Matrix<double, 6, 1> twist;             // 6D 扭量 [tx, ty, tz, rx, ry, rz]
twist << 0.1, 0.0, 0.0, 0.0, 0.0, 0.01;       // 沿 x 平移 0.1m + 绕 z 旋转 0.01rad
SE3d delta = SE3d::Exp(twist);                 // 切向量 → SE3d 变换

// ── 对数映射（变换 → 切向量）──
auto log_vec = pose.log();                     // SE3d → 6D 切向量
Eigen::Matrix<double, 6, 1> xi = log_vec.coeffs();  // 取出数值

// ── 左扰动更新（本项目使用）──
SE3d updated = SE3d::Exp(dx) * pose;           // Exp(δξ) · T → 左乘更新
// 等价于 pose.lplus(manif::SE3Tangentd(dx))

// ── 右扰动更新（GTSAM 风格，本项目不用）──
SE3d updated_r = pose * SE3d::Exp(dx);         // T · Exp(δξ) → 右乘更新
// 等价于 pose.rplus(manif::SE3Tangentd(dx))

// ── 相对变换（恒速模型核心）──
SE3d T_prev, T_curr;
SE3d delta = T_prev.inverse() * T_curr;        // T_prev→T_curr 的相对运动
SE3d T_predict = T_curr * delta;               // 恒速模型预测下一帧位姿
```

### A.4 tsl::robin_map 哈希表

```cpp
#include <tsl/robin_map.h>

// ── 类型（VoxelHashTarget 中的用法）──
tsl::robin_map<VoxelCoord, std::vector<Eigen::Vector3d>, VoxelHash> map_;
//              Key 类型      Value 类型                    哈希函数

// ── 插入/访问 ──
VoxelCoord voxel{3, 1, -1};
map_[voxel].push_back(point);                  // 不存在时自动创建空 vector
auto& bucket = map_[voxel];                    // 引用——避免重复哈希查找

// ── 查找（只读，不创建空桶）──
auto it = map_.find(voxel);
if (it != map_.end()) {
    for (const auto& p : it->second) {         // 遍历该体素内的点
        // ...
    }
}

// ── 遍历全部 ──
for (const auto& [voxel_coord, points] : map_) {
    // C++17 结构化绑定
}

// ── 删除 ──
map_.erase(voxel);                             // 按 key 删除
// 遍历中删除（滑窗地图）：
for (auto it = map_.begin(); it != map_.end();) {
    if (should_remove(it->second)) {
        it = map_.erase(it);                   // erase 返回下一个有效迭代器
    } else {
        ++it;
    }
}

// ── 状态查询 ──
map_.empty();                                  // 是否为空
map_.size();                                   // 有多少个体素
map_.clear();                                  // 清空全部
```

### A.5 SimpleSLAM 已有工具

```cpp
// ── 体素坐标（core/math/voxel_grid.hpp）──
#include <SimpleSLAM/core/math/voxel_grid.hpp>

VoxelCoord v{3, 1, -1};                        // 三维整数体素坐标
VoxelCoord v2 = toVoxelCoord(point_3f, voxel_size);  // Vector3f → 体素坐标
// 内部用 floor()，负坐标正确处理

// ── 点云降采样（core/math/point_ops.hpp）──
#include <SimpleSLAM/core/math/point_ops.hpp>

LidarScan source = voxelDownsample(scan, 1.5f);       // 体素降采样
LidarScan filtered = rangeFilter(scan, 1.0f, 100.0f); // 距离过滤

// ── MatchResult（core/concepts/registration_target.hpp）──
MatchResult result;
result.reserve(scan.size() * 3);               // 预分配（点到点：每点 3 行）
result.clear();                                // 每次 match 前清空
result.residuals.push_back(rx);                // 追加残差
result.jacobians.push_back(j_val);             // 追加雅可比元素
result.num_valid += 3;                         // 点到点每匹配 +3 行

// ── KeyframeSelector（odometry/keyframe_selector.hpp）──
KeyframeSelector kf_sel(KeyframeCriteria{.min_distance = 1.0, .min_angle_deg = 10.0});
if (kf_sel.shouldSelect(pose, timestamp)) {
    kf_sel.update(pose, timestamp);
    // 发布关键帧...
}

// ── OdometryBase 基类提供的方法（继承后可用）──
publishResult(odom_result);                    // 发布位姿到 Topic + 触发回调
publishKeyframe(keyframe_data);                // 发布关键帧到 Topic + 触发回调
log_->info("Processing frame {}", frame_id);   // 日志（spdlog，自动带里程计名前缀）
```

### A.6 常用代码模式速查

```cpp
// ── 模式 1：将传感器坐标系的点变换到世界坐标系 ──
auto R = pose.rotation();
auto t = pose.translation();
for (const auto& point : scan.points) {
    Eigen::Vector3d p_world = R * point.cast<double>() + t;
    // ... 使用 p_world
}
// 为什么不用 pose.act()？因为 act() 每次调用都要提取 R 和 t，
// 循环外提取一次更快。

// ── 模式 2：Eigen::Map 逐行读取 MatchResult ──
for (int i = 0; i < result.num_valid; ++i) {
    double ri = result.residuals[i];
    Eigen::Map<const Eigen::Matrix<double, 1, 6>> Ji(
        result.jacobians.data() + i * 6);
    // Ji 是 1×6 行向量，直接参与矩阵运算
    JtJ += Ji.transpose() * Ji;                // 6×6 += 6×1 * 1×6
    Jtr += Ji.transpose() * ri;                // 6×1 += 6×1 * scalar
}

// ── 模式 3：恒速模型 ──
SE3d delta = last_pose_.inverse() * current_pose;   // 本帧相对运动
SE3d next_predict = current_pose * delta;            // 预测下一帧位姿
// 假设：机器人下一帧的运动和本帧一样

// ── 模式 4：ICP 左扰动更新循环 ──
SE3d pose = initial_guess;
for (int iter = 0; iter < max_iter; ++iter) {
    result.clear();
    target.match(scan, pose, result);
    auto dx = solver.solveOneStep(result);
    pose = SE3d::Exp(dx) * pose;                    // 左乘更新
    if (dx.norm() < 1e-4) break;
}
```

---

## 延伸阅读

| 资料 | 难度 | 说明 |
|------|------|------|
| Besl & McKay, "A Method for Registration of 3-D Shapes", IEEE TPAMI 1992 | ⭐⭐ | ICP 奠基论文，必读 |
| Arun et al., "Least-Squares Fitting of Two 3-D Point Sets", IEEE TPAMI 1987 | ⭐⭐ | SVD 闭式解原文 |
| Rusinkiewicz & Levoy, "Efficient Variants of the ICP Algorithm", 3DIM 2001 | ⭐⭐⭐ | ICP 变体六维度分类法，理解 ICP 设计空间的最佳入门 |
| Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP", IEEE RA-L 2023 | ⭐⭐ | 现代工程化 ICP 的标杆，代码极其简洁可读 |
| Segal et al., "Generalized-ICP", RSS 2009 | ⭐⭐⭐ | 统一 p2p/p2pl/GICP 的理论框架 |
| Babin et al., "Analysis of Robust Functions for Registration", IROS 2019 | ⭐⭐⭐ | 鲁棒核函数在配准中的系统实验对比 |
| GenZ-ICP, arXiv:2411.06766, 2024 | ⭐⭐⭐⭐ | 退化感知 ICP，条件数驱动的 p2p/p2pl 自适应混合 |
| Rusinkiewicz, "A Symmetric Objective Function for ICP", SIGGRAPH 2019 | ⭐⭐⭐⭐ | 对称 ICP——二次收敛速率，研究前沿 |
| PRBonn/kiss-icp GitHub 源码 | ⭐⭐ | C++ 实现参考，约 500 行核心代码 |
| Nghia Ho, "Finding Optimal R and t using SVD" (博客) | ⭐ | SVD 方法的简洁图文教程 |
| John Lambert, "ICP Tutorial" (博客) | ⭐⭐ | 含 Python 代码的教学推导 |

---

> **下一章预告**：Ch4 将在 ICP 里程计的基础上加入 IMU 预积分和迭代误差状态卡尔曼滤波（IEKF），升级为 LIO 系统——从 `LoIcpOdometry` 进化到 `LioIekfOdometry`。
