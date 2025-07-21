# ModuleGraph 使用说明

## 简介
ModuleGraph是一个基于Qt的C++图形绘制模块，用于创建类似rqt的节点图。

## 编译和运行

### 编译
```bash
cd simple-slam
mkdir -p build && cd build
cmake ..
make graph_demo
```

### 运行
```bash
# 可执行文件位于 devel/lib/simple-slam/ 目录下（这是catkin的标准位置）
./devel/lib/simple-slam/graph_demo

# 或者从项目根目录运行：
cd ../devel/lib/simple-slam/
./graph_demo
```

## 基本用法

```cpp
#include "module_graph/module_graph.h"

int main(int argc, char* argv[]) {
    using namespace module_graph;
    
    ModuleGraph graph;
    graph.setWindowTitle("我的节点图");
    
    // 添加节点
    graph.addNode("node1", "传感器", 100, 100, NodeShape::Ellipse, QColor(Qt::lightBlue));
    graph.addNode("node2", "处理器", 300, 100, NodeShape::Rectangle, QColor(Qt::lightGreen));
    
    // 添加连接
    graph.addLink("node1", "node2", "数据流", QColor(Qt::red));
    
    // 添加图例
    graph.addLegend(QColor(Qt::lightBlue), "传感器节点");
    graph.addLegend(QColor(Qt::lightGreen), "处理节点");
    
    // 显示窗口
    graph.showWindow();
    
    return 0;
}
```

## 主要功能

1. **节点管理**：
   - 添加/删除节点
   - 设置节点形状（矩形/椭圆）
   - 自定义节点颜色和文本
   - 鼠标拖拽移动节点

2. **连接管理**：
   - 添加/删除带箭头的有向连接
   - 自定义连接颜色和文本
   - 自动计算连接点位置

3. **图例功能**：
   - 自定义颜色图例
   - 支持中文文本

4. **导出功能**：
   - 点击"保存图片"按钮导出PNG/JPEG

## 关于文件位置

**为什么可执行文件在 `devel/lib/simple-slam/` 而不是 `build/` 目录？**

这是因为项目使用了**catkin构建系统**（ROS的构建系统）：
- catkin会将可执行文件放在 `devel/lib/<package_name>/` 目录
- 这是ROS的标准做法，用于支持workspace overlay和包管理
- 这**不是错误**，而是catkin的正常行为

如果想要传统的cmake行为，可以移除CMakeLists.txt中的catkin相关内容，但这会失去ROS集成能力。

## API参考

### 节点操作
- `addNode(id, text, x, y, shape, color)` - 添加节点
- `removeNode(nodeId)` - 删除节点
- `setNodeColor/Text/Position/Shape/Size()` - 设置节点属性

### 连接操作  
- `addLink(fromId, toId, text, color)` - 添加连接
- `removeLink(fromId, toId)` - 删除连接
- `setLinkColor/Text/Width()` - 设置连接属性

### 其他操作
- `addLegend(color, text)` - 添加图例
- `showWindow()` - 显示窗口
- `setWindowTitle(title)` - 设置标题 