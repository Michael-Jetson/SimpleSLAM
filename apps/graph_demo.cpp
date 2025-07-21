#include "module_graph/module_graph.h"
#include <iostream>

int main(int argc, char* argv[]) {
    using namespace module_graph;
    
    // 创建模块图形实例
    ModuleGraph graph;
    
    // 设置窗口标题
    graph.setWindowTitle("ROS节点图示例");
    
    // 添加节点
    graph.addNode("sensor_node", "传感器节点", 100, 100, NodeShape::Ellipse, QColor(Qt::lightGray));
    graph.addNode("process_node", "处理节点", 300, 100, NodeShape::Rectangle, QColor(Qt::cyan));
    graph.addNode("output_node", "输出节点", 500, 100, NodeShape::Rectangle, QColor(Qt::green));
    graph.addNode("control_node", "控制节点", 300, 250, NodeShape::Ellipse, QColor(Qt::yellow));
    
    // 添加连接
    graph.addLink("sensor_node", "process_node", "sensor_data", QColor(Qt::blue));
    graph.addLink("process_node", "output_node", "processed_data", QColor(Qt::red));
    graph.addLink("process_node", "control_node", "control_signal", QColor(Qt::magenta));
    graph.addLink("control_node", "sensor_node", "config", QColor(Qt::darkGreen));
    
    // 添加图例
    graph.addLegend(QColor(Qt::lightGray), "传感器类节点");
    graph.addLegend(QColor(Qt::cyan), "处理类节点");
    graph.addLegend(QColor(Qt::green), "输出类节点");
    graph.addLegend(QColor(Qt::yellow), "控制类节点");
    
    // 显示窗口
    graph.showWindow();
    
    return 0;
} 