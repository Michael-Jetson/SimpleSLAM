#ifndef MODULE_GRAPH_H
#define MODULE_GRAPH_H

#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include <QtGui/QPainter>
#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QFont>
#include <QtGui/QFontMetrics>
#include <QtGui/QMouseEvent>
#include <QtGui/QPaintEvent>
#include <QtGui/QPixmap>
#include <QtCore/QPoint>
#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtCore/QMap>
#include <QtCore/QSize>
#include <QtCore/QRect>
#include <QtCore/QCoreApplication>
#include <QtGui/QColor>

namespace module_graph {

// 前向声明
class ModuleGraph;

/**
 * @brief 节点形状枚举
 */
enum class NodeShape {
    Rectangle,  // 矩形
    Ellipse     // 椭圆
};

/**
 * @brief 图形节点类
 */
class Node {
public:
    Node(const std::string& id, const std::string& text, const QPoint& position, 
         NodeShape shape = NodeShape::Rectangle, const QColor& color = Qt::white);
    
    // Getter方法
    const std::string& getId() const { return id_; }
    const std::string& getText() const { return text_; }
    const QPoint& getPosition() const { return position_; }
    NodeShape getShape() const { return shape_; }
    const QColor& getColor() const { return color_; }
    const QSize& getSize() const { return size_; }
    
    // Setter方法
    void setText(const std::string& text) { text_ = text; }
    void setPosition(const QPoint& position) { position_ = position; }
    void setShape(NodeShape shape) { shape_ = shape; }
    void setColor(const QColor& color) { color_ = color; }
    void setSize(const QSize& size) { size_ = size; }
    
    // 检查点是否在节点内
    bool contains(const QPoint& point) const;
    
    // 获取节点边界矩形
    QRect getBoundingRect() const;

private:
    std::string id_;
    std::string text_;
    QPoint position_;
    NodeShape shape_;
    QColor color_;
    QSize size_;
};

/**
 * @brief 节点连接类
 */
class Link {
public:
    Link(const std::string& fromNodeId, const std::string& toNodeId, 
         const std::string& text = "", const QColor& color = Qt::black);
    
    // Getter方法
    const std::string& getFromNodeId() const { return fromNodeId_; }
    const std::string& getToNodeId() const { return toNodeId_; }
    const std::string& getText() const { return text_; }
    const QColor& getColor() const { return color_; }
    int getLineWidth() const { return lineWidth_; }
    
    // Setter方法
    void setText(const std::string& text) { text_ = text; }
    void setColor(const QColor& color) { color_ = color; }
    void setLineWidth(int width) { lineWidth_ = width; }

private:
    std::string fromNodeId_;
    std::string toNodeId_;
    std::string text_;
    QColor color_;
    int lineWidth_;
};

/**
 * @brief 图例项类
 */
class LegendItem {
public:
    LegendItem(const QColor& color, const std::string& text);
    
    const QColor& getColor() const { return color_; }
    const std::string& getText() const { return text_; }
    void setText(const std::string& text) { text_ = text; }

private:
    QColor color_;
    std::string text_;
};

/**
 * @brief 图形绘制区域类
 */
class GraphWidget : public QWidget {
    Q_OBJECT

public:
    explicit GraphWidget(QWidget* parent = nullptr);
    
    // 节点操作
    void addNode(std::shared_ptr<Node> node);
    void removeNode(const std::string& nodeId);
    std::shared_ptr<Node> getNode(const std::string& nodeId);
    
    // 连接操作
    void addLink(std::shared_ptr<Link> link);
    void removeLink(const std::string& fromNodeId, const std::string& toNodeId);
    std::shared_ptr<Link> getLink(const std::string& fromNodeId, const std::string& toNodeId);
    
    // 图例操作
    void addLegendItem(const QColor& color, const std::string& text);
    void clearLegend();
    
    // 保存图片
    void saveImage(const QString& filename);
    
    // 清空所有内容
    void clear();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    friend class ModuleGraph;  // 允许ModuleGraph访问私有成员
    
    void drawNode(QPainter& painter, std::shared_ptr<Node> node);
    void drawLink(QPainter& painter, std::shared_ptr<Link> link);
    void drawArrow(QPainter& painter, const QPoint& from, const QPoint& to, const QColor& color);
    void drawLegend(QPainter& painter);
    QPoint getNodeCenter(std::shared_ptr<Node> node);
    QPoint getConnectionPoint(std::shared_ptr<Node> node, const QPoint& targetPoint);
    
    std::vector<std::shared_ptr<Node>> nodes_;
    std::vector<std::shared_ptr<Link>> links_;
    std::vector<LegendItem> legendItems_;
    
    // 鼠标交互相关
    std::shared_ptr<Node> selectedNode_;
    bool isDragging_;
    QPoint lastMousePos_;
};

/**
 * @brief 主窗口类
 */
class GraphWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit GraphWindow(QWidget* parent = nullptr);
    
    GraphWidget* getGraphWidget() { return graphWidget_; }

private slots:
    void saveImage();

private:
    void setupUI();
    
    GraphWidget* graphWidget_;
    QPushButton* saveButton_;
};

/**
 * @brief 模块图形类 - 主要接口类
 */
class ModuleGraph {
public:
    ModuleGraph();
    ~ModuleGraph();
    
    // 节点操作
    void addNode(const std::string& id, const std::string& text, int x, int y, 
                 NodeShape shape = NodeShape::Rectangle, const QColor& color = Qt::white);
    void removeNode(const std::string& nodeId);
    void setNodeColor(const std::string& nodeId, const QColor& color);
    void setNodeText(const std::string& nodeId, const std::string& text);
    void setNodePosition(const std::string& nodeId, int x, int y);
    void setNodeShape(const std::string& nodeId, NodeShape shape);
    void setNodeSize(const std::string& nodeId, int width, int height);
    
    // 连接操作
    void addLink(const std::string& fromNodeId, const std::string& toNodeId, 
                 const std::string& text = "", const QColor& color = Qt::black);
    void removeLink(const std::string& fromNodeId, const std::string& toNodeId);
    void setLinkColor(const std::string& fromNodeId, const std::string& toNodeId, const QColor& color);
    void setLinkText(const std::string& fromNodeId, const std::string& toNodeId, const std::string& text);
    void setLinkWidth(const std::string& fromNodeId, const std::string& toNodeId, int width);
    
    // 图例操作
    void addLegend(const QColor& color, const std::string& text);
    void clearLegend();
    
    // 显示窗口
    void showWindow();
    void hideWindow();
    
    // 清空所有内容
    void clear();
    
    // 设置窗口标题
    void setWindowTitle(const std::string& title);

private:
    std::unique_ptr<QApplication> app_;
    std::unique_ptr<GraphWindow> window_;
    bool appCreated_;
    
    void ensureApplication();
};

} // namespace module_graph

#endif // MODULE_GRAPH_H 