#include "module_graph/module_graph.h"
#include <QtCore/QCoreApplication>
#include <QtGui/QPainterPath>
#include <QtWidgets/QApplication>
#include <cmath>
#include <algorithm>

namespace module_graph {

// Node类实现
Node::Node(const std::string& id, const std::string& text, const QPoint& position, 
           NodeShape shape, const QColor& color)
    : id_(id), text_(text), position_(position), shape_(shape), color_(color), size_(120, 60) {
}

bool Node::contains(const QPoint& point) const {
    QRect rect = getBoundingRect();
    if (shape_ == NodeShape::Rectangle) {
        return rect.contains(point);
    } else {
        // 椭圆形检测
        QPoint center = rect.center();
        double dx = point.x() - center.x();
        double dy = point.y() - center.y();
        double a = rect.width() / 2.0;
        double b = rect.height() / 2.0;
        return (dx*dx)/(a*a) + (dy*dy)/(b*b) <= 1.0;
    }
}

QRect Node::getBoundingRect() const {
    return QRect(position_.x() - size_.width()/2, position_.y() - size_.height()/2, 
                 size_.width(), size_.height());
}

// Link类实现
Link::Link(const std::string& fromNodeId, const std::string& toNodeId, 
           const std::string& text, const QColor& color)
    : fromNodeId_(fromNodeId), toNodeId_(toNodeId), text_(text), color_(color), lineWidth_(2) {
}

// LegendItem类实现
LegendItem::LegendItem(const QColor& color, const std::string& text)
    : color_(color), text_(text) {
}

// GraphWidget类实现
GraphWidget::GraphWidget(QWidget* parent)
    : QWidget(parent), selectedNode_(nullptr), isDragging_(false) {
    setMinimumSize(800, 600);
    setMouseTracking(true);
}

void GraphWidget::addNode(std::shared_ptr<Node> node) {
    nodes_.push_back(node);
    update();
}

void GraphWidget::removeNode(const std::string& nodeId) {
    auto it = std::find_if(nodes_.begin(), nodes_.end(),
        [&nodeId](const std::shared_ptr<Node>& node) {
            return node->getId() == nodeId;
        });
    if (it != nodes_.end()) {
        nodes_.erase(it);
        
        // 删除相关的连接
        auto linkIt = links_.begin();
        while (linkIt != links_.end()) {
            if ((*linkIt)->getFromNodeId() == nodeId || (*linkIt)->getToNodeId() == nodeId) {
                linkIt = links_.erase(linkIt);
            } else {
                ++linkIt;
            }
        }
        update();
    }
}

std::shared_ptr<Node> GraphWidget::getNode(const std::string& nodeId) {
    auto it = std::find_if(nodes_.begin(), nodes_.end(),
        [&nodeId](const std::shared_ptr<Node>& node) {
            return node->getId() == nodeId;
        });
    return (it != nodes_.end()) ? *it : nullptr;
}

void GraphWidget::addLink(std::shared_ptr<Link> link) {
    links_.push_back(link);
    update();
}

void GraphWidget::removeLink(const std::string& fromNodeId, const std::string& toNodeId) {
    auto it = std::find_if(links_.begin(), links_.end(),
        [&](const std::shared_ptr<Link>& link) {
            return link->getFromNodeId() == fromNodeId && link->getToNodeId() == toNodeId;
        });
    if (it != links_.end()) {
        links_.erase(it);
        update();
    }
}

std::shared_ptr<Link> GraphWidget::getLink(const std::string& fromNodeId, const std::string& toNodeId) {
    auto it = std::find_if(links_.begin(), links_.end(),
        [&](const std::shared_ptr<Link>& link) {
            return link->getFromNodeId() == fromNodeId && link->getToNodeId() == toNodeId;
        });
    return (it != links_.end()) ? *it : nullptr;
}

void GraphWidget::addLegendItem(const QColor& color, const std::string& text) {
    legendItems_.emplace_back(color, text);
    update();
}

void GraphWidget::clearLegend() {
    legendItems_.clear();
    update();
}

void GraphWidget::saveImage(const QString& filename) {
    QPixmap pixmap(size());
    pixmap.fill(Qt::white);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 绘制所有内容
    for (auto& link : links_) {
        drawLink(painter, link);
    }
    for (auto& node : nodes_) {
        drawNode(painter, node);
    }
    drawLegend(painter);
    
    pixmap.save(filename);
}

void GraphWidget::clear() {
    nodes_.clear();
    links_.clear();
    legendItems_.clear();
    selectedNode_ = nullptr;
    update();
}

void GraphWidget::paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 设置背景
    painter.fillRect(rect(), Qt::white);
    
    // 绘制连接
    for (auto& link : links_) {
        drawLink(painter, link);
    }
    
    // 绘制节点
    for (auto& node : nodes_) {
        drawNode(painter, node);
    }
    
    // 绘制图例
    drawLegend(painter);
}

void GraphWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        QPoint clickPos = event->pos();
        
        // 查找被点击的节点
        for (auto& node : nodes_) {
            if (node->contains(clickPos)) {
                selectedNode_ = node;
                isDragging_ = true;
                lastMousePos_ = clickPos;
                break;
            }
        }
    }
}

void GraphWidget::mouseMoveEvent(QMouseEvent* event) {
    if (isDragging_ && selectedNode_) {
        QPoint delta = event->pos() - lastMousePos_;
        QPoint newPos = selectedNode_->getPosition() + delta;
        selectedNode_->setPosition(newPos);
        lastMousePos_ = event->pos();
        update();
    }
}

void GraphWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        isDragging_ = false;
        selectedNode_ = nullptr;
    }
}

void GraphWidget::drawNode(QPainter& painter, std::shared_ptr<Node> node) {
    QRect rect = node->getBoundingRect();
    
    // 设置画笔和画刷
    QPen pen(Qt::black, 2);
    QBrush brush(node->getColor());
    painter.setPen(pen);
    painter.setBrush(brush);
    
    // 绘制形状
    if (node->getShape() == NodeShape::Rectangle) {
        painter.drawRect(rect);
    } else {
        painter.drawEllipse(rect);
    }
    
    // 绘制文本
    painter.setPen(Qt::black);
    painter.setFont(QFont("Arial", 10));
    QString text = QString::fromStdString(node->getText());
    painter.drawText(rect, Qt::AlignCenter | Qt::TextWordWrap, text);
}

void GraphWidget::drawLink(QPainter& painter, std::shared_ptr<Link> link) {
    auto fromNode = getNode(link->getFromNodeId());
    auto toNode = getNode(link->getToNodeId());
    
    if (!fromNode || !toNode) return;
    
    QPoint fromCenter = getNodeCenter(fromNode);
    QPoint toCenter = getNodeCenter(toNode);
    
    // 计算连接点，自动为箭头预留空间
    QPoint fromPoint = getConnectionPoint(fromNode, toCenter);
    QPoint toPoint = getConnectionPoint(toNode, fromCenter);
    
    // 绘制线条
    QPen pen(link->getColor(), link->getLineWidth());
    painter.setPen(pen);
    painter.drawLine(fromPoint, toPoint);
    
    // 绘制箭头
    drawArrow(painter, fromPoint, toPoint, link->getColor());
    
    // 绘制文本
    if (!link->getText().empty()) {
        QPoint midPoint = (fromPoint + toPoint) / 2;
        painter.setPen(Qt::black);
        painter.setFont(QFont("Arial", 9));
        QString text = QString::fromStdString(link->getText());
        QFontMetrics fm(painter.font());
        QRect textRect = fm.boundingRect(text);
        textRect.moveCenter(midPoint);
        
        // 绘制文本背景
        painter.fillRect(textRect.adjusted(-2, -2, 2, 2), QColor(255, 255, 255, 200));
        painter.drawText(textRect, Qt::AlignCenter, text);
    }
}

void GraphWidget::drawArrow(QPainter& painter, const QPoint& from, const QPoint& to, const QColor& color) {
    QPen pen(color, 2);
    painter.setPen(pen);
    painter.setBrush(QBrush(color));
    
    // 计算箭头
    double angle = std::atan2((to.y() - from.y()), (to.x() - from.x()));
    int arrowLength = 15;
    double arrowAngle = M_PI / 6; // 30度
    
    QPoint arrowP1(to.x() - arrowLength * std::cos(angle - arrowAngle),
                   to.y() - arrowLength * std::sin(angle - arrowAngle));
    QPoint arrowP2(to.x() - arrowLength * std::cos(angle + arrowAngle),
                   to.y() - arrowLength * std::sin(angle + arrowAngle));
    
    QPolygon arrowHead;
    arrowHead << to << arrowP1 << arrowP2;
    painter.drawPolygon(arrowHead);
}

void GraphWidget::drawLegend(QPainter& painter) {
    if (legendItems_.empty()) return;
    
    int legendX = width() - 200;
    int legendY = 20;
    int itemHeight = 25;
    
    // 绘制图例背景 - 改为白色背景
    QRect legendRect(legendX - 10, legendY - 10, 180, legendItems_.size() * itemHeight + 20);
    painter.fillRect(legendRect, QColor(255, 255, 255, 240)); // 白色背景，略透明
    painter.setPen(QPen(Qt::black, 1));
    painter.drawRect(legendRect);
    
    // 绘制图例项
    painter.setFont(QFont("Arial", 9));
    for (size_t i = 0; i < legendItems_.size(); ++i) {
        int y = legendY + i * itemHeight;
        
        // 绘制颜色块 - 确保正确填充颜色
        QRect colorRect(legendX, y, 15, 15);
        
        // 先保存当前的画笔和画刷状态
        QPen oldPen = painter.pen();
        QBrush oldBrush = painter.brush();
        
        // 设置画刷为指定颜色，画笔为黑色边框
        painter.setBrush(QBrush(legendItems_[i].getColor()));
        painter.setPen(QPen(Qt::black, 1));
        
        // 绘制带颜色填充的矩形
        painter.drawRect(colorRect);
        
        // 恢复原来的画笔和画刷
        painter.setPen(oldPen);
        painter.setBrush(oldBrush);
        
        // 绘制文本
        painter.setPen(Qt::black);
        QString text = QString::fromStdString(legendItems_[i].getText());
        painter.drawText(legendX + 25, y + 12, text);
    }
}

QPoint GraphWidget::getNodeCenter(std::shared_ptr<Node> node) {
    return node->getPosition();
}

QPoint GraphWidget::getConnectionPoint(std::shared_ptr<Node> node, const QPoint& targetPoint) {
    QPoint center = node->getPosition();
    QRect rect = node->getBoundingRect();
    
    // 计算从目标点指向节点中心的方向（这是箭头的方向）
    QPoint direction = center - targetPoint;
    if (direction.manhattanLength() == 0) return center;
    
    // 归一化方向向量
    double length = std::sqrt(direction.x() * direction.x() + direction.y() * direction.y());
    double dx = direction.x() / length;
    double dy = direction.y() / length;
    
    // 箭头长度（与drawArrow函数保持一致）
    double arrowLength = 15.0;
    
    // 根据节点形状计算智能偏移量
    double offsetDistance = arrowLength;
    
    if (node->getShape() == NodeShape::Rectangle) {
        // 矩形：根据连接角度确定偏移量
        double w = rect.width() / 2.0;
        double h = rect.height() / 2.0;
        
        // 计算连接线与水平轴的夹角
        double angle = std::abs(std::atan2(std::abs(dy), std::abs(dx)));
        
        // 根据角度混合宽度和高度的偏移量
        // 水平方向（角度接近0）：使用宽度偏移
        // 垂直方向（角度接近π/2）：使用高度偏移
        double horizontalWeight = std::cos(angle);
        double verticalWeight = std::sin(angle);
        offsetDistance = horizontalWeight * w + verticalWeight * h + arrowLength;
        
    } else {
        // 椭圆：根据连接角度计算椭圆上的偏移量
        double a = rect.width() / 2.0;  // 长半轴
        double b = rect.height() / 2.0; // 短半轴
        
        // 计算连接线在椭圆上的距离
        double ellipseDistance = (a * b) / std::sqrt(b * b * dx * dx + a * a * dy * dy);
        offsetDistance = ellipseDistance + arrowLength;
    }
    
    // 计算最终的连接点（在节点边缘内侧，为箭头预留空间）
    QPoint connectionPoint = QPoint(center.x() - offsetDistance * dx, center.y() - offsetDistance * dy);
    
    return connectionPoint;
}

// GraphWindow类实现
GraphWindow::GraphWindow(QWidget* parent)
    : QMainWindow(parent) {
    setupUI();
    setWindowTitle("Module Graph");
    resize(1000, 700);
}

void GraphWindow::setupUI() {
    auto centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    
    auto layout = new QVBoxLayout(centralWidget);
    
    // 创建图形绘制区域
    graphWidget_ = new GraphWidget(this);
    layout->addWidget(graphWidget_);
    
    // 创建按钮区域
    auto buttonLayout = new QHBoxLayout();
    saveButton_ = new QPushButton("保存图片", this);
    buttonLayout->addWidget(saveButton_);
    buttonLayout->addStretch();
    
    layout->addLayout(buttonLayout);
    
    // 连接信号槽
    connect(saveButton_, &QPushButton::clicked, this, &GraphWindow::saveImage);
}

void GraphWindow::saveImage() {
    QString filename = QFileDialog::getSaveFileName(this, 
        "保存图片", "", "PNG Files (*.png);;JPEG Files (*.jpg);;All Files (*)");
    
    if (!filename.isEmpty()) {
        graphWidget_->saveImage(filename);
        QMessageBox::information(this, "保存成功", "图片已保存到: " + filename);
    }
}

// ModuleGraph类实现
ModuleGraph::ModuleGraph() : appCreated_(false) {
    ensureApplication();
}

ModuleGraph::~ModuleGraph() {
    if (window_) {
        window_->close();
    }
}

void ModuleGraph::ensureApplication() {
    if (!appCreated_) {
        if (!QApplication::instance()) {
            static int argc = 1;
            static char* argv[] = {const_cast<char*>("ModuleGraph")};
            app_ = std::make_unique<QApplication>(argc, argv);
        }
        appCreated_ = true;
    }
    
    if (!window_) {
        window_ = std::make_unique<GraphWindow>();
    }
}

void ModuleGraph::addNode(const std::string& id, const std::string& text, int x, int y, 
                         NodeShape shape, const QColor& color) {
    ensureApplication();
    auto node = std::make_shared<Node>(id, text, QPoint(x, y), shape, color);
    window_->getGraphWidget()->addNode(node);
}

void ModuleGraph::removeNode(const std::string& nodeId) {
    if (window_) {
        window_->getGraphWidget()->removeNode(nodeId);
    }
}

void ModuleGraph::setNodeColor(const std::string& nodeId, const QColor& color) {
    if (window_) {
        auto node = window_->getGraphWidget()->getNode(nodeId);
        if (node) {
            node->setColor(color);
            window_->getGraphWidget()->update();
        }
    }
}

void ModuleGraph::setNodeText(const std::string& nodeId, const std::string& text) {
    if (window_) {
        auto node = window_->getGraphWidget()->getNode(nodeId);
        if (node) {
            node->setText(text);
            window_->getGraphWidget()->update();
        }
    }
}

void ModuleGraph::setNodePosition(const std::string& nodeId, int x, int y) {
    if (window_) {
        auto node = window_->getGraphWidget()->getNode(nodeId);
        if (node) {
            node->setPosition(QPoint(x, y));
            window_->getGraphWidget()->update();
        }
    }
}

void ModuleGraph::setNodeShape(const std::string& nodeId, NodeShape shape) {
    if (window_) {
        auto node = window_->getGraphWidget()->getNode(nodeId);
        if (node) {
            node->setShape(shape);
            window_->getGraphWidget()->update();
        }
    }
}

void ModuleGraph::setNodeSize(const std::string& nodeId, int width, int height) {
    if (window_) {
        auto node = window_->getGraphWidget()->getNode(nodeId);
        if (node) {
            node->setSize(QSize(width, height));
            window_->getGraphWidget()->update();
        }
    }
}

void ModuleGraph::addLink(const std::string& fromNodeId, const std::string& toNodeId, 
                         const std::string& text, const QColor& color) {
    ensureApplication();
    auto link = std::make_shared<Link>(fromNodeId, toNodeId, text, color);
    window_->getGraphWidget()->addLink(link);
}

void ModuleGraph::removeLink(const std::string& fromNodeId, const std::string& toNodeId) {
    if (window_) {
        window_->getGraphWidget()->removeLink(fromNodeId, toNodeId);
    }
}

void ModuleGraph::setLinkColor(const std::string& fromNodeId, const std::string& toNodeId, const QColor& color) {
    if (window_) {
        auto link = window_->getGraphWidget()->getLink(fromNodeId, toNodeId);
        if (link) {
            link->setColor(color);
            window_->getGraphWidget()->update();
        }
    }
}

void ModuleGraph::setLinkText(const std::string& fromNodeId, const std::string& toNodeId, const std::string& text) {
    if (window_) {
        auto link = window_->getGraphWidget()->getLink(fromNodeId, toNodeId);
        if (link) {
            link->setText(text);
            window_->getGraphWidget()->update();
        }
    }
}

void ModuleGraph::setLinkWidth(const std::string& fromNodeId, const std::string& toNodeId, int width) {
    if (window_) {
        auto link = window_->getGraphWidget()->getLink(fromNodeId, toNodeId);
        if (link) {
            link->setLineWidth(width);
            window_->getGraphWidget()->update();
        }
    }
}

void ModuleGraph::addLegend(const QColor& color, const std::string& text) {
    ensureApplication();
    window_->getGraphWidget()->addLegendItem(color, text);
}

void ModuleGraph::clearLegend() {
    if (window_) {
        window_->getGraphWidget()->clearLegend();
    }
}

void ModuleGraph::showWindow() {
    ensureApplication();
    window_->show();
    if (app_) {
        app_->exec();
    }
}

void ModuleGraph::hideWindow() {
    if (window_) {
        window_->hide();
    }
}

void ModuleGraph::clear() {
    if (window_) {
        window_->getGraphWidget()->clear();
    }
}

void ModuleGraph::setWindowTitle(const std::string& title) {
    if (window_) {
        window_->setWindowTitle(QString::fromStdString(title));
    }
}

} // namespace module_graph 
