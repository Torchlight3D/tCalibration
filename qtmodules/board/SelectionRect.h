#pragma once

#include <QPainterPath>
#include <QWidget>

namespace tl {

class SelectionRect : public QWidget
{
    Q_OBJECT

public:
    enum HoverHandle
    {
        hhNone,
        hhTopLeft,
        hhTopRight,
        hhBottomLeft,
        hhBottomRight
    };

    explicit SelectionRect(QWidget *parent = nullptr);
    ~SelectionRect() override;

    void setColor(const QColor &color);
    void setPenActive(bool penActive);
    void setBrushActive(bool brushActive);
    void setResizeHandlesActive(bool resizeHandlesActive);

    HoverHandle hoverHandle(const QPoint &point);
    void setResizeHandleSize(int resizeHandleSize);

protected:
    void paintEvent(QPaintEvent *event) override;
    void moveEvent(QMoveEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

    void updateHandles();

signals:

public slots:

protected:
    QColor mColor;
    QColor mBrushColor;
    QBrush mBrush;

    bool mPenActive;
    bool mBrushActive;
    bool mResizeHandlesActive;
    int mResizeHandleSize;

    QPainterPath mTopLeftPath;
    QPainterPath mTopRightPath;
    QPainterPath mBottomLeftPath;
    QPainterPath mBottomRightPath;
};

} // namespace tl
