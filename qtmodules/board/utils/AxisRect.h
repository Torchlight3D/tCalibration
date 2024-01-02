#pragma once

#include <qcp/qcustomplot.h>

namespace thoht {
class AxisRect : public QCPAxisRect
{
    Q_OBJECT

public:
    AxisRect(QCustomPlot *plot);
    void clearPlottables();

    void setAxisColor(QColor color);

protected:
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event, const QVariant &details) override;
};

} // namespace thoht
