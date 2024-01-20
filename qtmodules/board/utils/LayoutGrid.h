#pragma once

#include <qcp/qcustomplot.h>

namespace tl {

class LayoutGrid : public QCPLayoutGrid
{
    Q_OBJECT

public:
    LayoutGrid();

    void updateLayout() override;

    void needUpdate(bool needUpdate);
    bool needUpdate() const;

protected:
    QRectF mPreviousRect;
    bool mNeedUpdate;

    // For accessing some protected methods
    // FIXME: make a public method for this purpose
    friend class SingleDisplay;
    friend class NumericalDisplay;
    friend class GaugeHorizontal;
    friend class GaugeVertical;
    friend class AlarmPanel;
    friend class TableValues;
    friend class PlotXY;
    friend class PlotTime;
    friend class TextLabel;
};

} // namespace tl
