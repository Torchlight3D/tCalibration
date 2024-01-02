#pragma once

#include "NumericalDisplay.h"

namespace thoht {

class AxisRect;
class GaugeVertical : public NumericalDisplay
{
    Q_OBJECT

public:
    GaugeVertical(Board *dashboard = nullptr);

    // reimplemented virtual methods:
    int defaultWidth() override;
    int defaultHeight() override;
    void initialize(Board *dashboard) override;
    void clearElement() override;
    void edit() override;

    void loadSettings(QSettings *settings) override;
    void saveSettings(QSettings *settings) override;

    void buildElement() override;
    void checkParameters() override;
    void resetElement() override;

    void displayData() override;
    void loadHistoricalData() override;
    void applySizeConstraints() override;

    void resetThresholds();

    void setTankGauge(bool tankGauge);
    bool tankGauge() const;

    enum AxisScale
    {
        asAuto,
        asParam,
        asCustom
    };
    void setAxisScale(AxisScale scale);
    AxisScale axisScale() const;

    enum GaugeVPosition
    {
        gpTop,
        gpMiddle,
        gpBottom
    };
    void setGaugeVPosition(const GaugeVPosition &gaugePosition);
    GaugeVPosition gaugeVPosition() const;

    void setAxisMaxCustom(double max);
    double axisMaxCustom() const;
    void setAxisMinCustom(double min);
    double axisMinCustom() const;

    virtual void setAxisTicksVisible(bool visible);
    bool axisTicksVisible() const;
    virtual void setAxisLabelsVisible(bool visible);
    bool axisLabelsVisible() const;
    virtual void setAxisGridVisible(bool visible);
    bool axisGridVisible() const;
    virtual void setAxisLineVisible(bool visible);
    bool axisLineVisible() const;

protected:
    LayoutGrid *mBarLayout;
    QCPLayoutElement *mBarLayoutEmptyElementFirst;
    QCPLayoutElement *mBarLayoutEmptyElementSecond;
    AxisRect *mAxisRect;
    AxisRect *mAxisRectThresholds;
    QCPBars *mBar;
    QVector<QCPItemRect *> mThresholdTracers;
    QCPRange mValueAxisRange;
    bool mAxisTicksVisible;
    bool mAxisLabelsVisible;
    bool mAxisLineVisible;
    bool mAxisGridVisible;
    AxisScale mAxisScale;
    double mAxisMaxCustom;
    double mAxisMinCustom;
    bool mTankGauge;
    GaugeVPosition mGaugeVPosition;
};

} // namespace thoht
