#pragma once

#include "NumericalDisplay.h"

namespace thoht {

class GaugeRect;

class GaugeRadial : public NumericalDisplay
{
    Q_OBJECT

public:
    explicit GaugeRadial(Board *dashboard = nullptr);

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

    void displayData() override;
    void loadHistoricalData() override;

    enum AxisScale
    {
        asAuto,
        asParam,
        asCustom
    };
    void setAxisScale(AxisScale axisScale);
    AxisScale axisScale() const;

    void setAxisMaxCustom(double max);
    double axisMaxCustom() const;
    void setAxisMinCustom(double min);
    double axisMinCustom() const;
    void setAxisTicksVisible(bool visible);
    bool axisTicksVisible() const;
    void setAxisLineVisible(bool visible);
    bool axisLineVisible() const;
    void setAxisLabelsVisible(bool visible);
    bool axisLabelsVisible() const;

    void setThresholdsVisible(bool visible);
    bool thresholdsVisible() const;

    enum GaugeRPosition
    {
        gpTop,
        gpMiddle,
        gpBottom
    };
    void setGaugePosition(GaugeRPosition gaugePosition);
    GaugeRPosition gaugePosition() const;

    void setAngleSpan(int angleSpan);
    int angleSpan() const;
    void setAngleStart(int angleStart);
    int angleStart() const;

protected:
    GaugeRect *mGauge;
    AxisScale mAxisScale;
    double mAxisMaxCustom;
    double mAxisMinCustom;
    GaugeRPosition mGaugePosition;
    int mAngleSpan;
    int mAngleStart;
};

} // namespace thoht
