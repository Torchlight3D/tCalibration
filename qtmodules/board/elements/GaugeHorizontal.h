#pragma once

#include "GaugeVertical.h"

namespace tl {

class GaugeHorizontal : public GaugeVertical
{
    Q_OBJECT

public:
    explicit GaugeHorizontal(Board *dashboard = nullptr);

    // reimplemented virtual methods:
    int defaultWidth() override;
    int defaultHeight() override;
    void initialize(Board *dashboard) override;
    void edit() override;

    void loadSettings(QSettings *settings) override;
    void saveSettings(QSettings *settings) override;

    void buildElement() override;
    void resetElement() override;

    void displayData() override;
    void applySizeConstraints() override;

    void setAxisTicksVisible(bool visible) override;
    void setAxisLabelsVisible(bool visible) override;
    void setAxisGridVisible(bool visible) override;
    void setAxisLineVisible(bool visible) override;

    enum GaugeHPosition
    {
        gpRight,
        gpLeft
    };
    void setGaugeHPosition(GaugeHPosition position);
    GaugeHPosition gaugeHPosition() const;

protected:
    GaugeHPosition mGaugeHPosition;
};

} // namespace tl
