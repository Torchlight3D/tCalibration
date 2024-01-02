#pragma once

#include <QMap>

#include "ColorSettings.h"

namespace thoht {

class Thresholds
{
public:
    enum CheckOrder
    {
        coLowThresholdsFirst,
        coHighThresholdsFirst
    };

    Thresholds();
    ~Thresholds();

    void insertLowThreshold(double value, const ColorSettings& settings);
    void insertHighThreshold(double value, const ColorSettings& settings);

    void removeLowThreshold(double value);
    void removeHighThreshold(double value);

    void clearLowThresholds();
    void clearHighThresholds();

    // TODO: change return value and parameters
    ColorSettings colorSettings(double value, bool& colorIsSet);

    bool active();

    CheckOrder checkOrder() const;
    void setCheckOrder(CheckOrder order);

    QMap<double, ColorSettings> highThresholds() const;
    QMap<double, ColorSettings> lowThresholds() const;

    void setModified(bool modified);
    bool modified() const;

protected:
    CheckOrder mCheckOrder;
    QMap<double, ColorSettings> mHighThresholds;
    QMap<double, ColorSettings> mLowThresholds;
    bool mModified;
};

} // namespace thoht
