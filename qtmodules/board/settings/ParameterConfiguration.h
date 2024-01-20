#pragma once

#include <QSharedData>
#include <qcp/qcustomplot.h>

#include "Bitfield.h"
#include "ColorSettings.h"
#include "States.h"
#include "Thresholds.h"

class QSettings;

namespace tl {

class ParameterConfiguration : public QSharedData
{
public:
    enum ConfigurationMode
    {
        cmFull,
        cmValue,
        cmState,
        cmBitFields,
        cmCurveX,
        cmCurveY,
        cmGraph
    };

    enum ItemColor
    {
        icBase,
        icDynamic,
        icDynamicSegmented,
        icCustom
    };

    enum BrushStyle
    {
        bsNone,
        bsFilled,
        bsGradient
    };

    using Ptr = QExplicitlySharedDataPointer<ParameterConfiguration>;

    ParameterConfiguration();

    void saveToFile(const QString &saveDirectory);
    void loadFromFile(const QString &settingsFile);

    void save(QSettings *settings, ConfigurationMode mode = cmFull);
    void load(QSettings *settings, ConfigurationMode mode = cmFull);

    void setPrecision(int precision);
    int precision() const;

    void setDescription(const QString &description);
    QString description() const;

    void setUserDefinedLabel(const QString &label);
    QString userDefinedLabel() const;

    void setUserDefinedUnit(const QString &unit);
    QString userDefinedUnit() const;

    void setValidRange(bool valid);
    bool validRange() const;

    void setRangeMaximum(double max);
    double rangeMaximum() const;
    void setRangeMinimum(double min);
    double rangeMinimum() const;

    void setLabel(const QString &label);
    QString label() const;
    void setUserLabelEnabled(bool enable);
    bool userLabelEnabled() const;
    void setUserUnitEnabled(bool enable);
    bool userUnitEnabled() const;

    void setOutOfRangeColorEnabled(bool enable);
    bool outOfRangeColorEnabled() const;

    void setGraphBrush(BrushStyle style);
    BrushStyle graphBrush() const;

    void setGraphLineStyle(QCPGraph::LineStyle style);
    QCPGraph::LineStyle graphLineStyle() const;

    void setPenWidth(double width);
    double penWidth() const;

    void setScatterShape(QCPScatterStyle::ScatterShape shape);
    QCPScatterStyle::ScatterShape scatterShape() const;

    void setScatterSize(int size);
    int scatterSize() const;

    void setScatterStyle(const QCPScatterStyle &style);
    QCPScatterStyle scatterStyle() const;

    void setCurveLineStyle(QCPCurve::LineStyle style);
    QCPCurve::LineStyle curveLineStyle() const;

    void setCurveThresholdsVisible(bool visible);
    bool itemsThresholdsVisible() const;

    void setCurveTracerVisible(bool visible);
    bool curveTracerVisible() const;

    void setGraphThresholdsVisible(bool visible);
    void setItemsThresholdsVisible(bool visible);

    void setModified(bool modified);
    bool modified();

    void setItemStaticColor(const QColor &color);
    QColor itemStaticColor() const;
    void setItemColorMode(ItemColor mode);
    ItemColor itemColorMode() const;

    ColorSettings &defaultColorSettingsRef();
    Thresholds &thresholdsSettingsRef();
    ColorSettings &outOfRangeColorSettingsRef();
    States &statesSettingsRef();
    Bitfield &bitfieldsSettingsRef();

    QString saveFilePath() const;

protected:
    QString mSaveFilePath;
    int mPrecision;
    QString mLabel;
    QString mDescription;
    QString mUserDefinedLabel;
    bool mUserLabelEnabled;
    QString mUserDefinedUnit;
    bool mUserUnitEnabled;
    bool mValidRange;
    double mRangeMaximum;
    double mRangeMinimum;
    bool mOutOfRangeColorEnabled;

    bool mItemsThresholdsVisible;

    QColor mItemStaticColor;
    ItemColor mItemColorMode;
    double mPenWidth;

    QCPGraph::LineStyle mGraphLineStyle;
    BrushStyle mGraphBrush;

    QCPCurve::LineStyle mCurveLineStyle;
    QCPScatterStyle::ScatterShape mScatterShape;
    int mScatterSize;
    QCPScatterStyle mScatterStyle;
    bool mCurveTracerVisible;

    ColorSettings mOutOfRangeColorSettings;
    ColorSettings mDefaultColorSettings;
    Thresholds mThresholdsSettings;
    States mStatesSettings;
    Bitfield mBitfieldsSettings;

    bool mModified;
};
} // namespace tl
