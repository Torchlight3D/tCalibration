#pragma once

#include <QSharedPointer>

#include "data/TimeSeries.h"
#include "settings/ParameterConfiguration.h"

class QSettings;

namespace tl {

enum ValueStringFormat
{
    vsfDecimal,
    vsfHexa,
    vsfHexaLsb16,
    vsfHexaMsb16,
    vsfTimeFromSec,
    vsfBinary
};

class Board;

class BoardParameter
{
public:
    using Ptr = QSharedPointer<BoardParameter>;

    BoardParameter(Board *board);
    BoardParameter(const QSharedPointer<TimeSeries> &dataParameter,
                   Board *board);
    BoardParameter(
        ParameterConfiguration::Ptr paramProperties,
        Board *board);
    BoardParameter(const QString &parameterLabel, Board *board);

    QString getDisplayedLabel() const;
    QString getDisplayedUnit() const;

    QString getValueString(ValueStringFormat format = vsfDecimal) const;
    double getValueDouble() const;
    quint32 getValueBinaryWeight32() const;
    QString getStateString() const;
    QString getBitDescription(int bitNumber) const;
    bool getBitLogic(int bitNumber) const;

    void updateData();
    void processValueData();
    void processValueData(double value);
    void processStateData();

    bool configurationHasChanged();
    void modificationsApplied();

    bool connected() const;
    void disconnectSharedConfiguration(bool resetSharedConfig = false);

    void saveParameterSettings(QSettings *settings,
                               ParameterConfiguration::ConfigurationMode mode =
                                   ParameterConfiguration::cmFull);
    void loadParameterSettings(QSettings *settings,
                               ParameterConfiguration::ConfigurationMode mode =
                                   ParameterConfiguration::cmFull);

    ParameterConfiguration::Ptr
    sharedParameterConfiguration() const;
    ParameterConfiguration::Ptr
    exclusiveParameterConfiguration() const;
    ParameterConfiguration::Ptr
    parameterConfiguration() const;

    void setSharedParameterConfiguration(
        const ParameterConfiguration::Ptr &configs,
        bool connected);
    void setExclusiveParameterConfiguration(
        const ParameterConfiguration::Ptr &configs);

    double getTimestamp() const;

    QColor getColor() const;
    QColor getForegroundColor() const;
    QBrush getBackgroundBrush() const;

    // TODO: change name
    void setTimeSeries(const TimeSeriesParameter &timeSeries);
    TimeSeriesParameter getTimeSeries() const;

protected:
    double mTimestamp;
    DataValue mValue;

    QColor mColor;
    QColor mForegroundColor;
    QBrush mBackgroundBrush;

    Board *mBoard;

    TimeSeriesParameter mTimeSeries;

    bool mParameterConfigurationIsShared;
    ParameterConfiguration::Ptr
        mSharedParameterConfiguration;
    ParameterConfiguration::Ptr
        mExclusiveParameterConfiguration;
    ParameterConfiguration::Ptr
        mParameterConfiguration;
};

} // namespace tl
