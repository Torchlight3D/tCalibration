#pragma once

#include <qcp/qcustomplot.h>

#include "Alarms.h"
#include "settings/ParameterConfiguration.h"

namespace thoht {

class Alarms;
class Board;
class BoardParameter;
class TimeSeries;

class BoardElementPrivate;
class BoardElement : public QCPLayoutElement
{
    Q_OBJECT

public:
    enum ElementType
    {
        etUnknown = 0,
        etAlarm = 1,
        etTimeSeries = 2,
        etOther = 10
    };

    enum TimeSeriesType
    {
        tstUnknown = 0x1,
        tstScalar = 0x2,
        tstVector = 0x4,
        tstScalarAndVector = 0x8
    };
    Q_DECLARE_FLAGS(TimeSeriesTypes, TimeSeriesType)
    Q_FLAG(TimeSeriesTypes)

    enum TimeSeriesSize
    {
        tssUnknown = 1,
        tssSingle = 2,
        tssMulti = 4
    };
    Q_DECLARE_FLAGS(TimeSeriesSizes, TimeSeriesSize)
    Q_FLAG(TimeSeriesSizes)

    enum BackgroundMode
    {
        bmBackground,
        bmTransparent
    };
    Q_ENUM(BackgroundMode)

    explicit BoardElement(Board *dashboard = nullptr);
    ~BoardElement() override;

    void update(UpdatePhase phase) override;
    double selectTest(const QPointF &pos, bool onlySelectable,
                      QVariant *details = nullptr) const override;

    virtual void initialize(Board *dashboard);
    virtual void edit() {}

    void rebuildElement(bool replot = false);
    virtual void clearElement() {}
    virtual void resetElement() {}
    virtual void buildElement() {}

    virtual void beforeDataUpdate(bool refresh);
    virtual void resetModifications();

    virtual void droppedParameter(
        ParameterConfiguration::Ptr parameterSettings);
    virtual void droppedParameter(QSharedPointer<TimeSeries> dataParameter,
                                  bool replace);

    QSharedPointer<BoardParameter> boardParameter(int index);
    QSharedPointer<BoardParameter> boardParameter(QString label);

    virtual QSharedPointer<BoardParameter> addParameter(
        ParameterConfiguration::Ptr parameterSettings);
    virtual QSharedPointer<BoardParameter> addParameter(QString paramLabel);
    virtual QSharedPointer<BoardParameter> addParameter(
        QSharedPointer<TimeSeries> dataParameter);
    virtual void addParameter(QSharedPointer<BoardParameter> boardParameter);

    virtual void removeBoardParameter(int index, bool update = true);
    virtual void removeBoardParameter(QString label, bool update = true);
    virtual void removeBoardParameters();

    virtual QSharedPointer<BoardParameter> replaceParameter(
        int index,
        ParameterConfiguration::Ptr parameterSettings);
    virtual QSharedPointer<BoardParameter> replaceParameter(int index,
                                                            QString paramLabel);
    virtual QSharedPointer<BoardParameter> replaceParameter(
        int index, QSharedPointer<TimeSeries> dataParameter);
    virtual void replaceParameter(
        int index, QSharedPointer<BoardParameter> boardParameter);

    virtual void addAlarm(QExplicitlySharedDataPointer<Alarms> alarmConfig)
    {
        Q_UNUSED(alarmConfig)
    }

    virtual void loadData();
    virtual void loadHistoricalData();
    virtual void displayData();
    virtual void checkParameters();

    int parametersCount() { return mBoardParameters.count(); }

    virtual void loadSettings(QSettings *settings) { Q_UNUSED(settings) }
    virtual void saveSettings(QSettings *settings) { Q_UNUSED(settings) }

    virtual void saveConfigurations(QSettings *settings);
    virtual void loadConfigurations(QSettings *settings);

    virtual void setBoardLiveState(bool isLive);

    void setMainElement(bool mainElement);

    virtual int defaultWidth() = 0;
    virtual int defaultHeight() = 0;

    void setName(const QString &name);
    QString name() const;

    void setType(ElementType type);
    ElementType type() const;

    void setTimeSeriesType(TimeSeriesType type);
    TimeSeriesType timeSeriesType() const;

    void setTimeSeriesSize(TimeSeriesSize sizes);
    TimeSeriesSize timeSeriesSize() const;

    void setBackgroundMode(BackgroundMode mode);
    BackgroundMode backgroundMode() const;

    void setParametersMaxCount(int max);
    int parametersMaxCount() const;

    void setStatModeEnabled(bool enable);
    bool statModeEnabled() const;

    void setMouseInteractionsEnabled(bool enable);
    bool mouseInteractionsEnabled() const;

    void setConfigurationMode(ParameterConfiguration::ConfigurationMode mode);
    ParameterConfiguration::ConfigurationMode configurationMode() const;

    Board *board() const;

protected:
    void draw(QCPPainter *painter) override;

public slots:
    virtual void mouseMoveSlot(QMouseEvent *event) { Q_UNUSED(event) }

protected:
    BackgroundMode mBackgroundMode;
    QBrush mDefaultBrush;
    QBrush mHGradientBrush;
    QBrush mVGradientBrush;
    QPen mBorderPen;
    Board *mBoard;

    int mParametersMaxCount;
    QString mName;
    ElementType mType;
    TimeSeriesType mTimeSeriesType;
    TimeSeriesSize mTimeSeriesSize;
    bool mStatModeEnabled;
    bool mMouseInteractionsEnabled;
    bool mMainElement;

    QList<QSharedPointer<BoardParameter>> mBoardParameters;
    QStringList mParametersLabel;
    ParameterConfiguration::ConfigurationMode mConfigurationMode;

    int mParameterColorIndex;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(BoardElement::TimeSeriesTypes)
Q_DECLARE_OPERATORS_FOR_FLAGS(BoardElement::TimeSeriesSizes)

} // namespace thoht
