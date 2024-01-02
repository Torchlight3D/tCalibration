#pragma once

#include "../BoardElement.h"

namespace thoht {

class AdaptiveTextElement;
class AxisRect;
class LayoutGrid;
class PlotTimeTracer;

class PlotTime : public BoardElement
{
    Q_OBJECT

public:
    explicit PlotTime(Board *dashboard = nullptr);

    // reimplemented virtual methods:
    int defaultWidth() override;
    int defaultHeight() override;
    void initialize(Board *dashboard) override;
    void clearElement() override;
    void edit() override;
    QList<QCPLayoutElement *> elements(bool recursive) const override;

    void loadSettings(QSettings *settings) override;
    void saveSettings(QSettings *settings) override;

    void loadConfigurations(QSettings *settings) override;

    QSharedPointer<BoardParameter> addParameter(
        ParameterConfiguration::Ptr parameterSettings)
        override;
    QSharedPointer<BoardParameter> addParameter(QString paramLabel) override;
    void addParameter(QSharedPointer<BoardParameter> boardParameter) override;

    QSharedPointer<BoardParameter> replaceParameter(
        int index,
        ParameterConfiguration::Ptr parameterSettings)
        override;
    QSharedPointer<BoardParameter> replaceParameter(
        int index, QString paramLabel) override;
    void replaceParameter(
        int index, QSharedPointer<BoardParameter> boardParameter) override;
    void removeBoardParameter(int index, bool update = true) override;
    void checkParameters() override;

    void update(UpdatePhase phase) override;
    void resetElement() override;
    void buildElement() override;
    void loadData() override;
    void displayData() override;
    void loadHistoricalData() override;

    void applySizeConstraints();
    void updateAxes();
    void resetItems();
    void updateItems();
    void updateGraphsStyle();
    void resetGraphsStyle();

    enum LegendPosition
    {
        lpTop,
        lpRight,
        lpBottom,
        lpLeft
    };
    void setLegendPosition(LegendPosition position);
    LegendPosition legendPosition() const;

    void setLegendVisible(bool visible);
    bool legendVisible() const;
    void setYAxisTicksVisible(bool visible);
    bool yAxisTicksVisible() const;
    void setYAxisTicksCount(int count);
    int yAxisTicksCount() const;

    void setYAxisLineVisible(bool visible);
    bool yAxisLineVisible() const;
    void setYAxisLabelsVisible(bool visible);
    bool yAxisLabelsVisible() const;
    void setYAxisGridVisible(bool visible);
    bool yAxisGridVisible() const;

    enum YAxisScale
    {
        asAuto,
        asAutoZero,
        asParam,
        asCustom
    };
    void setYAxisScale(YAxisScale scale);
    YAxisScale yAxisScale() const;

    void setYAxisMaxCustom(double max);
    double yAxisMaxCustom() const;
    void setYAxisMinCustom(double min);
    double yAxisMinCustom() const;
    void setXAxisTicksVisible(bool visible);
    bool xAxisTicksVisible() const;
    void setXAxisTicksCount(int count);
    int xAxisTicksCount() const;
    void setXAxisLineVisible(bool visible);
    bool xAxisLineVisible() const;
    void setXAxisLabelsVisible(bool visible);
    bool xAxisLabelsVisible() const;
    void setXAxisGridVisible(bool visible);
    bool xAxisGridVisible() const;
    void setXAxisHistory(double history);
    double xAxisHistory() const;

    enum XAxisDirection
    {
        adRightToLeft,
        adLeftToRight
    };
    void setXAxisDirection(XAxisDirection direction);
    XAxisDirection xAxisDirection() const;

    void setXAxisFixedTicker(bool fixed);
    bool xAxisFixedTicker() const;
    void setThresholdsVisible(bool visible);
    bool thresholdsVisible() const;
    void setBoardReferenceTimeActive(bool active);
    bool boardReferenceTimeActive() const;

    enum StatMode
    {
        smNone,
        smMin,
        smMax,
        smMinMax
    };
    void setStatMode(StatMode mode);
    StatMode statMode() const;

    void setTickStepMinValue(double min);
    double tickStepMinValue() const;
    void setTickStepMinValueEnabled(bool enable);
    bool tickStepMinValueEnabled() const;

    void setTitle(const QString &title);
    QString title() const;
    void setTitleVisible(bool visible);
    bool titleVisible() const;

public slots:
    void mouseMoveSlot(QMouseEvent *event) override;

protected:
    LayoutGrid *mMainLayout;
    LayoutGrid *mLayout;
    LayoutGrid *mLegendLayout;
    QCPLayoutElement *mLegendEmptyElementFirst{};
    QCPLayoutElement *mLegendEmptyElementLast{};
    AdaptiveTextElement *mTitleLabel;

    AxisRect *mAxisRect{};

    QCPAxis *mYRightAxis;
    QCPAxis *mXBottomMovingAxis;
    QCPAxis *mXBottomStaticAxis;
    LegendPosition mLegendPosition;
    bool mLegendVisible;
    bool mYAxisTicksVisible;
    int mYAxisTicksCount;
    bool mYAxisLineVisible;
    bool mYAxisLabelsVisible;
    bool mYAxisGridVisible;
    bool mXAxisTicksVisible;
    int mXAxisTicksCount;
    bool mXAxisLineVisible;
    bool mXAxisLabelsVisible;
    bool mXAxisGridVisible;
    bool mXAxisFixedTicker;
    bool mBoardReferenceTimeActive;
    YAxisScale mYAxisScale;
    double mYAxisMaxCustom{};
    double mYAxisMinCustom{};
    double mXAxisHistory;
    XAxisDirection mXAxisDirection;
    bool mThresholdsVisible;
    QCPRange mAutoRange;
    double mTickStepMinValue;
    bool mTickStepMinValueEnabled;
    bool mTitleVisible;
    QString mTitle;

    PlotTimeTracer *mTracer;

    StatMode mStatMode;
    QVector<QCPItemTracer *> mStatMaxTracers;
    QVector<QCPItemText *> mStatMaxTexts;
    QVector<QCPItemLine *> mStatMaxArrows;
    QVector<QCPItemTracer *> mStatMinTracers;
    QVector<QCPItemText *> mStatMinTexts;
    QVector<QCPItemLine *> mStatMinArrows;

    QSharedPointer<QCPAxisTickerFixed> mValueTicker;
    QSharedPointer<QCPAxisTickerDateTime> mTimeTicker;
};

} // namespace thoht
