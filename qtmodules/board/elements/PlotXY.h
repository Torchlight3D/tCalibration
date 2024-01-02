#pragma once

#include "../BoardElement.h"

namespace thoht {

class AdaptiveTextElement;
class AxisRect;
class CurvePatron;
class LayoutGrid;
class NumericalDisplay;

class PlotXY : public BoardElement
{
    Q_OBJECT

public:
    explicit PlotXY(Board *dashboard = nullptr);

    // reimplemented virtual methods:
    int defaultWidth() override;
    int defaultHeight() override;
    void initialize(Board *dashboard) override;
    void clearElement() override;
    void edit() override;

    void loadSettings(QSettings *settings) override;
    void saveSettings(QSettings *settings) override;
    void saveConfigurations(QSettings *settings) override;
    void loadConfigurations(QSettings *settings) override;

    void checkParameters() override;

    void droppedParameter(ParameterConfiguration::Ptr
                              parameterSettings) override;
    void droppedParameter(QSharedPointer<TimeSeries> dataParameter,
                          bool replace) override;
    QSharedPointer<BoardParameter> addYParameter(
        ParameterConfiguration::Ptr parameterSettings);
    QSharedPointer<BoardParameter> addYParameter(QString paramLabel);
    QSharedPointer<BoardParameter> addYParameter(
        QSharedPointer<TimeSeries> dataParameter);
    QSharedPointer<BoardParameter> addXParameter(
        ParameterConfiguration::Ptr parameterSettings);
    QSharedPointer<BoardParameter> addXParameter(QString paramLabel);
    QSharedPointer<BoardParameter> addXParameter(
        const QSharedPointer<TimeSeries> &dataParameter);
    void removeYParameter(int index, bool update = true);
    void removeXParameter();
    void removeBoardParameters() override;

    void update(UpdatePhase phase) override;
    void resetElement() override;
    void buildElement() override;
    void loadData() override;
    void loadHistoricalData() override;
    void displayData() override;

    void applySizeConstraints();
    void updateAxes();
    void updateItems();
    void updateGraphsStyle();

    void addPatron(QSharedPointer<CurvePatron> patron);
    void removePatron(int index);

    QSharedPointer<BoardParameter> xParameter() const;

    enum YLegendPosition
    {
        lpRight,
        lpLeft
    };
    void setYLegendPosition(YLegendPosition position);
    YLegendPosition yLegendPosition() const;
    enum XLegendPosition
    {
        lpTop,
        lpBottom
    };
    void setXLegendPosition(XLegendPosition position);
    XLegendPosition xLegendPosition() const;

    enum AxisScale
    {
        asAuto,
        asParam,
        asCustom
    };
    void setYAxisScale(AxisScale scale);
    AxisScale yAxisScale() const;

    void setXAxisScale(AxisScale scale);
    AxisScale xAxisScale() const;
    void setYLegendVisible(bool visible);
    bool yLegendVisible() const;
    void setXLegendVisible(bool visible);
    bool xLegendVisible() const;
    void setYAxisTicksVisible(bool visible);
    bool yAxisTicksVisible() const;
    void setYAxisTicksCount(int count);
    int yAxisTicksCount() const;
    void setYAxisLabelsVisible(bool visible);
    bool yAxisLabelsVisible() const;
    void setYAxisGridVisible(bool visible);
    bool yAxisGridVisible() const;
    void setXAxisTicksVisible(bool visible);
    bool xAxisTicksVisible() const;
    void setXAxisTicksCount(int count);
    int xAxisTicksCount() const;
    void setXAxisLabelsVisible(bool visible);
    bool xAxisLabelsVisible() const;
    void setXAxisGridVisible(bool visible);
    bool xAxisGridVisible() const;
    void setXAxisHistory(int history);
    int xAxisHistory() const;
    void setYAxisMaxCustom(double max);
    double yAxisMaxCustom() const;

    void setYAxisMinCustom(double min);
    double yAxisMinCustom() const;
    void setXAxisMaxCustom(double max);
    double xAxisMaxCustom() const;

    void setXAxisMinCustom(double min);
    double xAxisMinCustom() const;

    void setYAxisLineVisible(bool visible);
    bool yAxisLineVisible() const;

    void setXAxisLineVisible(bool visible);
    bool xAxisLineVisible() const;

    void setYThresholdsVisible(bool visible);
    bool yThresholdsVisible() const;
    void setXThresholdsVisible(bool visible);
    bool xThresholdsVisible() const;
    void setTitleVisible(bool visible);
    bool titleVisible() const;
    void setTitle(const QString &title);
    QString title() const;
    void setPlotInRangePoints(bool plotInRangePoints);
    bool plotInRangePoints() const;

    QList<QSharedPointer<CurvePatron>> patrons() const;

    QList<QSharedPointer<BoardParameter>> yParameters() const;

protected:
    LayoutGrid *mMainLayout;
    LayoutGrid *mLayout;
    LayoutGrid *mYLegendLayout;
    LayoutGrid *mXLegendLayout;
    AxisRect *mAxisRect;
    AdaptiveTextElement *mTitleLabel;

    bool mTitleVisible;
    QString mTitle;

    YLegendPosition mYLegendPosition;
    AxisScale mYAxisScale;
    bool mYLegendVisible;
    double mYAxisMaxCustom;
    double mYAxisMinCustom;
    QCPLayoutElement *mYLegendEmptyElementBottom;
    QCPLayoutElement *mYLegendEmptyElementTop;

    XLegendPosition mXLegendPosition;
    AxisScale mXAxisScale;
    bool mXLegendVisible;
    double mXAxisMaxCustom;
    double mXAxisMinCustom;
    QCPLayoutElement *mXLegendEmptyElementRight;
    QCPLayoutElement *mXLegendEmptyElementLeft;

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
    int mXAxisHistory;

    bool mYThresholdsVisible;
    bool mXThresholdsVisible;

    bool mPlotInRangePoints;

    QList<QCPCurve *> mCurves;
    QList<QCPItemTracer *> mTracers;
    QList<QCPItemStraightLine *> mHLines;
    QList<QCPItemStraightLine *> mVLines;

    QList<QSharedPointer<CurvePatron>> mPatrons;

    QList<QSharedPointer<BoardParameter>> mYParameters;
    QList<NumericalDisplay *> mYDisplays;
    QSharedPointer<BoardParameter> mXParameter;
    NumericalDisplay *mXDisplay;
};

} // namespace thoht
