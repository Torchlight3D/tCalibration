#include "GaugeRadial.h"
#include "editor/ui_GaugeRadialEditor.h"

#include <QDialog>

#include "../Board.h"
#include "../BoardElement.h"
#include "../BoardLayout.h"
#include "../utils/AdaptiveTextElement.h"
#include "../utils/LayoutGrid.h"
#include "../data/DataManager.h"
#include "../widgets/ElementPropertiesWidget.h"

namespace tl {

///------- GaugeRadial Components starts from here
class CircularAxis : public QCPLayoutElement
{
    Q_OBJECT

public:
    CircularAxis(Board *dashboard);

    QPen basePen() const;
    void setBasePen(const QPen &basePen);

    bool tickLabels() const;
    void setTickLabels(bool tickLabels);

    QFont tickLabelFont() const;
    void setTickLabelFont(const QFont &tickLabelFont);

    QColor tickLabelColor() const;
    void setTickLabelColor(const QColor &tickLabelColor);

    bool ticks() const;
    void setTicks(bool show);

    bool subTicks() const;
    void setSubTicks(bool show);

    int tickLengthIn() const;
    void setTickLengthIn(int inside);

    int tickLengthOut() const;
    void setTickLengthOut(int outside);

    int subTickLengthIn() const;
    void setSubTickLengthIn(int inside);

    int subTickLengthOut() const;
    void setSubTickLengthOut(int outside);

    QPen tickPen() const;
    void setTickPen(const QPen &tickPen);

    QPen subTickPen() const;
    void setSubTickPen(const QPen &subTickPen);

    QCPRange range() const;
    void setRange(const QCPRange &range);

    void setTickLabelPadding(int padding);

    void setRange(double lower, double upper);
    void setRangeLower(double lower);
    void setRangeUpper(double upper);

    virtual void update(UpdatePhase phase) override;

    double coordToAngleRad(double coord) const
    {
        if (coord < mRange.lower)
            return mAngleRad;
        if (coord > mRange.upper)
            return mAngleRad + (mRangeReversed
                                    ? -(mSpanAngle / 360.0) * 2.0 * M_PI
                                    : (mSpanAngle / 360.0) * 2.0 * M_PI);

        return mAngleRad + (coord - mRange.lower) / mRange.size() *
                               (mRangeReversed
                                    ? -(mSpanAngle / 360.0) * 2.0 * M_PI
                                    : (mSpanAngle / 360.0) * 2.0 * M_PI);
    } // mention in doc that return doesn't wrap

    double radius() const;
    QPointF center() const;
    double angleRad() const;
    QColor baseColor() const;
    double spanAngle() const;

    void setSpanAngle(double spanAngle);

    double angle() const;
    void setAngle(double angle);

protected:
    virtual void draw(QCPPainter *painter) override;
    void setupTickVectors();

    double mAngle;
    double mAngleRad;
    double mSpanAngle;
    QPen mBasePen;
    // tick labels:
    bool mTickLabels;
    QFont mTickLabelFont;
    QColor mTickLabelColor;
    QColor mBaseColor;
    int mNumberPrecision;
    QLatin1Char mNumberFormatChar;
    // ticks and subticks:
    bool mTicks;
    bool mSubTicks;
    int mTickLengthIn, mTickLengthOut, mSubTickLengthIn, mSubTickLengthOut;
    QPen mTickPen;
    QPen mSubTickPen;
    // scale and range:
    QCPRange mRange;
    bool mRangeReversed;

    QPointF mCenter;
    double mRadius;
    QVector<double> mTickVector;
    QVector<QString> mTickVectorLabels;
    QVector<QPointF> mTickVectorCosSin;
    QVector<double> mSubTickVector;
    QVector<QPointF> mSubTickVectorCosSin;
    QSharedPointer<QCPAxisTicker> mTicker;
    QCPLabelPainterPrivate *mLabelPainter;

    friend class QTBGaugeRect;
};

CircularAxis::CircularAxis(Board *dashboard)
    : QCPLayoutElement(dashboard),
      mAngle(135),
      mAngleRad(mAngle / 180.0 * M_PI),
      mSpanAngle(270),
      mTickLabels(true),
      mTickLabelFont(mParentPlot->font()),
      mNumberPrecision(6),
      mNumberFormatChar('g'),
      mTicks(true),
      mSubTicks(true),
      mTickLengthIn(10),
      mTickLengthOut(0),
      mSubTickLengthIn(4),
      mSubTickLengthOut(0),
      mRange(0, mSpanAngle),
      mRangeReversed(false),
      mRadius(1),
      mTicker(new QCPAxisTicker)
{
    auto textColor = QApplication::palette().toolTipText().color();
    mBasePen = QPen(textColor, 0, Qt::SolidLine, Qt::RoundCap);

    mTickLabelColor = textColor;
    mBaseColor = textColor;
    mTickPen = QPen(textColor, 0, Qt::SolidLine, Qt::RoundCap);
    mSubTickPen = QPen(textColor, 0, Qt::SolidLine, Qt::RoundCap);

    mLabelPainter = new QCPLabelPainterPrivate(dashboard);

    mTicker->setTickCount(6);
    mTicker->setTickStepStrategy(QCPAxisTicker::tssMeetTickCount);
    setAntialiased(true);
    setLayer(QLatin1String("axes"));
    setMinimumMargins(QMargins(45, 30, 45, 30));

    mLabelPainter->setPadding(5);
    mLabelPainter->setRotation(0);
    mLabelPainter->setAnchorMode(QCPLabelPainterPrivate::amSkewedUpright);
}

void CircularAxis::draw(QCPPainter *painter)
{
    //    qDebug() << Q_FUNC_INFO << mRect << painter->clipBoundingRect() <<
    //    painter->clipRegion().boundingRect();
    painter->setPen(mBasePen);
    QRectF axisRect = mRect;
    axisRect.setWidth(2 * mRadius);
    axisRect.setHeight(2 * mRadius);
    axisRect.moveCenter(mCenter.toPoint());
    painter->drawArc(axisRect, -1 * (mAngle)*16, -mSpanAngle * 16);

    // draw subticks:
    if (!mSubTickVector.isEmpty()) {
        painter->setPen(mSubTickPen);
        for (int i = 0; i < mSubTickVector.size(); ++i) {
            painter->drawLine(mCenter + mSubTickVectorCosSin.at(i) *
                                            (mRadius - mSubTickLengthIn),
                              mCenter + mSubTickVectorCosSin.at(i) *
                                            (mRadius + mSubTickLengthOut));
        }
    }

    // draw ticks and labels:
    if (!mTickVector.isEmpty()) {
        mLabelPainter->setAnchorReference(mCenter);
        mLabelPainter->setFont(mTickLabelFont);
        mLabelPainter->setColor(mTickLabelColor);
        painter->setClipRect(mOuterRect);
        const QPen ticksPen = mTickPen;
        painter->setPen(ticksPen);
        for (int i = 0; i < mTickVector.size(); ++i) {
            const QPointF outerTick =
                mCenter + mTickVectorCosSin.at(i) * (mRadius + mTickLengthOut);
            painter->drawLine(
                mCenter + mTickVectorCosSin.at(i) * (mRadius - mTickLengthIn),
                outerTick);
            // draw tick labels:
            if (!mTickVectorLabels.isEmpty()) {
                if (i < mTickVectorLabels.count() - 1 ||
                    (mTickVectorCosSin.at(i) - mTickVectorCosSin.first())
                            .manhattanLength() >
                        5 / 180.0 * M_PI) // skip last label if it's closer than
                                          // approx 5 degrees to first
                    mLabelPainter->drawTickLabel(painter, outerTick,
                                                 mTickVectorLabels.at(i));
            }
        }
    }
}

void CircularAxis::setupTickVectors()
{
    if (!mParentPlot)
        return;
    if ((!mTicks && !mTickLabels) || mRange.size() <= 0)
        return;

    mSubTickVector
        .clear(); // since we might not pass it to mTicker->generate(), and we
                  // don't want old data in there
    mTicker->generate(mRange, mParentPlot->locale(), mNumberFormatChar,
                      mNumberPrecision, mTickVector,
                      mSubTicks ? &mSubTickVector : nullptr,
                      mTickLabels ? &mTickVectorLabels : nullptr);

    // fill cos/sin buffers which will be used by draw() and
    // QCPPolarGrid::draw(), so we don't have to calculate it twice:
    mTickVectorCosSin.resize(mTickVector.size());
    for (int i = 0; i < mTickVector.size(); ++i) {
        const double theta = coordToAngleRad(mTickVector.at(i));
        mTickVectorCosSin[i] = QPointF(qCos(theta), qSin(theta));
    }
    mSubTickVectorCosSin.resize(mSubTickVector.size());
    for (int i = 0; i < mSubTickVector.size(); ++i) {
        const double theta = coordToAngleRad(mSubTickVector.at(i));
        mSubTickVectorCosSin[i] = QPointF(qCos(theta), qSin(theta));
    }
}

double CircularAxis::angle() const { return mAngle; }

void CircularAxis::setAngle(double angle)
{
    mAngle = angle;
    mAngleRad = mAngle / 180.0 * M_PI;
}

double CircularAxis::spanAngle() const { return mSpanAngle; }

void CircularAxis::setSpanAngle(double spanAngle) { mSpanAngle = spanAngle; }

QColor CircularAxis::baseColor() const { return mBaseColor; }

double CircularAxis::angleRad() const { return mAngleRad; }

QPointF CircularAxis::center() const { return mCenter; }

double CircularAxis::radius() const { return mRadius; }

QCPRange CircularAxis::range() const { return mRange; }

void CircularAxis::setRange(const QCPRange &range)
{
    if (qFuzzyCompare(range.lower, mRange.lower) &&
        qFuzzyCompare(range.upper, mRange.upper))
        return;

    if (!QCPRange::validRange(range))
        return;
    mRange = range.sanitizedForLinScale();
}

void CircularAxis::setTickLabelPadding(int padding)
{
    Q_UNUSED(padding)
    //    mLabelPainter.setPadding(padding);
}

void CircularAxis::setRange(double lower, double upper)
{
    if (qFuzzyCompare(lower, mRange.lower) &&
        qFuzzyCompare(upper, mRange.upper))
        return;

    if (!QCPRange::validRange(lower, upper))
        return;

    mRange.lower = lower;
    mRange.upper = upper;
    mRange = mRange.sanitizedForLinScale();
}

void CircularAxis::setRangeLower(double lower)
{
    if (qFuzzyCompare(lower, mRange.lower))
        return;

    mRange.lower = lower;
    mRange = mRange.sanitizedForLinScale();
}

void CircularAxis::setRangeUpper(double upper)
{
    if (qFuzzyCompare(upper, mRange.upper))
        return;

    mRange.upper = upper;
    mRange = mRange.sanitizedForLinScale();
}

void CircularAxis::update(QCPLayoutElement::UpdatePhase phase)
{
    QCPLayoutElement::update(phase);

    switch (phase) {
        case upPreparation: {
            setupTickVectors();
            break;
        }
        case upLayout: {
            mCenter = mRect.center();
            mRadius = 0.5 * qMin(qAbs(mRect.width()), qAbs(mRect.height()));
            if (mRadius < 1)
                mRadius = 1; // prevent cases where radius might become 0 which
                             // causes trouble

            break;
        }
        default:
            break;
    }
}

QPen CircularAxis::subTickPen() const { return mSubTickPen; }

void CircularAxis::setSubTickPen(const QPen &subTickPen)
{
    mSubTickPen = subTickPen;
}

QPen CircularAxis::tickPen() const { return mTickPen; }

void CircularAxis::setTickPen(const QPen &tickPen) { mTickPen = tickPen; }

int CircularAxis::subTickLengthOut() const { return mSubTickLengthOut; }

void CircularAxis::setSubTickLengthOut(int outside)
{
    if (mSubTickLengthOut != outside)
        mSubTickLengthOut = outside;
}

int CircularAxis::subTickLengthIn() const { return mSubTickLengthIn; }

void CircularAxis::setSubTickLengthIn(int inside)
{
    if (mSubTickLengthIn != inside)
        mSubTickLengthIn = inside;
}

int CircularAxis::tickLengthOut() const { return mTickLengthOut; }

void CircularAxis::setTickLengthOut(int outside)
{
    if (mTickLengthOut != outside)
        mTickLengthOut = outside;
}

int CircularAxis::tickLengthIn() const { return mTickLengthIn; }

void CircularAxis::setTickLengthIn(int inside)
{
    if (mTickLengthIn != inside)
        mTickLengthIn = inside;
}

bool CircularAxis::subTicks() const { return mSubTicks; }

void CircularAxis::setSubTicks(bool show)
{
    if (mSubTicks != show)
        mSubTicks = show;
}

bool CircularAxis::ticks() const { return mTicks; }

void CircularAxis::setTicks(bool show)
{
    if (mTicks != show)
        mTicks = show;
}

QColor CircularAxis::tickLabelColor() const { return mTickLabelColor; }

void CircularAxis::setTickLabelColor(const QColor &tickLabelColor)
{
    mTickLabelColor = tickLabelColor;
}

QFont CircularAxis::tickLabelFont() const { return mTickLabelFont; }

void CircularAxis::setTickLabelFont(const QFont &tickLabelFont)
{
    mTickLabelFont = tickLabelFont;
}

bool CircularAxis::tickLabels() const { return mTickLabels; }

void CircularAxis::setTickLabels(bool show)
{
    if (mTickLabels != show) {
        mTickLabels = show;
        if (!mTickLabels)
            mTickVectorLabels.clear();
    }
}

QPen CircularAxis::basePen() const { return mBasePen; }

void CircularAxis::setBasePen(const QPen &basePen) { mBasePen = basePen; }

class GaugeRect : public QCPLayoutElement
{
public:
    GaugeRect(Board *dashboard);
    ~GaugeRect() override;

    virtual void update(UpdatePhase phase) override;

    void clearThresholdBands();
    void addLowThreshold(QColor color, double value);
    void addHighThreshold(QColor color, double value);

    bool thresholdsVisible() const;
    void setThresholdsVisible(bool thresholdsVisible);

    bool axisLabelsVisible() const;
    void setAxisLabelsVisible(bool axisLabelsVisible);

    bool axisTicksVisible() const;
    void setAxisTicksVisible(bool axisTicksVisible);

    void setCurrentColor(const QColor &currentColor);

    void setValue(double value);

    QCPRange &axisRange();

    CircularAxis *axis() const;

    bool axisLineVisible() const;
    void setAxisLineVisible(bool axisLineVisible);

    QColor backgroundColor() const;
    void setBackgroundColor(const QColor &backgroundColor);

    bool enabled() const;
    void setEnabled(bool enabled);

    int angleSpan() const;
    void setAngleSpan(int angleSpan);

    int angleStart() const;
    void setAngleStart(int angleStart);

protected:
    Board *mBoard;
    CircularAxis *mAxis;
    QCPRange mAxisRange;
    QColor mCurrentColor;
    QColor mBackgroundColor;
    QColor mNeedleColor;
    double mValue;
    bool mAxisTicksVisible;
    bool mAxisLineVisible;
    bool mAxisLabelsVisible;
    bool mThresholdsVisible;
    bool mEnabled;

    int mAngleSpan;
    int mAngleStart;

    QList<QPair<QColor, double>> mLowThresholdsBandColors;
    QList<QPair<QColor, double>> mHighThresholdsBandColors;

    void drawColorBands(QCPPainter *painter);
    void drawNeedle(QCPPainter *painter);
    void drawCenter(QCPPainter *painter);
    void drawCenterBackground(QCPPainter *painter);
    void drawBackgroundBand(QCPPainter *painter);
    void drawBand(QCPPainter *painter);
    void draw(QCPPainter *painter) override;
};

GaugeRect::GaugeRect(Board *dashboard)
    : QCPLayoutElement(dashboard),
      mBoard(dashboard),
      mAxisRange(QCPRange(0, 0)),
      mCurrentColor(QColor(255, 255, 255, 0)),
      mBackgroundColor(QApplication::palette().light().color()),
      mValue(0),
      mAxisTicksVisible(true),
      mAxisLineVisible(false),
      mAxisLabelsVisible(true),
      mThresholdsVisible(true),
      mEnabled(false),
      mAngleSpan(270),
      mAngleStart(135)
{
    setLayer(QLatin1String("main"));

    mAxis = new CircularAxis(dashboard);
    mAxis->setRange(-0.1, 0.1);
    mAxis->setTickLengthOut(mAxis->tickLengthIn());
    mAxis->setTickLengthIn(0);
    mAxis->setSubTickLengthOut(mAxis->subTickLengthIn());
    mAxis->setSubTickLengthIn(0);
    mAxis->setBasePen(Qt::NoPen);

    mNeedleColor = QApplication::palette().toolTipText().color();
}

GaugeRect::~GaugeRect()
{
    if (mAxis)
        delete mAxis;
}

void GaugeRect::update(QCPLayoutElement::UpdatePhase phase)
{
    if (phase == upLayout)
        mAxis->setOuterRect(mRect);
    mAxis->update(phase);
}

void GaugeRect::clearThresholdBands()
{
    mLowThresholdsBandColors.clear();
    mHighThresholdsBandColors.clear();
}

void GaugeRect::addLowThreshold(QColor color, double value)
{
    mLowThresholdsBandColors.append(QPair<QColor, double>(color, value));
}

void GaugeRect::addHighThreshold(QColor color, double value)
{
    mHighThresholdsBandColors.append(QPair<QColor, double>(color, value));
}

bool GaugeRect::thresholdsVisible() const { return mThresholdsVisible; }

void GaugeRect::setThresholdsVisible(bool thresholdsVisible)
{
    mThresholdsVisible = thresholdsVisible;
}

bool GaugeRect::axisLabelsVisible() const { return mAxisLabelsVisible; }

void GaugeRect::setAxisLabelsVisible(bool axisLabelsVisible)
{
    mAxisLabelsVisible = axisLabelsVisible;
    mAxis->setTickLabels(mAxisLabelsVisible);
}

bool GaugeRect::axisTicksVisible() const { return mAxisTicksVisible; }

void GaugeRect::setAxisTicksVisible(bool axisTicksVisible)
{
    mAxisTicksVisible = axisTicksVisible;
    if (mAxisTicksVisible) {
        mAxis->setTickPen(QPen(mAxis->baseColor(), 0));
        mAxis->setSubTickPen(QPen(mAxis->baseColor(), 0));
    }
    else {
        mAxis->setTickPen(Qt::NoPen);
        mAxis->setSubTickPen(Qt::NoPen);
    }
}

void GaugeRect::setCurrentColor(const QColor &currentColor)
{
    mCurrentColor = currentColor;
}

void GaugeRect::setValue(double value) { mValue = value; }

QCPRange &GaugeRect::axisRange() { return mAxisRange; }

CircularAxis *GaugeRect::axis() const { return mAxis; }

bool GaugeRect::axisLineVisible() const { return mAxisLineVisible; }

void GaugeRect::setAxisLineVisible(bool axisLineVisible)
{
    mAxisLineVisible = axisLineVisible;
    if (mAxisLineVisible) {
        mAxis->setBasePen(QPen(mAxis->baseColor(), 0));
    }
    else {
        mAxis->setBasePen(Qt::NoPen);
    }
}

QColor GaugeRect::backgroundColor() const { return mBackgroundColor; }

void GaugeRect::setBackgroundColor(const QColor &backgroundColor)
{
    mBackgroundColor = backgroundColor;
}

bool GaugeRect::enabled() const { return mEnabled; }

void GaugeRect::setEnabled(bool enabled) { mEnabled = enabled; }

int GaugeRect::angleSpan() const { return mAngleSpan; }

void GaugeRect::setAngleSpan(int angleSpan) { mAngleSpan = angleSpan; }

int GaugeRect::angleStart() const { return mAngleStart; }

void GaugeRect::setAngleStart(int angleStart) { mAngleStart = angleStart; }

void GaugeRect::drawColorBands(QCPPainter *painter)
{
    QPen pen;
    pen.setCapStyle(Qt::FlatCap);
    pen.setWidthF(mAxis->radius() / 20.0);
    painter->setBrush(Qt::NoBrush);

    QRectF bandRect;
    bandRect.setWidth(0.7 * 2 * mAxis->radius());
    bandRect.setHeight(0.7 * 2 * mAxis->radius());
    bandRect.moveCenter(mAxis->center());

    for (int i = 0; i < mLowThresholdsBandColors.size(); i++) {
        int startAngle, currentAngle;
        if (i == 0)
            startAngle = int(-1.0 * mAxis->angleRad() * 180.0 / M_PI) * 16;
        else
            startAngle = int(-1.0 *
                             mAxis->coordToAngleRad(
                                 mLowThresholdsBandColors[i - 1].second) *
                             180.0 / M_PI) *
                         16;

        currentAngle =
            int(-1.0 *
                mAxis->coordToAngleRad(mLowThresholdsBandColors[i].second) *
                180.0 / M_PI) *
            16;

        pen.setColor(mLowThresholdsBandColors[i].first);
        painter->setPen(pen);
        painter->drawArc(bandRect, startAngle, currentAngle - startAngle);
    }

    for (int i = mHighThresholdsBandColors.size() - 1; i >= 0; i--) {
        int startAngle, currentAngle;

        startAngle =
            int(-1.0 *
                mAxis->coordToAngleRad(mHighThresholdsBandColors[i].second) *
                180.0 / M_PI) *
            16;

        if (i == mHighThresholdsBandColors.size() - 1)
            currentAngle =
                int(-1.0 * mAxis->coordToAngleRad(mAxis->range().upper) *
                    180.0 / M_PI) *
                16;
        else
            currentAngle = int(-1.0 *
                               mAxis->coordToAngleRad(
                                   mHighThresholdsBandColors[i + 1].second) *
                               180.0 / M_PI) *
                           16;

        pen.setColor(mHighThresholdsBandColors[i].first);
        painter->setPen(pen);
        painter->drawArc(bandRect, startAngle, currentAngle - startAngle);
    }
}

void GaugeRect::drawNeedle(QCPPainter *painter)
{
    if (!qIsNaN(mValue)) {
        double valueAngle = mAxis->coordToAngleRad(mValue);
        double valueDeg = valueAngle * 180.0 / M_PI - 90;

        painter->save();
        painter->translate(mAxis->center());
        painter->rotate(valueDeg);
        painter->setBrush(QBrush(mNeedleColor));
        painter->setPen(Qt::NoPen);

        QVector<QPointF> tmpPoints;
        tmpPoints.append(QPointF(0.0, 0.0));
        tmpPoints.append(
            QPointF(-mAxis->radius() / 20.0, mAxis->radius() / 20.0));
        tmpPoints.append(QPointF(0.0, mAxis->radius()));
        tmpPoints.append(
            QPointF(mAxis->radius() / 20.0, mAxis->radius() / 20.0));

        painter->drawConvexPolygon(tmpPoints);
        painter->restore();
    }
}

void GaugeRect::drawCenter(QCPPainter *painter)
{
    QRectF centerRect;
    centerRect.setWidth(0.3 * 2 * mAxis->radius());
    centerRect.setHeight(0.3 * 2 * mAxis->radius());
    centerRect.moveCenter(mAxis->center());

    painter->setPen(Qt::NoPen);
    painter->setBrush(QBrush(mBackgroundColor));
    painter->drawEllipse(centerRect);
}

void GaugeRect::drawCenterBackground(QCPPainter *painter)
{
    QRectF centerRect;
    centerRect.setWidth(0.45 * 2 * mAxis->radius());
    centerRect.setHeight(0.45 * 2 * mAxis->radius());
    centerRect.moveCenter(mAxis->center());

    QRadialGradient gradient(mAxis->center(), 0.45 * mAxis->radius(),
                             mAxis->center());

    QColor transp = mBackgroundColor;
    transp.setAlpha(0);
    gradient.setColorAt(0, transp);
    gradient.setColorAt(0.85, transp);
    gradient.setColorAt(1, mBackgroundColor);

    painter->setPen(Qt::NoPen);
    painter->setBrush(QBrush(gradient));
    painter->drawEllipse(centerRect);
}

void GaugeRect::drawBackgroundBand(QCPPainter *painter)
{
    QPen pen;
    pen.setColor(mBackgroundColor);
    pen.setCapStyle(Qt::FlatCap);
    pen.setWidthF(mAxis->radius() / 10.0);
    painter->setBrush(Qt::NoBrush);

    QRectF bandRect;
    bandRect.setWidth(0.8 * 2 * mAxis->radius());
    bandRect.setHeight(0.8 * 2 * mAxis->radius());
    bandRect.moveCenter(mAxis->center());

    int startAngle = int(-1.0 * mAxis->angleRad() * 180.0 / M_PI) * 16;
    int currentAngle = int(-1.0 * mAxis->spanAngle()) * 16;

    painter->setPen(pen);
    painter->drawArc(bandRect, startAngle, currentAngle);
}

void GaugeRect::drawBand(QCPPainter *painter)
{
    QPen pen;
    pen.setColor(mCurrentColor);
    pen.setCapStyle(Qt::FlatCap);
    pen.setWidthF(mAxis->radius() / 10.0);
    painter->setBrush(Qt::NoBrush);

    QRectF bandRect;
    bandRect.setWidth(0.8 * 2 * mAxis->radius());
    bandRect.setHeight(0.8 * 2 * mAxis->radius());
    bandRect.moveCenter(mAxis->center());

    int startAngle = int(-1.0 * mAxis->angleRad() * 180.0 / M_PI) * 16;
    int currentAngle =
        int(-1.0 * mAxis->coordToAngleRad(mValue) * 180.0 / M_PI) * 16;
    painter->setPen(pen);
    painter->drawArc(bandRect, startAngle, currentAngle - startAngle);
}

void GaugeRect::draw(QCPPainter *painter)
{
    drawBackgroundBand(painter);

    if (mThresholdsVisible) {
        drawColorBands(painter);
    }

    if (mEnabled) {
        drawBand(painter);
        drawNeedle(painter);
    }

    drawCenter(painter);
}

///------- GaugeRadial Editor starts from here
class GaugeRadialEditor : public QDialog
{
    Q_OBJECT

public:
    explicit GaugeRadialEditor(GaugeRadial *display, QWidget *parent = nullptr);
    ~GaugeRadialEditor() override;

    void accept() override;

private slots:
    void updateElement();
    void connectProperties(bool connected);
    void newParameter();
    void removeParameter();
    void replaceParameter();

    void on_rangeComboBox_currentIndexChanged(int index);
    void on_listWidget_currentRowChanged(int currentRow);

private:
    Ui::GaugeRadialEditor *ui;
    GaugeRadial *mGauge;
    ElementPropertiesWidget *mPropertiesWidget;
    QToolButton *mNewParamButton;
    QToolButton *mRemoveParamButton;
    QToolButton *mReplaceParamButton;
};

GaugeRadialEditor::GaugeRadialEditor(GaugeRadial *display, QWidget *parent)
    : QDialog(parent), ui(new Ui::GaugeRadialEditor), mGauge(display)
{
    ui->setupUi(this);

    ui->stackedWidget->setCurrentIndex(0);
    ui->listWidget->setCurrentRow(0);

    connect(ui->buttonBox->button(QDialogButtonBox::Apply),
            &QPushButton::clicked, this, &GaugeRadialEditor::updateElement);

    QWidget *cornerWidget = new QWidget(this);
    mNewParamButton = new QToolButton(cornerWidget);
    mNewParamButton->setCursor(Qt::ArrowCursor);
    mNewParamButton->setAutoRaise(true);
    mNewParamButton->setIconSize(QSize(20, 20));
    mNewParamButton->setMinimumSize(QSize(30, 30));
    mNewParamButton->setIcon({}); // FIXME
    connect(mNewParamButton, SIGNAL(clicked()), this, SLOT(newParameter()));
    mNewParamButton->setToolTip(tr("Add parameter"));

    mRemoveParamButton = new QToolButton(cornerWidget);
    mRemoveParamButton->setCursor(Qt::ArrowCursor);
    mRemoveParamButton->setAutoRaise(true);
    mRemoveParamButton->setIconSize(QSize(20, 20));
    mRemoveParamButton->setMinimumSize(QSize(30, 30));
    mRemoveParamButton->setIcon({}); // FIXME
    connect(mRemoveParamButton, SIGNAL(clicked()), this,
            SLOT(removeParameter()));
    mRemoveParamButton->setToolTip(tr("Remove parameter"));
    mRemoveParamButton->setVisible(false);

    mReplaceParamButton = new QToolButton(cornerWidget);
    mReplaceParamButton->setCursor(Qt::ArrowCursor);
    mReplaceParamButton->setAutoRaise(true);
    mReplaceParamButton->setIconSize(QSize(20, 20));
    mReplaceParamButton->setMinimumSize(QSize(30, 30));
    mReplaceParamButton->setIcon({}); // FIXME
    connect(mReplaceParamButton, SIGNAL(clicked()), this,
            SLOT(replaceParameter()));
    mReplaceParamButton->setToolTip(tr("Replace parameter"));
    mReplaceParamButton->setVisible(false);

    auto *lay = new QHBoxLayout(cornerWidget);
    lay->setContentsMargins({});
    lay->addWidget(mReplaceParamButton);
    lay->addWidget(mRemoveParamButton);
    lay->addWidget(mNewParamButton);

    cornerWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    ui->tabWidget->setCornerWidget(cornerWidget, Qt::TopRightCorner);

    mPropertiesWidget = new ElementPropertiesWidget(this);
    mPropertiesWidget->setVisible(false);
    mPropertiesWidget->setProject(display->board()->project());

    connect(mPropertiesWidget, SIGNAL(connectProperties(bool)), this,
            SLOT(connectProperties(bool)));
    connect(ui->tabWidget, &QTabWidget::currentChanged, this, [this]() {
        const bool visible = ui->tabWidget->currentIndex() > 0;
        mRemoveParamButton->setVisible(visible);
        mReplaceParamButton->setVisible(visible);
    });

    if (mGauge) {
        ui->headerCheckBox->setChecked(mGauge->headerVisible());
        ui->valueCheckBox->setChecked(mGauge->valueVisible());
        ui->backgroundComboBox->setCurrentIndex(mGauge->backgroundMode());
        ui->ticksCheckBox->setChecked(mGauge->axisTicksVisible());
        ui->labelsCheckBox->setChecked(mGauge->axisLabelsVisible());
        ui->lineCheckBox->setChecked(mGauge->axisLineVisible());
        ui->rangeComboBox->setCurrentIndex(mGauge->axisScale());
        ui->rangeMinDoubleSpinBox->setValue(mGauge->axisMinCustom());
        ui->rangeMaxDoubleSpinBox->setValue(mGauge->axisMaxCustom());
        ui->positionComboBox->setCurrentIndex(mGauge->gaugePosition());
        ui->startAngleSpinBox->setValue(mGauge->angleStart());
        ui->spanAngleSpinBox->setValue(mGauge->angleSpan());

        QSharedPointer<BoardParameter> dashParam = mGauge->boardParameter(0);
        if (dashParam) {
            if (dashParam->connected()) {
                mPropertiesWidget->setEditionMode(
                    ElementPropertiesWidget::emElementConnected);
                mPropertiesWidget->updateUi(
                    dashParam->sharedParameterConfiguration());
            }
            else {
                if (dashParam->sharedParameterConfiguration())
                    mPropertiesWidget->setEditionMode(
                        ElementPropertiesWidget::emElementDisconnected);
                else
                    mPropertiesWidget->setEditionMode(
                        ElementPropertiesWidget::emElementStandAlone);
                mPropertiesWidget->updateUi(
                    dashParam->exclusiveParameterConfiguration());
            }
            mPropertiesWidget->setPropertiesMode(
                ParameterConfiguration::cmValue);
            mPropertiesWidget->setVisible(true);
            ui->tabWidget->insertTab(1, mPropertiesWidget,
                                     dashParam->getDisplayedLabel());
        }
    }
    setMinimumSize(600, 400);
}

GaugeRadialEditor::~GaugeRadialEditor() { delete ui; }

void GaugeRadialEditor::accept()
{
    updateElement();
    QDialog::accept();
}

void GaugeRadialEditor::updateElement()
{
    if (mGauge) {
        mGauge->setHeaderVisible(ui->headerCheckBox->isChecked());
        mGauge->setValueVisible(ui->valueCheckBox->isChecked());
        mGauge->setBackgroundMode(BoardElement::BackgroundMode(
            ui->backgroundComboBox->currentIndex()));
        mGauge->setAxisTicksVisible(ui->ticksCheckBox->isChecked());
        mGauge->setAxisLabelsVisible(ui->labelsCheckBox->isChecked());
        mGauge->setAxisScale(
            GaugeRadial::AxisScale(ui->rangeComboBox->currentIndex()));
        mGauge->setAxisMinCustom(ui->rangeMinDoubleSpinBox->value());
        mGauge->setAxisMaxCustom(ui->rangeMaxDoubleSpinBox->value());
        mGauge->setAxisLineVisible(ui->lineCheckBox->isChecked());
        mGauge->setAngleSpan(ui->spanAngleSpinBox->value());
        mGauge->setAngleStart(ui->startAngleSpinBox->value());

        mGauge->setGaugePosition(
            GaugeRadial::GaugeRPosition(ui->positionComboBox->currentIndex()));
        QSharedPointer<BoardParameter> dashParam = mGauge->boardParameter(0);
        if (dashParam) {
            if (!mPropertiesWidget->isConnected()) {
                mPropertiesWidget->updateParameterSettings(
                    dashParam->exclusiveParameterConfiguration());
                dashParam->disconnectSharedConfiguration();
            }
            else {
                dashParam->setSharedParameterConfiguration(
                    mPropertiesWidget->currentSettings(), true);
            }
        }

        mGauge->rebuildElement(true);
    }
}

void GaugeRadialEditor::connectProperties(bool connected)
{
    QSharedPointer<BoardParameter> dashParam = mGauge->boardParameter(0);
    if (dashParam) {
        if (connected) {
            mPropertiesWidget->setEditionMode(
                ElementPropertiesWidget::emElementConnected);
            mPropertiesWidget->updateUi(
                dashParam->sharedParameterConfiguration());
        }
        else {
            mPropertiesWidget->setEditionMode(
                ElementPropertiesWidget::emElementDisconnected);
            mPropertiesWidget->updateUi(
                dashParam->exclusiveParameterConfiguration());
        }
        mPropertiesWidget->setPropertiesMode(ParameterConfiguration::cmValue);
    }
}

void GaugeRadialEditor::newParameter()
{
    QString paramLabel =
        QInputDialog::getText(this, "Parameter Label", "Parameter Label");
    if (!paramLabel.isEmpty()) {
        mGauge->addParameter(paramLabel);

        QSharedPointer<BoardParameter> dashParam = mGauge->boardParameter(0);
        if (dashParam) {
            mPropertiesWidget->setEditionMode(
                ElementPropertiesWidget::emElementStandAlone);
            mPropertiesWidget->setPropertiesMode(
                ParameterConfiguration::cmValue);
            mPropertiesWidget->updateUi(
                dashParam->exclusiveParameterConfiguration());
            mPropertiesWidget->setVisible(true);
            ui->tabWidget->insertTab(1, mPropertiesWidget,
                                     dashParam->getDisplayedLabel());
        }

        if (!mGauge->board()->dataManager()->liveDataEnabled()) {
            mGauge->board()->resetPlayback();
            mGauge->board()->replot(QCustomPlot::rpQueuedReplot);
        }
    }
}

void GaugeRadialEditor::removeParameter()
{
    mGauge->removeBoardParameter(0);
    ui->tabWidget->removeTab(1);
}

void GaugeRadialEditor::replaceParameter()
{
    if (ui->tabWidget->currentIndex() > 0) {
        QString paramLabel =
            QInputDialog::getText(this, "Parameter Label", "Parameter Label");
        if (!paramLabel.isEmpty()) {
            QSharedPointer<BoardParameter> dashParam = mGauge->replaceParameter(
                ui->tabWidget->currentIndex() - 1, paramLabel);
            if (dashParam) {
                mPropertiesWidget->setEditionMode(
                    ElementPropertiesWidget::emElementStandAlone);
                mPropertiesWidget->setPropertiesMode(
                    ParameterConfiguration::cmValue);
                mPropertiesWidget->updateUi(
                    dashParam->exclusiveParameterConfiguration());
                mPropertiesWidget->setVisible(true);
                ui->tabWidget->insertTab(1, mPropertiesWidget,
                                         dashParam->getDisplayedLabel());
            }
            if (!mGauge->board()->dataManager()->liveDataEnabled()) {
                mGauge->board()->resetPlayback();
                mGauge->board()->replot(QCustomPlot::rpQueuedReplot);
            }
        }
    }
}

void GaugeRadialEditor::on_rangeComboBox_currentIndexChanged(int index)
{
    ui->scaleWidget->setEnabled(index == 2);
}

void GaugeRadialEditor::on_listWidget_currentRowChanged(int currentRow)
{
    ui->stackedWidget->setCurrentIndex(currentRow);
}

///------- GaugeRadial starts from here
GaugeRadial::GaugeRadial(Board *dashboard)
    : NumericalDisplay(dashboard),
      mAxisScale(asAuto),
      mGaugePosition(gpMiddle),
      mAngleSpan(270),
      mAngleStart(0)
{
}

void GaugeRadial::clearElement()
{
    if (mGauge)
        delete mGauge;

    NumericalDisplay::clearElement();
}

void GaugeRadial::initialize(Board *dashboard)
{
    if (dashboard) {
        mGauge = new GaugeRect(dashboard);
        mGauge->axis()->setRange(QCPRange(0., 0.1));
        mGauge->axisRange().lower = 0.;
        mGauge->axisRange().upper = .1;
        mGauge->setAngleSpan(mAngleSpan);
        mGauge->setAngleStart(mAngleStart - 90);
        mGauge->axis()->setSpanAngle(mAngleSpan);
        mGauge->axis()->setAngle(mAngleStart - 90);

        NumericalDisplay::initialize(dashboard);
    }
}

void GaugeRadial::edit()
{
    GaugeRadialEditor editor(this);
    editor.exec();
}

void GaugeRadial::loadSettings(QSettings *settings)
{
    SingleDisplay::loadSettings(settings);

    settings->beginGroup("Axis");
    if (settings->contains("TicksVisible"))
        setAxisTicksVisible(settings->value("TicksVisible").toBool());
    if (settings->contains("LabelsVisible"))
        setAxisLabelsVisible(settings->value("LabelsVisible").toBool());
    if (settings->contains("LineVisible"))
        setAxisLineVisible(settings->value("LineVisible").toBool());
    if (settings->contains("ScaleMode"))
        setAxisScale(AxisScale(settings->value("ScaleMode").toInt()));
    if (settings->contains("ScaleMin"))
        setAxisMinCustom(settings->value("ScaleMin").toDouble());
    if (settings->contains("ScaleMax"))
        setAxisMaxCustom(settings->value("ScaleMax").toDouble());
    if (settings->contains("AngleSpan"))
        setAngleSpan(settings->value("AngleSpan").toInt());
    if (settings->contains("AngleStart"))
        setAngleStart(settings->value("AngleStart").toInt());
    settings->endGroup();

    settings->beginGroup("Gauge");
    if (settings->contains("Position"))
        setGaugePosition(GaugeRPosition(settings->value("Position").toInt()));
    settings->endGroup();
}

void GaugeRadial::saveSettings(QSettings *settings)
{
    SingleDisplay::saveSettings(settings);

    settings->beginGroup("Axis");
    settings->setValue("TicksVisible", axisTicksVisible());
    settings->setValue("LineVisible", axisLineVisible());
    settings->setValue("LabelsVisible", axisLabelsVisible());
    settings->setValue("ScaleMode", axisScale());
    settings->setValue("ScaleMin", axisMinCustom());
    settings->setValue("ScaleMax", axisMaxCustom());
    settings->setValue("AngleSpan", angleSpan());
    settings->setValue("AngleStart", angleStart());
    settings->endGroup();

    settings->beginGroup("Gauge");
    settings->setValue("Position", mGaugePosition);
    settings->endGroup();
}

void GaugeRadial::buildElement()
{
    mGauge->setAngleSpan(mAngleSpan);
    mGauge->setAngleStart(mAngleStart - 90);
    mGauge->axis()->setSpanAngle(mAngleSpan);
    mGauge->axis()->setAngle(mAngleStart - 90);

    mMainLayout->setFillOrder(QCPLayoutGrid::foRowsFirst);
    mMainLayout->setRowSpacing(0);

    mTextLabel->setVisible(mHeaderVisible);
    mTextUnit->setVisible(mHeaderVisible);
    mTextValue->setVisible(mValueVisible);

    switch (mGaugePosition) {
        case gpTop:
            mMainLayout->addElement(mGauge);
            mMainLayout->setRowStretchFactor(mMainLayout->elementCount() - 1,
                                             1);

            if (mHeaderVisible) {
                mMainLayout->addElement(mTextLabel);
                mMainLayout->setRowStretchFactor(
                    mMainLayout->elementCount() - 1, 0.001);
                mMainLayout->addElement(mTextUnit);
                mMainLayout->setRowStretchFactor(
                    mMainLayout->elementCount() - 1, 0.001);
            }

            if (mValueVisible) {
                mMainLayout->addElement(mTextValue);
                mMainLayout->setRowStretchFactor(
                    mMainLayout->elementCount() - 1, 0.001);
            }
            break;
        case gpMiddle:
            if (mHeaderVisible) {
                mMainLayout->addElement(mTextLabel);
                mMainLayout->setRowStretchFactor(
                    mMainLayout->elementCount() - 1, 0.001);
                mMainLayout->addElement(mTextUnit);
                mMainLayout->setRowStretchFactor(
                    mMainLayout->elementCount() - 1, 0.001);
            }

            mMainLayout->addElement(mGauge);
            mMainLayout->setRowStretchFactor(mMainLayout->elementCount() - 1,
                                             1);

            if (mValueVisible) {
                mMainLayout->addElement(mTextValue);
                mMainLayout->setRowStretchFactor(
                    mMainLayout->elementCount() - 1, 0.001);
            }
            break;
        case gpBottom:
            if (mHeaderVisible) {
                mMainLayout->addElement(mTextLabel);
                mMainLayout->setRowStretchFactor(
                    mMainLayout->elementCount() - 1, 0.001);
                mMainLayout->addElement(mTextUnit);
                mMainLayout->setRowStretchFactor(
                    mMainLayout->elementCount() - 1, 0.001);
            }

            if (mValueVisible) {
                mMainLayout->addElement(mTextValue);
                mMainLayout->setRowStretchFactor(
                    mMainLayout->elementCount() - 1, 0.001);
            }

            mMainLayout->addElement(mGauge);
            mMainLayout->setRowStretchFactor(mMainLayout->elementCount() - 1,
                                             1);
            break;
    }

    mMainLayout->needUpdate(true);

    switch (mBackgroundMode) {
        case bmBackground:
            mGauge->setBackgroundColor(QApplication::palette().light().color());
            break;
        case bmTransparent:
            mGauge->setBackgroundColor(QApplication::palette().dark().color());
            break;
    }

    mGauge->clearThresholdBands();
    QSharedPointer<BoardParameter> dashParam = boardParameter(0);
    if (dashParam) {
        QMap<double, ColorSettings> lowThr = dashParam->parameterConfiguration()
                                                 ->thresholdsSettingsRef()
                                                 .lowThresholds();
        QMap<double, ColorSettings>::iterator lowIt;
        for (lowIt = lowThr.begin(); lowIt != lowThr.end(); ++lowIt) {
            QColor color = lowIt.value().color();
            color.setAlpha(150);
            mGauge->addLowThreshold(color, lowIt.key());
        }

        QMap<double, ColorSettings> highThr =
            dashParam->parameterConfiguration()
                ->thresholdsSettingsRef()
                .highThresholds();
        QMap<double, ColorSettings>::iterator highIt;
        for (highIt = highThr.begin(); highIt != highThr.end(); ++highIt) {
            QColor color = highIt.value().color();
            color.setAlpha(150);
            mGauge->addHighThreshold(color, highIt.key());
        }
    }
}

void GaugeRadial::checkParameters()
{
    NumericalDisplay::checkParameters();

    mGauge->setEnabled(false);
    QSharedPointer<BoardParameter> dashParam = boardParameter(0);
    if (dashParam) {
        if (dashParam->getTimeSeries().parameterId() > 0) {
            mGauge->setEnabled(true);
            return;
        }
    }
}

void GaugeRadial::displayData()
{
    QSharedPointer<BoardParameter> dashParam = boardParameter(0);
    if (dashParam) {
        if (dashParam->getTimeSeries().parameterId() > 0) {
            dashParam->processValueData();
            if (mValueVisible) {
                mTextValue->setText(dashParam->getValueString(mValueFormat));
                mTextValue->setTextColor(dashParam->getForegroundColor());
                mTextValue->setBackgroundBrush(dashParam->getBackgroundBrush());
            }

            QColor colorBar;
            double value = dashParam->getValueDouble();
            switch (dashParam->parameterConfiguration()->itemColorMode()) {
                case ParameterConfiguration::icBase:
                    colorBar = (dashParam->parameterConfiguration()
                                    ->defaultColorSettingsRef()
                                    .color());
                    break;
                case ParameterConfiguration::icDynamic:
                case ParameterConfiguration::icDynamicSegmented:
                    colorBar = (dashParam->getColor());
                    break;
                case ParameterConfiguration::icCustom:
                    colorBar = (dashParam->parameterConfiguration()
                                    ->itemStaticColor());
                    break;
            }

            mGauge->setCurrentColor(colorBar);

            mGauge->setValue(value);

            if (value < mGauge->axisRange().lower)
                mGauge->axisRange().lower = value;
            if (value > mGauge->axisRange().upper)
                mGauge->axisRange().upper = value;
        }

        switch (mAxisScale) {
            case asAuto:
                mGauge->axis()->setRange(mGauge->axisRange());
                mAxisMinCustom = mGauge->axisRange().lower;
                mAxisMaxCustom = mGauge->axisRange().upper;
                break;
            case asParam: {
                if (dashParam->parameterConfiguration()->validRange()) {
                    mGauge->axis()->setRange(QCPRange(
                        dashParam->parameterConfiguration()->rangeMinimum(),
                        dashParam->parameterConfiguration()->rangeMaximum()));
                }
                else {
                    mGauge->axis()->setRange(mGauge->axisRange());
                    mAxisMinCustom = mGauge->axisRange().lower;
                    mAxisMaxCustom = mGauge->axisRange().upper;
                }
                break;
            }
            case asCustom:
                mGauge->axis()->setRange(
                    QCPRange(mAxisMinCustom, mAxisMaxCustom));
                break;
        }
    }
}

void GaugeRadial::loadHistoricalData()
{
    QSharedPointer<BoardParameter> dashParam = boardParameter(0);
    if (dashParam) {
        QVector<DataValue> values;
        QVector<double> timestamps;
        double min = (std::numeric_limits<double>::max)();
        double max = -1. * (std::numeric_limits<double>::max)();

        if (mBoard->dataManager()->lastBoardData(
                dashParam->getTimeSeries().parameterId(), values, timestamps)) {
            QVector<DataValue>::const_iterator itValues;
            QVector<double>::const_iterator itTimestamps;

            for (itValues = values.constBegin(),
                itTimestamps = timestamps.constBegin();
                 (itValues != values.constEnd()) &&
                 (itTimestamps != timestamps.constEnd());
                 ++itValues, ++itTimestamps) {
                auto val = itValues->toDouble();
                if (val < min)
                    min = val;
                if (val > max)
                    max = val;
            }

            mGauge->axisRange().lower = min;
            mGauge->axisRange().upper = max;
        }
        dashParam->updateData();
    }
}

GaugeRadial::AxisScale GaugeRadial::axisScale() const { return mAxisScale; }

double GaugeRadial::axisMaxCustom() const { return mAxisMaxCustom; }

double GaugeRadial::axisMinCustom() const { return mAxisMinCustom; }

void GaugeRadial::setAxisMinCustom(double axisMinCustom)
{
    mAxisMinCustom = axisMinCustom;
}

bool GaugeRadial::axisTicksVisible() const
{
    return mGauge->axisTicksVisible();
}

bool GaugeRadial::axisLineVisible() const { return mGauge->axisLineVisible(); }

void GaugeRadial::setAxisTicksVisible(bool axisTicksVisible)
{
    mGauge->setAxisTicksVisible(axisTicksVisible);
}

void GaugeRadial::setAxisLineVisible(bool axisLineVisible)
{
    mGauge->setAxisLineVisible(axisLineVisible);
}

bool GaugeRadial::axisLabelsVisible() const
{
    return mGauge->axisLabelsVisible();
}

void GaugeRadial::setAxisLabelsVisible(bool axisLabelsVisible)
{
    mGauge->setAxisLabelsVisible(axisLabelsVisible);
}

bool GaugeRadial::thresholdsVisible() const
{
    return mGauge->thresholdsVisible();
}

void GaugeRadial::setThresholdsVisible(bool thresholdsVisible)
{
    mGauge->setThresholdsVisible(thresholdsVisible);
}

void GaugeRadial::setAxisScale(AxisScale axisScale) { mAxisScale = axisScale; }

void GaugeRadial::setAxisMaxCustom(double axisMaxCustom)
{
    mAxisMaxCustom = axisMaxCustom;
}

GaugeRadial::GaugeRPosition GaugeRadial::gaugePosition() const
{
    return mGaugePosition;
}

void GaugeRadial::setGaugePosition(GaugeRPosition gaugePosition)
{
    mGaugePosition = gaugePosition;
}

int GaugeRadial::angleSpan() const { return mAngleSpan; }

void GaugeRadial::setAngleSpan(int angleSpan) { mAngleSpan = angleSpan; }

int GaugeRadial::angleStart() const { return mAngleStart; }

void GaugeRadial::setAngleStart(int angleStart) { mAngleStart = angleStart; }

int GaugeRadial::defaultWidth()
{
    return 3 * mBoard->dashboardLayout()->singleElementColumnCount();
}

int GaugeRadial::defaultHeight()
{
    return 4 * mBoard->dashboardLayout()->singleElementRowCount();
}

} // namespace tl

#include "GaugeRadial.moc"
