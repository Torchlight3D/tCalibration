#include "qtwaitingspinner.h"

#include <numbers>

#include <QPainter>
#include <QTimer>

namespace tl {

using std::numbers::pi;

namespace {

int lineCountDistanceFromPrimary(int current, int primary, int numLines)
{
    int distance = primary - current;
    if (distance < 0) {
        distance += numLines;
    }
    return distance;
}

QColor currentLineColor(int distance, int numLines, qreal trailFadePerc,
                        qreal minOpacity, const QColor &color)
{
    if (distance == 0) {
        return color;
    }

    auto newColor = color;

    const qreal minAlphaF = minOpacity / 100.;
    const auto distanceThreshold =
        static_cast<int>(std::ceil((numLines - 1) * trailFadePerc / 100.));
    if (distance > distanceThreshold) {
        newColor.setAlphaF(minAlphaF);
    }
    else {
        qreal alphaDiff = color.alphaF() - minAlphaF;
        qreal gradient = alphaDiff / (distanceThreshold + 1);
        qreal resultAlpha = color.alphaF() - gradient * distance;
        newColor.setAlphaF(std::clamp(resultAlpha, 0., 1.));
    }
    return newColor;
}

} // namespace

///------- QtWaitingSpinnerPrivate starts from here
class QtWaitingSpinnerPrivate
{
    Q_DEFINE_PIMPL(QtWaitingSpinner)

public:
    QtWaitingSpinnerPrivate(QtWaitingSpinner *q, bool centerOnParent,
                            bool disableParentWhenSpinning);

    void init();

    void updateSize();
    void updateTimer();
    void updatePosition();

    QSize spinnerSize() const;

public:
    QColor m_color;
    QString m_text;
    QTimer *m_timer;
    qreal m_roundness; // 0..100
    qreal m_minimumTrailOpacity;
    qreal m_trailFadePercentage;
    qreal m_revolutionsPerSecond;
    int m_numLines;
    int m_lineLength;
    int m_lineWidth;
    int m_innerRadius;
    int m_currentCounter;
    bool m_centerOnParent;
    bool m_disableParentWhenSpinning;
};

QtWaitingSpinnerPrivate::QtWaitingSpinnerPrivate(QtWaitingSpinner *q,
                                                 bool centerOnParent,
                                                 bool disableParentWhenSpinning)
    : q_ptr(q),
      m_color(Qt::black),
      m_timer(new QTimer(q)),
      m_trailFadePercentage(80.),
      m_minimumTrailOpacity(pi),
      m_revolutionsPerSecond(pi * 0.5),
      m_roundness(100.),
      m_numLines(20),
      m_lineWidth(2),
      m_lineLength(10),
      m_innerRadius(10),
      m_currentCounter(0),
      m_centerOnParent(centerOnParent),
      m_disableParentWhenSpinning(disableParentWhenSpinning)
{
}

void QtWaitingSpinnerPrivate::init()
{
    Q_Q(QtWaitingSpinner);
    updateSize();
    updateTimer();
    q->hide();
}

void QtWaitingSpinnerPrivate::updateSize()
{
    Q_Q(QtWaitingSpinner);
    const auto spinnerSize = this->spinnerSize();

    if (m_text.isEmpty()) {
        q->setFixedSize(spinnerSize);
    }
    else {
        const auto textSize =
            QFontMetrics{q->font()}.boundingRect(m_text).size();

        q->setFixedSize(std::max(spinnerSize.width(), textSize.width()),
                        spinnerSize.height() + spinnerSize.height() / 4 +
                            textSize.height());
    }
}

void QtWaitingSpinnerPrivate::updateTimer()
{
    m_timer->setInterval(1000 / (m_numLines * m_revolutionsPerSecond));
}

void QtWaitingSpinnerPrivate::updatePosition()
{
    Q_Q(QtWaitingSpinner);
    if (q->parentWidget() && m_centerOnParent) {
        q->move(q->parentWidget()->width() / 2 - q->width() / 2,
                q->parentWidget()->height() / 2 - q->height() / 2);
    }
}

QSize QtWaitingSpinnerPrivate::spinnerSize() const
{
    const auto edge = (m_innerRadius + m_lineLength) * 2;
    return {edge, edge};
}

///------- QtWaitingSpinner starts from here
QtWaitingSpinner::QtWaitingSpinner(QWidget *parent, bool centerOnParent,
                                   bool disableParentWhenSpinning)
    : QWidget(parent),
      d_ptr(new QtWaitingSpinnerPrivate(this, centerOnParent,
                                        disableParentWhenSpinning))
{
    Q_D(QtWaitingSpinner);
    d->init();

    connect(d->m_timer, &QTimer::timeout, this, &QtWaitingSpinner::rotate);
}

QtWaitingSpinner::QtWaitingSpinner(Qt::WindowModality modality, QWidget *parent,
                                   bool centerOnParent,
                                   bool disableParentWhenSpinning)
    : QWidget(parent, Qt::Dialog | Qt::FramelessWindowHint),
      d_ptr(new QtWaitingSpinnerPrivate(this, centerOnParent,
                                        disableParentWhenSpinning))
{
    Q_D(QtWaitingSpinner);
    d->init();

    // We need to set the window modality AFTER we've hidden the
    // widget for the first time since changing this property while
    // the widget is visible has no effect.
    setWindowModality(modality);
    setAttribute(Qt::WA_TranslucentBackground);

    connect(d->m_timer, &QTimer::timeout, this, &QtWaitingSpinner::rotate);
}

QtWaitingSpinner::~QtWaitingSpinner() = default;

QColor QtWaitingSpinner::color() const
{
    Q_D(const QtWaitingSpinner);
    return d->m_color;
}

void QtWaitingSpinner::setColor(const QColor &color)
{
    Q_D(QtWaitingSpinner);
    d->m_color = color;
}

qreal QtWaitingSpinner::roundness() const
{
    Q_D(const QtWaitingSpinner);
    return d->m_roundness;
}

void QtWaitingSpinner::setRoundness(qreal roundness)
{
    Q_D(QtWaitingSpinner);
    d->m_roundness = std::clamp(roundness, 0., 100.);
}

qreal QtWaitingSpinner::minimumTrailOpacity() const
{
    Q_D(const QtWaitingSpinner);
    return d->m_minimumTrailOpacity;
}

void QtWaitingSpinner::setMinimumTrailOpacity(qreal val)
{
    Q_D(QtWaitingSpinner);
    d->m_minimumTrailOpacity = val;
}

qreal QtWaitingSpinner::trailFadePercentage() const
{
    Q_D(const QtWaitingSpinner);
    return d->m_trailFadePercentage;
}

void QtWaitingSpinner::setTrailFadePercentage(qreal val)
{
    Q_D(QtWaitingSpinner);
    d->m_trailFadePercentage = val;
}

qreal QtWaitingSpinner::revolutionsPersSecond() const
{
    Q_D(const QtWaitingSpinner);
    return d->m_revolutionsPerSecond;
}

void QtWaitingSpinner::setRevolutionsPerSecond(qreal val)
{
    Q_D(QtWaitingSpinner);
    d->m_revolutionsPerSecond = val;
    d->updateTimer();
}

int QtWaitingSpinner::numberOfLines() const
{
    Q_D(const QtWaitingSpinner);
    return d->m_numLines;
}

void QtWaitingSpinner::setNumberOfLines(int lines)
{
    Q_D(QtWaitingSpinner);
    d->m_numLines = lines;
    d->m_currentCounter = 0;
    d->updateTimer();
}

int QtWaitingSpinner::lineLength() const
{
    Q_D(const QtWaitingSpinner);
    return d->m_lineLength;
}

void QtWaitingSpinner::setLineLength(int length)
{
    Q_D(QtWaitingSpinner);
    d->m_lineLength = length;
    d->updateSize();
}

int QtWaitingSpinner::lineWidth() const
{
    Q_D(const QtWaitingSpinner);
    return d->m_lineWidth;
}

void QtWaitingSpinner::setLineWidth(int width)
{
    Q_D(QtWaitingSpinner);
    d->m_lineWidth = width;
    d->updateSize();
}

int QtWaitingSpinner::innerRadius() const
{
    Q_D(const QtWaitingSpinner);
    return d->m_innerRadius;
}

void QtWaitingSpinner::setInnerRadius(int radius)
{
    Q_D(QtWaitingSpinner);
    d->m_innerRadius = radius;
    d->updateSize();
}

QString QtWaitingSpinner::text() const
{
    Q_D(const QtWaitingSpinner);
    return d->m_text;
}

void QtWaitingSpinner::setText(const QString &text)
{
    if (text.isEmpty()) {
        return;
    }

    Q_D(QtWaitingSpinner);
    d->m_text = text;
    d->updateSize();
}

void QtWaitingSpinner::start()
{
    Q_D(QtWaitingSpinner);
    d->updatePosition();
    show();

    if (parentWidget() && d->m_disableParentWhenSpinning) {
        parentWidget()->setEnabled(false);
    }

    if (!d->m_timer->isActive()) {
        d->m_timer->start();
        d->m_currentCounter = 0;
    }
}

void QtWaitingSpinner::stop()
{
    Q_D(QtWaitingSpinner);
    hide();

    if (parentWidget() && d->m_disableParentWhenSpinning) {
        parentWidget()->setEnabled(true);
    }

    if (d->m_timer->isActive()) {
        d->m_timer->stop();
        d->m_currentCounter = 0;
    }
}

bool QtWaitingSpinner::isSpinning() const
{
    Q_D(const QtWaitingSpinner);
    return isVisible();
}

void QtWaitingSpinner::paintEvent(QPaintEvent * /*event*/)
{
    Q_D(QtWaitingSpinner);
    d->updatePosition();

    QPainter painter{this};
    painter.fillRect(rect(), Qt::transparent);
    painter.setRenderHint(QPainter::Antialiasing, true);

    if (d->m_currentCounter >= d->m_numLines) {
        d->m_currentCounter = 0;
    }

    const auto spinnerSize = d->spinnerSize();

    painter.setPen(Qt::NoPen);
    for (int i = 0; i < d->m_numLines; ++i) {
        painter.save();
        painter.translate(d->m_innerRadius + d->m_lineLength,
                          d->m_innerRadius + d->m_lineLength);
        painter.translate((width() - spinnerSize.width()) / 2, 0);

        const qreal rotateAngle = 360. * i / d->m_numLines;
        painter.rotate(rotateAngle);
        painter.translate(d->m_innerRadius, 0);

        const int distance =
            lineCountDistanceFromPrimary(i, d->m_currentCounter, d->m_numLines);
        const auto color =
            currentLineColor(distance, d->m_numLines, d->m_trailFadePercentage,
                             d->m_minimumTrailOpacity, d->m_color);
        painter.setBrush(color);
        painter.drawRoundedRect(
            QRect{0, -d->m_lineWidth / 2, d->m_lineLength, d->m_lineWidth},
            d->m_roundness, d->m_roundness, Qt::RelativeSize);
        painter.restore();
    }

    if (!d->m_text.isEmpty()) {
        painter.setPen(Qt::black);
        painter.drawText(QRect{0, spinnerSize.height(), width(),
                               height() - spinnerSize.height()},
                         Qt::AlignBottom | Qt::AlignHCenter, d->m_text);
    }
}

void QtWaitingSpinner::rotate()
{
    Q_D(QtWaitingSpinner);
    ++d->m_currentCounter;
    if (d->m_currentCounter >= d->m_numLines) {
        d->m_currentCounter = 0;
    }
    update();
}

} // namespace tl

#include "moc_qtwaitingspinner.cpp"
