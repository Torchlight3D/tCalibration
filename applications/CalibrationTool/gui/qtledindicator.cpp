#include "qtledindicator.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QTimer>

namespace tl {

// In case there's more states
namespace State {
constexpr bool On{true};
constexpr bool Off{false};
} // namespace State

///------- QtLedIndicatorPrivate starts from here
class QtLedIndicatorPrivate
{
    Q_DEFINE_PIMPL(QtLedIndicator)

public:
    explicit QtLedIndicatorPrivate(QtLedIndicator *q);

    void init();

public:
    struct StateColors
    {
        const QColor &of(bool state) const
        {
            return state ? onColor : offColor;
        }

        QColor onColor = QtLedIndicator::defaultOnColor();
        QColor offColor = QtLedIndicator::defaultOffColor();
    };

    struct Blink
    {
        QTimer timer;
        bool on{State::On};
    };

    StateColors m_stateColors;
    Blink m_blink;
    QtLedIndicator::Shape m_shape{QtLedIndicator::RoundRect};
    bool m_state{State::Off};
    bool m_clickable{false};
    bool m_flat{false};
};

QtLedIndicatorPrivate::QtLedIndicatorPrivate(QtLedIndicator *q) : q_ptr(q) {}

void QtLedIndicatorPrivate::init()
{
    Q_Q(QtLedIndicator);
    m_blink.timer.callOnTimeout([this, q]() {
        m_blink.on = !m_blink.on;
        q->repaint();
    });
    m_blink.timer.setInterval(500);
}

///------- QtLedIndicator starts from here
QtLedIndicator::QtLedIndicator(QWidget *parent)
    : QWidget(parent), d_ptr(new QtLedIndicatorPrivate(this))
{
    d_ptr->init();
}

QtLedIndicator::~QtLedIndicator() = default;

void QtLedIndicator::setSize(const QSize &size) { setFixedSize(size); }

QtLedIndicator::Shape QtLedIndicator::shape() const
{
    Q_D(const QtLedIndicator);
    return d->m_shape;
}

void QtLedIndicator::setShape(Shape shape)
{
    Q_D(QtLedIndicator);
    if (d->m_shape == shape) {
        return;
    }

    d->m_shape = shape;
    repaint();
}

const QColor &QtLedIndicator::onColor() const
{
    Q_D(const QtLedIndicator);
    return d->m_stateColors.onColor;
}

void QtLedIndicator::setOnColor(const QColor &color)
{
    Q_D(QtLedIndicator);
    if (d->m_stateColors.onColor == color) {
        return;
    }

    d->m_stateColors.onColor = color;
    repaint();
}

QColor QtLedIndicator::defaultOnColor() { return Qt::green; }

const QColor &QtLedIndicator::offColor() const
{
    Q_D(const QtLedIndicator);
    return d->m_stateColors.offColor;
}

void QtLedIndicator::setOffColor(const QColor &color)
{
    Q_D(QtLedIndicator);
    if (d->m_stateColors.offColor == color) {
        return;
    }

    d->m_stateColors.offColor = color;
    repaint();
}

QColor QtLedIndicator::defaultOffColor() { return Qt::lightGray; }

bool QtLedIndicator::on() const
{
    Q_D(const QtLedIndicator);
    return d->m_state;
}

void QtLedIndicator::setOn(bool state)
{
    Q_D(QtLedIndicator);
    if (state == d->m_state) {
        return;
    }

    d->m_state = state;
    repaint();
}

bool QtLedIndicator::blinking() const
{
    Q_D(const QtLedIndicator);
    return d->m_blink.timer.isActive();
}

void QtLedIndicator::setBlinking(bool on)
{
    Q_D(QtLedIndicator);
    if (on) {
        d->m_blink.timer.setInterval(d->m_blink.timer.interval());
        d->m_blink.timer.start();
        d->m_blink.on = on;
    }
    else {
        // Blinking shall not influence the normal output now.
        d->m_blink.timer.stop();
        d->m_blink.on = true;
    }
    repaint();
}

int QtLedIndicator::blinkCycle() const
{
    Q_D(const QtLedIndicator);
    return d->m_blink.timer.interval();
}

void QtLedIndicator::setBlinkCycle(int cycle)
{
    Q_D(QtLedIndicator);
    d->m_blink.timer.setInterval(cycle);
}

void QtLedIndicator::setStatus(Status status)
{
    switch (status) {
        case Status::Off:
            setOn(false);
            break;
        case Status::On:
            setOn(true);
            setBlinking(false);
            break;
        case Status::Blink:
            setOn(true);
            setBlinking(true);
            break;
        default:
            break;
    }
}

bool QtLedIndicator::clickable() const
{
    Q_D(const QtLedIndicator);
    return d->m_clickable;
}

void QtLedIndicator::setClickable(bool on)
{
    Q_D(QtLedIndicator);
    d->m_clickable = on;
}

bool QtLedIndicator::flat() const
{
    Q_D(const QtLedIndicator);
    return d->m_flat;
}

void QtLedIndicator::setFlat(bool on)
{
    Q_D(QtLedIndicator);
    d->m_flat = on;
}

void QtLedIndicator::setState(State state)
{
    switch (state) {
        case State::Idle:
            setStatus(Status::Off);
            break;
        case State::Busy:
            setOnColor(Qt::green);
            setStatus(Status::Blink);
            break;
        case State::Warning:
            setOnColor(Qt::yellow);
            setStatus(Status::Blink);
            break;
        case State::Success:
            setOnColor(Qt::green);
            setStatus(Status::On);
            break;
        case State::Failure:
            setOnColor(Qt::red);
            setStatus(Status::On);
            break;
        default:
            break;
    }
}

void QtLedIndicator::paintEvent(QPaintEvent * /*event*/)
{
    Q_D(QtLedIndicator);
    const auto &paintColor = d->m_stateColors.of(d->m_blink.on && d->m_state);

    QPainter painter{this};
    painter.setRenderHint(QPainter::Antialiasing);

    const QRect paintArea{0, 0, width() - 1, height() - 1};
    const QPoint center{paintArea.width() / 2, paintArea.height() / 2};

    QBrush brush{paintColor};
    if (!d->m_flat) {
        QRadialGradient grad;
        if (d->m_shape != Triangle) {
            grad = {QPointF{width() / 3., height() / 3.}, center.x() / 2.};
        }
        else {
            // Triangle exception: 1/3 light reflex would be out of led here. So
            // here is the 1/2 tri exception.
            grad = {QPointF{width() / 2., height() / 2.}, center.x() / 2.};
        }

        grad.setColorAt(0., paintColor.lighter(150));
        grad.setColorAt(1., paintColor);

        QBrush gradBrush{grad};
        brush.swap(gradBrush);
    }

    painter.setPen(Qt::NoPen);
    painter.setBrush(brush);

    switch (d->m_shape) {
        case Elipse:
            painter.drawEllipse(center, center.x(), center.y());
            break;
        case Rect:
            painter.drawRect(paintArea);
            break;
        case RoundRect: {
            constexpr qreal kRadius{5.};
            painter.drawRoundedRect(paintArea, kRadius, kRadius);
        } break;
        case Triangle: {
            const auto rect = paintArea.toRectF();
            const QPointF apex{rect.left() + rect.width() * 0.5, rect.top()};

            QPainterPath path;
            path.moveTo(apex);
            path.lineTo(rect.bottomLeft());
            path.lineTo(rect.bottomRight());
            path.lineTo(apex);

            painter.drawPath(path);
            painter.fillPath(path, brush);
        } break;
    }
}

void QtLedIndicator::mousePressEvent(QMouseEvent *event)
{
    Q_D(QtLedIndicator);
    if (!d->m_clickable) {
        return;
    }

    if (event->button() == Qt::LeftButton) {
        toggleOnOff();
    }
}

} // namespace tl

#include "moc_qtledindicator.cpp"
