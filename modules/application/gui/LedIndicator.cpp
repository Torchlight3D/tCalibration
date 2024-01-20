#include "LedIndicator.h"

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

///------- LedIndicatorPrivate starts from here
class LedIndicatorPrivate
{
    Q_DEFINE_PIMPL(LedIndicator)

public:
    explicit LedIndicatorPrivate(LedIndicator *q);

    void init();

public:
    struct StateColors
    {
        const QColor &of(bool state) const
        {
            return state ? onColor : offColor;
        }

        QColor onColor = LedIndicator::defaultOnColor();
        QColor offColor = LedIndicator::defaultOffColor();
    };

    struct Blink
    {
        QTimer timer;
        bool state{State::On};
    };

    StateColors m_stateColors;
    Blink m_blink;
    LedIndicator::Shape m_shape{LedIndicator::RoundRect};
    bool m_state{State::Off};
    bool m_clickable{false};
};

LedIndicatorPrivate::LedIndicatorPrivate(LedIndicator *q) : q_ptr(q) {}

void LedIndicatorPrivate::init()
{
    Q_Q(LedIndicator);
    m_blink.timer.callOnTimeout([this, q]() {
        m_blink.state = !m_blink.state;
        q->repaint();
    });
    m_blink.timer.setInterval(500);
}

///------- LedIndicator starts from here
LedIndicator::LedIndicator(QWidget *parent)
    : QWidget(parent), d_ptr(new LedIndicatorPrivate(this))
{
    d_ptr->init();
}

LedIndicator::~LedIndicator() = default;

bool LedIndicator::state() const
{
    Q_D(const LedIndicator);
    return d->m_state;
}

void LedIndicator::setState(bool state)
{
    Q_D(LedIndicator);
    if (state == d->m_state) {
        return;
    }

    d->m_state = state;
    repaint();
}

LedIndicator::Shape LedIndicator::shape() const
{
    Q_D(const LedIndicator);
    return d->m_shape;
}

void LedIndicator::setShape(Shape shape)
{
    Q_D(LedIndicator);
    d->m_shape = shape;
    repaint();
}

const QColor &LedIndicator::onColor() const
{
    Q_D(const LedIndicator);
    return d->m_stateColors.onColor;
}

void LedIndicator::setOnColor(const QColor &color)
{
    Q_D(LedIndicator);
    d->m_stateColors.onColor = color;
    repaint();
}

QColor LedIndicator::defaultOnColor() { return {0, 255, 0}; }

const QColor &LedIndicator::offColor() const
{
    Q_D(const LedIndicator);
    return d->m_stateColors.offColor;
}

void LedIndicator::setOffColor(const QColor &color)
{
    Q_D(LedIndicator);
    d->m_stateColors.offColor = color;
    repaint();
}

QColor LedIndicator::defaultOffColor() { return {0, 50, 0}; }

bool LedIndicator::blinking() const
{
    Q_D(const LedIndicator);
    return d->m_blink.timer.isActive();
}

void LedIndicator::setBlinking(bool on)
{
    Q_D(LedIndicator);
    if (on) {
        d->m_blink.timer.setInterval(d->m_blink.timer.interval());
        d->m_blink.timer.start();
        d->m_blink.state = on;
    }
    else {
        // Blinking shall not influence the normal output now.
        d->m_blink.timer.stop();
        d->m_blink.state = true;
    }
    repaint();
}

int LedIndicator::blinkCycle() const
{
    Q_D(const LedIndicator);
    return d->m_blink.timer.interval();
}

void LedIndicator::setBlinkCycle(int cycle)
{
    Q_D(LedIndicator);
    d->m_blink.timer.setInterval(cycle);
}

void LedIndicator::setStatus(Status status)
{
    switch (status) {
        case Status::Off:
            setState(false);
            break;
        case Status::On:
            setState(true);
            setBlinking(false);
            break;
        case Status::Blink:
            setState(true);
            setBlinking(true);
            break;
        default:
            break;
    }
}

bool LedIndicator::clickable() const
{
    Q_D(const LedIndicator);
    return d->m_clickable;
}

void LedIndicator::setClickable(bool on)
{
    Q_D(LedIndicator);
    d->m_clickable = on;
}

void LedIndicator::paintEvent(QPaintEvent * /*event*/)
{
    Q_D(LedIndicator);
    const auto &paintColor =
        d->m_stateColors.of(d->m_blink.state && d->m_state);

    QPainter painter{this};
    painter.setRenderHint(QPainter::Antialiasing);

    const QRect paintArea{0, 0, width() - 1, height() - 1};
    const QPoint center{paintArea.width() / 2, paintArea.height() / 2};

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

    painter.setPen(Qt::NoPen);
    painter.setBrush(grad);

    switch (d->m_shape) {
        case Elipse:
            painter.drawEllipse(center, center.x(), center.y());
            break;
        case Rect:
            painter.drawRect(paintArea);
            break;
        case RoundRect: {
            constexpr qreal kRadius{5.0};
            painter.drawRoundedRect(paintArea, kRadius, kRadius);
        } break;
        case Triangle: {
            const QRectF rect{paintArea};

            QPainterPath path;
            path.moveTo(rect.left() + (rect.width() / 2), rect.top());
            path.lineTo(rect.bottomLeft());
            path.lineTo(rect.bottomRight());
            path.lineTo(rect.left() + (rect.width() / 2), rect.top());

            painter.drawPath(path);
            painter.fillPath(path, QBrush{grad});
        } break;
    }
}

void LedIndicator::mousePressEvent(QMouseEvent *event)
{
    Q_D(LedIndicator);
    if (d->m_clickable && event->button() == Qt::LeftButton) {
        toggleState();
    }
}

} // namespace tl

#include "moc_LedIndicator.cpp"
