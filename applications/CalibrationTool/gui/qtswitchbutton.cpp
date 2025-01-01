#include "qtswitchbutton.h"

#include <QPainter>
#include <QPropertyAnimation>

QT_BEGIN_NAMESPACE
extern Q_DECL_IMPORT void qt_blurImage(QPainter *p, QImage &blurImage,
                                       qreal radius, bool quality,
                                       bool alphaOnly, int transposed = 0);
QT_END_NAMESPACE

namespace tl {

namespace {
/// Some default properties
constexpr int kDefaultHeight{36};
constexpr qreal kTrackCornerRadius{8.};
constexpr qreal kTrackDisabledOpacity{0.25};
constexpr qreal kHandleRadius{14.5};
constexpr qreal kHandleShadowElevation{2.};
constexpr qreal kDisabledTextOpacity{0.25};
constexpr QMargins kIndicatorMargin{8, 8, 8, 8};
const QColor kTrackDisabledColor{Qt::black};
const QColor kHandleDisabledColor{Qt::gray};

QPixmap drawShadowEllipse(qreal radius, qreal elevation, const QColor &color)
{
    QPixmap pm{qRound(radius * 2.), qRound(radius * 2.)};
    pm.fill(Qt::transparent);

    QPainter p{&pm};
    p.setBrush(color);
    p.setPen(Qt::NoPen);
    p.setRenderHint(QPainter::Antialiasing);

    p.drawEllipse(QRectF{QPointF{}, pm.size().toSizeF()}.center(),
                  radius - elevation, radius - elevation);
    p.end();

    QImage tmp{pm.size(), QImage::Format_ARGB32_Premultiplied};
    tmp.setDevicePixelRatio(pm.devicePixelRatioF());
    tmp.fill(0);
    QImage blurred{tmp};

    p.begin(&tmp);
    p.setCompositionMode(QPainter::CompositionMode_Source);
    p.drawPixmap(QPointF{}, pm);
    p.end();

    p.begin(&blurred);
    qt_blurImage(&p, tmp, elevation * 4, true, false);
    p.end();

    return QPixmap::fromImage(blurred);
}

} // namespace

namespace prop {
constexpr char kHandlePosition[]{"handlePosition"};
} // namespace prop

class QtSwitchButtonPrivate
{
    Q_DEFINE_PIMPL(QtSwitchButton)

public:
    explicit QtSwitchButtonPrivate(QtSwitchButton *q);

    void init();

    QRect indicatorRect() const;
    QRect textRect() const;

    void toggleImpl(Qt::CheckState state);

public:
    const QPixmap m_shadowPixmap;
    QColor m_trackOnColor{QtSwitchButton::kDefaultTrackOnColor};
    QColor m_trackOffColor{QtSwitchButton::kDefaultTrackOffColor};
    QColor m_handleOnColor{QtSwitchButton::kDefaultHandleOnColor};
    QColor m_handleOffColor{QtSwitchButton::kDefaultHandleOffColor};
    QPropertyAnimation *m_handlePosAnimation;
    int m_handlePos;
};

QtSwitchButtonPrivate::QtSwitchButtonPrivate(QtSwitchButton *q)
    : q_ptr(q),
      m_shadowPixmap(drawShadowEllipse(kHandleRadius, kHandleShadowElevation,
                                       QColor{0, 0, 0, 70}))
{
}

void QtSwitchButtonPrivate::init()
{
    Q_Q(QtSwitchButton);
    q->setCursor(Qt::PointingHandCursor);
    q->setCheckable(true);

    m_handlePos = q->isChecked()
                      ? (kIndicatorMargin.left() + kIndicatorMargin.right() + 2)
                      : 0;

    m_handlePosAnimation = new QPropertyAnimation(q, prop::kHandlePosition, q);
    m_handlePosAnimation->setDuration(150);
    m_handlePosAnimation->setEasingCurve(QEasingCurve::OutQuad);
}

void QtSwitchButtonPrivate::toggleImpl(Qt::CheckState state)
{
    m_handlePosAnimation->stop();
    m_handlePosAnimation->setEndValue(
        state == Qt::Unchecked
            ? 0
            : (kIndicatorMargin.left() + kIndicatorMargin.right() + 2) * 2);
    m_handlePosAnimation->start();
}

QRect QtSwitchButtonPrivate::indicatorRect() const
{
    Q_Q(const QtSwitchButton);
    constexpr auto w =
        kIndicatorMargin.left() + kDefaultHeight + kIndicatorMargin.right();

    return q->isLeftToRight() ? QRect{0, 0, w, kDefaultHeight}
                              : QRect{q->width() - w, 0, w, kDefaultHeight};
}

QRect QtSwitchButtonPrivate::textRect() const
{
    Q_Q(const QtSwitchButton);
    constexpr auto w =
        kIndicatorMargin.left() + kDefaultHeight + kIndicatorMargin.right();

    return q->isLeftToRight() ? q->rect().marginsRemoved(QMargins{w, 0, 0, 0})
                              : q->rect().marginsRemoved(QMargins{0, 0, w, 0});
}

QtSwitchButton::QtSwitchButton(QWidget *parent)
    : QAbstractButton(parent), d_ptr(new QtSwitchButtonPrivate(this))
{
    d_ptr->init();
}

QtSwitchButton::QtSwitchButton(const QString &text, QWidget *parent)
    : QtSwitchButton(parent)
{
    setText(text);
}

QtSwitchButton::~QtSwitchButton() = default;

QSize QtSwitchButton::sizeHint() const
{
    Q_D(const QtSwitchButton);
    return {kIndicatorMargin.left() + kDefaultHeight +
                kIndicatorMargin.right() +
                fontMetrics().horizontalAdvance(text()),
            kDefaultHeight};
}

void QtSwitchButton::setTrackColor(const QColor &onColor,
                                   const QColor &offColor)
{
    Q_D(QtSwitchButton);
    d->m_trackOnColor = onColor;
    d->m_trackOffColor = offColor;
}

void QtSwitchButton::setHandleColor(const QColor &onColor,
                                    const QColor &offColor)
{
    Q_D(QtSwitchButton);
    d->m_handleOnColor = onColor;
    d->m_handleOffColor = offColor;
}

void QtSwitchButton::paintEvent(QPaintEvent * /*event*/)
{
    Q_D(QtSwitchButton);
    QPainter p{this};
    p.setPen(Qt::NoPen);
    p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

    auto trackMargin{kIndicatorMargin};
    trackMargin.setTop(trackMargin.top() + 2);
    trackMargin.setBottom(trackMargin.bottom() + 2);

    auto trackRect = d->indicatorRect().marginsRemoved(trackMargin).toRectF();

    if (isEnabled()) {
        // Track
        p.setBrush(isChecked() ? d->m_trackOnColor : d->m_trackOffColor);
        p.drawRoundedRect(trackRect, kTrackCornerRadius, kTrackCornerRadius);

        // Handle
        trackRect.setX(trackRect.x() - trackMargin.left() -
                       trackMargin.right() - 2 +
                       d->m_handlePosAnimation->currentValue().toInt());
        if (!d->m_shadowPixmap.isNull()) {
            p.drawPixmap(
                trackRect.center() - QPointF{kHandleRadius, kHandleRadius - 1.},
                d->m_shadowPixmap);
        }
        p.setBrush(isChecked() ? d->m_handleOnColor : d->m_handleOffColor);
        p.drawEllipse(trackRect.center(),
                      kHandleRadius - kHandleShadowElevation - 1.,
                      kHandleRadius - kHandleShadowElevation - 1.);

        // Text
        if (!text().isEmpty()) {
            p.setPen(palette().color(QPalette::Active, QPalette::ButtonText));
            p.drawText(d->textRect(), Qt::AlignLeft | Qt::AlignVCenter, text());
        }
    }
    else {
        // Track
        p.setBrush(kTrackDisabledColor);
        p.setOpacity(kTrackDisabledOpacity);
        p.drawRoundedRect(trackRect, kTrackCornerRadius, kTrackCornerRadius);

        // Handle
        p.setOpacity(1.);
        trackRect.setX(
            isChecked()
                ? trackRect.x() + trackMargin.left() + trackMargin.right() + 2
                : trackRect.x() - trackMargin.left() - trackMargin.right() - 2);

        if (!d->m_shadowPixmap.isNull()) {
            p.drawPixmap(
                trackRect.center() - QPointF{kHandleRadius, kHandleRadius - 1.},
                d->m_shadowPixmap);
        }
        p.setBrush(kHandleDisabledColor);
        p.drawEllipse(trackRect.center(),
                      kHandleRadius - kHandleShadowElevation - 1.,
                      kHandleRadius - kHandleShadowElevation - 1.);

        // Text
        if (!text().isEmpty()) {
            p.setOpacity(kDisabledTextOpacity);
            p.setPen(palette().color(QPalette::Disabled, QPalette::ButtonText));
            p.drawText(d->textRect(), Qt::AlignLeft | Qt::AlignVCenter, text());
        }
    }
}

void QtSwitchButton::nextCheckState()
{
    QAbstractButton::nextCheckState();
    checkStateSet();
}

void QtSwitchButton::checkStateSet()
{
    Q_D(QtSwitchButton);
    d->toggleImpl(Qt::CheckState{isChecked() ? Qt::Checked : Qt::Unchecked});
}

int QtSwitchButton::handlePosition() const
{
    Q_D(const QtSwitchButton);
    return d->m_handlePos;
}

void QtSwitchButton::setHandlePosition(int pos)
{
    Q_D(QtSwitchButton);
    d->m_handlePos = pos;
    update();
    emit handlePositionChanged();
}

} // namespace tl

#include "moc_qtswitchbutton.cpp"
