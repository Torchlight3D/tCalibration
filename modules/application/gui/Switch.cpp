#include "Switch.h"

#include <QEvent>
#include <QPainter>

#include "primitiveutils.h"
#include "qfontutils.h"
#include "qimageutils.h"
#include "RoundedFocusFrame.h"
#include "stylestateutils.h"
#include "TheStyle.h"
#include "TheStyleOptions.h"

namespace thoht {

Switch::Switch(QWidget* parent) : QAbstractButton(parent)
{
    setCheckable(true);
    setChecked(false);
    setAutoRepeat(false);
    setupAnimation();
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    const auto* style = this->style();
    const auto* theStyle = qobject_cast<const QlementineStyle*>(style);
    m_handlePadding = theStyle ? theStyle->theme().borderWidth * 2 : 2;

    // Focus frame.
    m_focusFrame = new RoundedFocusFrame(this);
    m_focusFrame->setRadiuses(theStyle ? theStyle->theme().borderRadius : 0);
    m_focusFrame->setWidget(this);
}

void Switch::initStyleOptionFocus(QStyleOptionFocusRoundedRect& opt) const
{
    const auto* style = this->style();
    const auto deltaX =
        style->pixelMetric(QStyle::PM_FocusFrameHMargin, &opt, this);
    const auto deltaY =
        style->pixelMetric(QStyle::PM_FocusFrameVMargin, &opt, this);
    const auto rect = switchRect();
    opt.rect =
        rect.marginsAdded({deltaX / 2, deltaY / 2, deltaX / 2, deltaY / 2})
            .translated(deltaX, deltaY);
    opt.radiuses = rect.height() / 2.;
}

QSize Switch::sizeHint() const
{
    const auto* style = this->style();
    const auto* theStyle = qobject_cast<const QlementineStyle*>(style);

    const auto fm = fontMetrics();
    const auto textW = textWidth(fm, text());
    const auto textH = fm.height();
    const auto spacing = style->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);
    const auto rect = switchRect();
    const auto w = (textW > 0 ? textW + spacing : 0) + rect.width();
    const auto defaultH = theStyle ? theStyle->theme().controlHeightMedium : 0;
    const auto h = std::max({textH, rect.height(), defaultH});
    return {w, h};
}

void Switch::paintEvent(QPaintEvent* /*e*/)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    const auto* style = this->style();
    const auto spacing = style->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);
    const auto contentRect = rect();
    const auto iconSize = this->iconSize();
    const auto hasIcon = !icon().isNull() || iconSize.isEmpty();
    const auto hasText = !text().isEmpty();

    // Draw switch button.
    const auto rect = switchRect();
    const auto switchRadius = rect.height() / 2.;
    const auto& bgColor = m_animBg.currentValue().value<QColor>();
    const auto& fgColor = m_animFg.currentValue().value<QColor>();
    const auto& borderColor = m_animBorder.currentValue().value<QColor>();
    const auto textColor = this->textColor();
    p.setPen(Qt::NoPen);
    p.setBrush(bgColor);
    p.drawRoundedRect(rect, switchRadius, switchRadius);
    drawRoundedRectBorder(&p, rect, borderColor, 1., switchRadius);

    // Draw handle.
    const auto handleXRatio = m_animHandle.currentValue().toDouble();
    const auto handleDiameter =
        static_cast<double>(rect.height() - m_handlePadding * 2);
    const auto handleGrooveWidth =
        rect.width() - m_handlePadding * 2 - handleDiameter;
    const auto handleX =
        rect.x() + m_handlePadding + handleGrooveWidth * handleXRatio;
    const auto handleY = static_cast<double>(rect.y() + m_handlePadding);
    const QRectF handleRect{handleX, handleY, handleDiameter, handleDiameter};
    p.setPen(Qt::NoPen);
    p.setBrush(fgColor);
    p.drawEllipse(handleRect);
    auto availableX = rect.x() + rect.width() + spacing;
    auto availableW = contentRect.width() - rect.width() - spacing;

    // Draw icon.
    const auto extent = iconSize.height();
    if (hasIcon && availableW >= extent) {
        const auto pixmap =
            icon().pixmap(extent, QIcon::Mode::Normal, QIcon::State::On);
        const auto coloredPixmap = qimg::colorizePixmap(pixmap, textColor);
        const auto iconX = availableX;
        const auto iconY =
            contentRect.y() + (contentRect.height() - extent) / 2;
        const QRect iconRect{iconX, iconY, extent, extent};
        p.drawPixmap(iconRect, coloredPixmap);

        availableX += extent + spacing;
        availableW -= extent + spacing;
    }

    // Draw text.
    if (hasText && availableW > 0) {
        const auto textX = availableX;
        const auto textY = contentRect.y();
        const auto textH = contentRect.height();
        const auto textW = availableW;
        const QRect textRect{textX, textY, textW, textH};
        const auto fm = fontMetrics();
        const auto elidedText = fm.elidedText(
            text(), Qt::ElideRight, textRect.width(), Qt::TextSingleLine);
        p.setBrush(Qt::NoBrush);
        p.setPen(textColor);
        p.drawText(textRect, Qt::AlignVCenter | Qt::AlignLeft, elidedText);
    }
}

void Switch::enterEvent(QEnterEvent* e)
{
    QAbstractButton::enterEvent(e);
    m_isMouseOver = true;
    startAnimation();
}

void Switch::changeEvent(QEvent* e)
{
    QAbstractButton::changeEvent(e);
    if (e->type() == QEvent::EnabledChange) {
        startAnimation();
    }
}

void Switch::focusInEvent(QFocusEvent* e)
{
    QAbstractButton::focusInEvent(e);
    startAnimation();
}

void Switch::focusOutEvent(QFocusEvent* e)
{
    QAbstractButton::focusOutEvent(e);
    startAnimation();
}

void Switch::checkStateSet()
{
    QAbstractButton::checkStateSet();
    startAnimation();
}

void Switch::startAnimation()
{
    const auto animationDuration =
        isVisible() ? style()->styleHint(QStyle::SH_Widget_Animation_Duration)
                    : 0;

    const auto currentBg = m_animBg.currentValue();
    m_animBg.stop();
    m_animBg.setDuration(animationDuration);
    m_animBg.setStartValue(currentBg);
    m_animBg.setEndValue(backgroundColor());
    m_animBg.start();

    const auto currentBorder = m_animBorder.currentValue();
    m_animBorder.stop();
    m_animBorder.setDuration(animationDuration);
    m_animBorder.setStartValue(currentBorder);
    m_animBorder.setEndValue(borderColor());
    m_animBorder.start();

    const auto currentFg = m_animFg.currentValue();
    m_animFg.stop();
    m_animFg.setDuration(animationDuration);
    m_animFg.setStartValue(currentFg);
    m_animFg.setEndValue(foregroundColor());
    m_animFg.start();

    const auto currentXRatio = m_animHandle.currentValue();
    m_animHandle.stop();
    m_animHandle.setDuration(animationDuration);
    m_animHandle.setStartValue(currentXRatio);
    m_animHandle.setEndValue(isChecked() ? 1. : 0.);
    m_animHandle.start();
}

void Switch::setupAnimation()
{
    constexpr auto animationDuration = 0; // Don't animate before it is shown.

    const auto& startBgColor = backgroundColor();
    m_animBg.setDuration(animationDuration);
    m_animBg.setEasingCurve(QEasingCurve::OutCubic);
    m_animBg.setStartValue(startBgColor);
    m_animBg.setEndValue(startBgColor);
    connect(&m_animBg, &QVariantAnimation::valueChanged, this,
            qOverload<>(&QWidget::update));

    const auto& startBorderColor = borderColor();
    m_animBorder.setDuration(animationDuration);
    m_animBorder.setEasingCurve(QEasingCurve::OutCubic);
    m_animBorder.setStartValue(startBorderColor);
    m_animBorder.setEndValue(startBorderColor);
    connect(&m_animBorder, &QVariantAnimation::valueChanged, this,
            qOverload<>(&QWidget::update));

    const auto& startFgColor = foregroundColor();
    m_animFg.setDuration(animationDuration);
    m_animFg.setEasingCurve(QEasingCurve::OutCubic);
    m_animFg.setStartValue(startFgColor);
    m_animFg.setEndValue(startFgColor);
    connect(&m_animFg, &QVariantAnimation::valueChanged, this,
            qOverload<>(&QWidget::update));

    connect(this, &QAbstractButton::pressed, this, &Switch::startAnimation);
    connect(this, &QAbstractButton::released, this, &Switch::startAnimation);
    connect(this, &QAbstractButton::toggled, this, &Switch::startAnimation);

    m_animHandle.setDuration(animationDuration);
    m_animHandle.setEasingCurve(QEasingCurve::OutCubic);
    m_animHandle.setStartValue(0.);
    m_animHandle.setEndValue(0.);
    connect(&m_animHandle, &QVariantAnimation::valueChanged, this,
            qOverload<>(&QWidget::update));
}

const QColor& Switch::backgroundColor() const
{
    const auto* style = this->style();
    const auto* theStyle = qobject_cast<const QlementineStyle*>(style);
    const auto& bgColor =
        theStyle ? theStyle->switchGrooveColor(
                       getMouseState(isDown(), m_isMouseOver, isEnabled()),
                       getCheckState(isChecked()))
                 : style->standardPalette().color(
                       isEnabled() ? QPalette::Normal : QPalette::Disabled,
                       QPalette::Button);
    return bgColor;
}

const QColor& Switch::borderColor() const
{
    const auto* style = this->style();
    const auto* theStyle = qobject_cast<const QlementineStyle*>(style);
    const auto& borderColor =
        theStyle ? theStyle->switchGrooveBorderColor(
                       getMouseState(isDown(), m_isMouseOver, isEnabled()),
                       getFocusState(hasFocus()), getCheckState(isChecked()))
                 : style->standardPalette().color(
                       isEnabled() ? QPalette::Normal : QPalette::Disabled,
                       QPalette::ButtonText);
    return borderColor;
}

const QColor& Switch::foregroundColor() const
{
    const auto* style = this->style();
    const auto* theStyle = qobject_cast<const QlementineStyle*>(style);
    const auto& fgColor =
        theStyle ? theStyle->switchHandleColor(
                       getMouseState(isDown(), m_isMouseOver, isEnabled()),
                       getCheckState(isChecked()))
                 : style->standardPalette().color(
                       isEnabled() ? QPalette::Normal : QPalette::Disabled,
                       QPalette::ButtonText);
    return fgColor;
}

const QColor& Switch::textColor() const
{
    const auto* style = this->style();
    const auto* theStyle = qobject_cast<const QlementineStyle*>(style);
    const auto& textColor =
        theStyle
            ? theStyle->labelForegroundColor(
                  getMouseState(isDown(), m_isMouseOver, isEnabled()), this)
            : style->standardPalette().color(
                  isEnabled() ? QPalette::Normal : QPalette::Disabled,
                  QPalette::Text);
    return textColor;
}

QRect Switch::switchRect() const
{
    const auto* style = this->style();
    const auto* theStyle = qobject_cast<const QlementineStyle*>(style);
    const auto contentRect = rect();
    const auto switchW = theStyle ? theStyle->theme().controlHeightLarge : 28;
    const auto switchH = theStyle ? theStyle->theme().controlHeightSmall : 16;
    const auto switchX = contentRect.x();
    const auto switchY = contentRect.y() + (contentRect.height() - switchH) / 2;

    return {switchX, switchY, switchW, switchH};
}

void Switch::leaveEvent(QEvent* e)
{
    QAbstractButton::leaveEvent(e);
    m_isMouseOver = false;
    startAnimation();
}

} // namespace thoht

#include "moc_Switch.cpp"
