#include "StatusBadge.h"

#include <QPainter>
#include <QPainterPath>

#include "primitiveutils.h"
#include "TheStyle.h"
#include "TheTheme.h"

namespace tl {

namespace {

std::pair<const QColor, const QColor> getStatusBadgeColors(
    StatusBadge::Type type, const Theme& theme)
{
    switch (type) {
        case StatusBadge::Type::Error:
            return std::make_pair(theme.statusColorError,
                                  theme.statusColorForeground);
        case StatusBadge::Type::Success:
            return std::make_pair(theme.statusColorSuccess,
                                  theme.statusColorForeground);
        case StatusBadge::Type::Warning:
            return std::make_pair(theme.statusColorWarning,
                                  theme.statusColorForeground);
        case StatusBadge::Type::Info:
        default:
            return std::make_pair(theme.statusColorInfo,
                                  theme.statusColorForeground);
    }
}

std::pair<QSize, QSize> getStatusBadgeSizes(StatusBadge::Size size,
                                            const Theme& theme)
{
    switch (size) {
        case StatusBadge::Size::Small:
            return std::make_pair(
                QSize{theme.controlHeightSmall, theme.controlHeightSmall},
                QSize{10, 10});
        case StatusBadge::Size::Medium:
        default:
            return std::make_pair(
                QSize{theme.controlHeightMedium, theme.controlHeightMedium},
                theme.iconSize);
    }
}

// Draws the icons by drawing QPainterPaths directly
void drawStatusBadgeIcon(QPainter* p, const QRect& rect, StatusBadge::Type type,
                         StatusBadge::Size size, const QColor& color,
                         qreal lineThickness)
{
    // TODO: Change a better name
    constexpr auto kSmallSize = 10.;
    constexpr auto kDefaultSize = 16.;

    switch (type) {
        case StatusBadge::Type::Success: {
            if (size == StatusBadge::Size::Small) {
                p->setBrush(Qt::NoBrush);
                p->setPen({color, lineThickness, Qt::SolidLine, Qt::RoundCap,
                           Qt::RoundJoin});
                const auto halfLineThickness = lineThickness * 0.5;
                const auto ellipseRect = QRectF(rect).marginsRemoved(
                    {halfLineThickness, halfLineThickness, halfLineThickness,
                     halfLineThickness});
                p->drawEllipse(ellipseRect);

                const auto w = rect.width();
                const auto h = rect.width();
                const auto x = rect.x();
                const auto y = rect.y();

                {
                    const QPointF p1{(3. / kSmallSize) * w + x,
                                     (5. / kSmallSize) * h + y};
                    const QPointF p2{(4.5 / kSmallSize) * w + x,
                                     (6.5 / kSmallSize) * h + y};
                    const QPointF p3{(7. / kSmallSize) * w + x,
                                     (4. / kSmallSize) * h + y};

                    QPainterPath path;
                    path.moveTo(p1);
                    path.lineTo(p2);
                    path.lineTo(p3);
                    p->drawPath(path);
                }
            }
            else {
                p->setBrush(Qt::NoBrush);
                p->setPen({color, lineThickness, Qt::SolidLine, Qt::RoundCap,
                           Qt::RoundJoin});
                const auto halfLineThickness = lineThickness * 0.5;
                const auto ellipseRect = QRectF(rect).marginsRemoved(
                    QMarginsF{halfLineThickness, halfLineThickness,
                              halfLineThickness, halfLineThickness});
                p->drawEllipse(ellipseRect);

                const auto w = rect.width();
                const auto h = rect.width();
                const auto x = rect.x();
                const auto y = rect.y();

                {
                    const QPointF p1{(5. / kDefaultSize) * w + x,
                                     (8.5 / kDefaultSize) * h + y};
                    const QPointF p2{(7. / kDefaultSize) * w + x,
                                     (10.5 / kDefaultSize) * h + y};
                    const QPointF p3{(11. / kDefaultSize) * w + x,
                                     (6.5 / kDefaultSize) * h + y};

                    QPainterPath path;
                    path.moveTo(p1);
                    path.lineTo(p2);
                    path.lineTo(p3);
                    p->drawPath(path);
                }
            }
        } break;
        case StatusBadge::Type::Info: {
            if (size == StatusBadge::Size::Small) {
                p->setBrush(Qt::NoBrush);
                p->setPen(QPen(color, lineThickness, Qt::SolidLine,
                               Qt::RoundCap, Qt::RoundJoin));
                const auto halfLineThickness = lineThickness * 0.5;
                const auto ellipseRect = QRectF(rect).marginsRemoved(
                    QMarginsF{halfLineThickness, halfLineThickness,
                              halfLineThickness, halfLineThickness});
                p->drawEllipse(ellipseRect);

                const auto w = rect.width();
                const auto h = rect.width();
                const auto x = rect.x();
                const auto y = rect.y();

                {
                    const QPointF p1{(5. / kSmallSize) * w + x,
                                     (5. / kSmallSize) * h + y};
                    const QPointF p2{(5. / kSmallSize) * w + x,
                                     (7. / kSmallSize) * h + y};
                    p->drawLine(p1, p2);
                }
                {
                    const QPointF ellipseCenter{(5. / kSmallSize) * w + x,
                                                (3. / kSmallSize) * h + y};
                    const auto ellipseRadius =
                        (1.1 * lineThickness / kSmallSize) * w;
                    p->setPen(Qt::NoPen);
                    p->setBrush(color);
                    p->drawEllipse(ellipseCenter, ellipseRadius, ellipseRadius);
                }
            }
            else {
                p->setBrush(Qt::NoBrush);
                p->setPen({color, lineThickness, Qt::SolidLine, Qt::RoundCap,
                           Qt::RoundJoin});
                const auto halfLineThickness = lineThickness * 0.5;
                const auto ellipseRect = QRectF(rect).marginsRemoved(
                    {halfLineThickness, halfLineThickness, halfLineThickness,
                     halfLineThickness});
                p->drawEllipse(ellipseRect);

                const auto w = rect.width();
                const auto h = rect.width();
                const auto x = rect.x();
                const auto y = rect.y();

                {
                    const QPointF p1{(6.75 / kDefaultSize) * w + x,
                                     (7. / kDefaultSize) * h + y};
                    const QPointF p2{(8. / kDefaultSize) * w + x,
                                     (7. / kDefaultSize) * h + y};
                    const QPointF p3{(8. / kDefaultSize) * w + x,
                                     (12. / kDefaultSize) * h + y};

                    QPainterPath path;
                    path.moveTo(p1);
                    path.lineTo(p2);
                    path.lineTo(p3);
                    p->drawPath(path);
                }
                {
                    const QPointF p1{(6.75 / kDefaultSize) * w + x,
                                     (12. / kDefaultSize) * h + y};
                    const QPointF p2{(9.25 / kDefaultSize) * w + x,
                                     (12. / kDefaultSize) * h + y};
                    p->drawLine(p1, p2);
                }
                {
                    const QPointF ellipseCenter = {(8. / kDefaultSize) * w + x,
                                                   (4. / kDefaultSize) * h + y};
                    const auto ellipseRadius =
                        (1.1 * lineThickness / kDefaultSize) * w;
                    p->setPen(Qt::NoPen);
                    p->setBrush(color);
                    p->drawEllipse(ellipseCenter, ellipseRadius, ellipseRadius);
                }
            }
        } break;
        case StatusBadge::Type::Warning: {
            if (size == StatusBadge::Size::Small) {
                const auto w = rect.width();
                const auto h = rect.width();
                const auto x = rect.x();
                const auto y = rect.y();

                p->setBrush(Qt::NoBrush);
                p->setPen(QPen(color, lineThickness, Qt::SolidLine,
                               Qt::RoundCap, Qt::RoundJoin));
                const auto triangleMargin = (1. / kSmallSize) * w;
                const auto triangleDeltaY = (1.5 / kSmallSize) * h;
                const auto triangleRadius = (1. / kSmallSize) * h;
                const auto triangleRect =
                    QRectF(rect)
                        .marginsAdded({triangleMargin, triangleMargin,
                                       triangleMargin, triangleMargin})
                        .translated(QPointF(0., triangleDeltaY));
                drawRoundedTriangle(p, triangleRect, triangleRadius);

                {
                    const QPointF p1{(5. / kSmallSize) * w + x,
                                     (2.5 / kSmallSize) * h + y};
                    const QPointF p2{(5. / kSmallSize) * w + x,
                                     (6.5 / kSmallSize) * h + y};
                    p->drawLine(p1, p2);
                }
                {
                    const QPointF ellipseCenter{(5. / kSmallSize) * w + x,
                                                (9. / kSmallSize) * h + y};
                    const auto ellipseRadius =
                        (1.1 * lineThickness / kSmallSize) * w;
                    p->setPen(Qt::NoPen);
                    p->setBrush(color);
                    p->drawEllipse(ellipseCenter, ellipseRadius, ellipseRadius);
                }
            }
            else {
                const auto w = rect.width();
                const auto h = rect.width();
                const auto x = rect.x();
                const auto y = rect.y();

                p->setBrush(Qt::NoBrush);
                p->setPen(QPen(color, lineThickness, Qt::SolidLine,
                               Qt::RoundCap, Qt::RoundJoin));
                const auto triangleMargin = (1. / kDefaultSize) * w;
                const auto triangleDeltaY = (2.5 / kDefaultSize) * h;
                const auto triangleRadius = (2. / kDefaultSize) * h;
                const auto triangleRect =
                    QRectF(rect)
                        .marginsAdded({triangleMargin, triangleMargin,
                                       triangleMargin, triangleMargin})
                        .translated(QPointF(0., triangleDeltaY));
                drawRoundedTriangle(p, triangleRect, triangleRadius);

                {
                    const QPointF p1{(8. / kDefaultSize) * w + x,
                                     (5.5 / kDefaultSize) * h + y};
                    const QPointF p2{(8. / kDefaultSize) * w + x,
                                     (9.5 / kDefaultSize) * h + y};
                    p->drawLine(p1, p2);
                }
                {
                    const QPointF ellipseCenter{(8. / kDefaultSize) * w + x,
                                                (12. / kDefaultSize) * h + y};
                    const auto ellipseRadius =
                        (1.1 * lineThickness / kDefaultSize) * w;
                    p->setPen(Qt::NoPen);
                    p->setBrush(color);
                    p->drawEllipse(ellipseCenter, ellipseRadius, ellipseRadius);
                }
            }
        } break;
        case StatusBadge::Type::Error: {
            if (size == StatusBadge::Size::Small) {
                const auto w = rect.width();
                const auto h = rect.width();
                const auto x = rect.x();
                const auto y = rect.y();

                p->setBrush(Qt::NoBrush);
                p->setPen(QPen(color, lineThickness, Qt::SolidLine,
                               Qt::RoundCap, Qt::RoundJoin));

                {
                    const QPointF p1{(3. / kSmallSize) * w + x,
                                     (0.5 / kSmallSize) * h + y};
                    const QPointF p2{(7. / kSmallSize) * w + x,
                                     (0.5 / kSmallSize) * h + y};
                    const QPointF p3{(9.5 / kSmallSize) * w + x,
                                     (3. / kSmallSize) * h + y};
                    const QPointF p4{(9.5 / kSmallSize) * w + x,
                                     (7. / kSmallSize) * h + y};
                    const QPointF p5{(7. / kSmallSize) * w + x,
                                     (9.5 / kSmallSize) * h + y};
                    const QPointF p6{(3. / kSmallSize) * w + x,
                                     (9.5 / kSmallSize) * h + y};
                    const QPointF p7{(0.5 / kSmallSize) * w + x,
                                     (7. / kSmallSize) * h + y};
                    const QPointF p8{(0.5 / kSmallSize) * w + x,
                                     (3. / kSmallSize) * h + y};

                    QPainterPath path;
                    path.moveTo(p1);
                    path.lineTo(p2);
                    path.lineTo(p3);
                    path.lineTo(p4);
                    path.lineTo(p5);
                    path.lineTo(p6);
                    path.lineTo(p7);
                    path.lineTo(p8);
                    path.closeSubpath();
                    p->drawPath(path);
                }
                {
                    const QPointF p1{(3.5 / kSmallSize) * w + x,
                                     (3.5 / kSmallSize) * h + y};
                    const QPointF p2{(6.5 / kSmallSize) * w + x,
                                     (6.5 / kSmallSize) * h + y};
                    p->drawLine(p1, p2);
                }
                {
                    const QPointF p1{(3.5 / kSmallSize) * w + x,
                                     (6.5 / kSmallSize) * h + y};
                    const QPointF p2{(6.5 / kSmallSize) * w + x,
                                     (3.5 / kSmallSize) * h + y};
                    p->drawLine(p1, p2);
                }
            }
            else {
                const auto w = rect.width();
                const auto h = rect.width();
                const auto x = rect.x();
                const auto y = rect.y();

                p->setBrush(Qt::NoBrush);
                p->setPen({color, lineThickness, Qt::SolidLine, Qt::RoundCap,
                           Qt::RoundJoin});

                {
                    const QPointF p1{(4.5 / kDefaultSize) * w + x,
                                     (0.5 / kDefaultSize) * h + y};
                    const QPointF p2{(11.5 / kDefaultSize) * w + x,
                                     (0.5 / kDefaultSize) * h + y};
                    const QPointF p3{(15.5 / kDefaultSize) * w + x,
                                     (4.5 / kDefaultSize) * h + y};
                    const QPointF p4{(15.5 / kDefaultSize) * w + x,
                                     (11.5 / kDefaultSize) * h + y};
                    const QPointF p5{(11.5 / kDefaultSize) * w + x,
                                     (15.5 / kDefaultSize) * h + y};
                    const QPointF p6{(4.5 / kDefaultSize) * w + x,
                                     (15.5 / kDefaultSize) * h + y};
                    const QPointF p7{(0.5 / kDefaultSize) * w + x,
                                     (11.5 / kDefaultSize) * h + y};
                    const QPointF p8{(0.5 / kDefaultSize) * w + x,
                                     (4.5 / kDefaultSize) * h + y};

                    QPainterPath path;
                    path.moveTo(p1);
                    path.lineTo(p2);
                    path.lineTo(p3);
                    path.lineTo(p4);
                    path.lineTo(p5);
                    path.lineTo(p6);
                    path.lineTo(p7);
                    path.lineTo(p8);
                    path.closeSubpath();
                    p->drawPath(path);
                }
                {
                    const QPointF p1{(5.5 / kDefaultSize) * w + x,
                                     (5.5 / kDefaultSize) * h + y};
                    const QPointF p2{(10.5 / kDefaultSize) * w + x,
                                     (10.5 / kDefaultSize) * h + y};
                    p->drawLine(p1, p2);
                }
                {
                    const QPointF p1{(10.5 / kDefaultSize) * w + x,
                                     (5.5 / kDefaultSize) * h + y};
                    const QPointF p2{(5.5 / kDefaultSize) * w + x,
                                     (10.5 / kDefaultSize) * h + y};
                    p->drawLine(p1, p2);
                }
            }
        } break;
        default:
            break;
    }
}

void drawStatusBadge(QPainter* p, const QRect& rect, StatusBadge::Type type,
                     StatusBadge::Size size, const Theme& theme)
{
    const auto [bgColor, fgColor] = getStatusBadgeColors(type, theme);
    const auto [badgeSize, iconSize] = getStatusBadgeSizes(size, theme);

    const QRect badgeRect{
        QPoint{rect.x() + (rect.width() - badgeSize.width()) / 2,
               rect.y() + (rect.height() - badgeSize.height()) / 2},
        badgeSize,
    };
    const QRect iconRect{
        QPoint{rect.x() + (rect.width() - iconSize.width()) / 2,
               rect.y() + (rect.height() - iconSize.height()) / 2},
        iconSize,
    };
    const auto radius = badgeRect.height() / 4.;

    // Background
    p->setRenderHint(QPainter::Antialiasing, true);
    p->setPen(Qt::NoPen);
    p->setBrush(bgColor);
    p->drawRoundedRect(badgeRect, radius, radius);

    // Foreground
    constexpr auto kLineThickness = 1.0001;
    drawStatusBadgeIcon(p, iconRect, type, size, fgColor, kLineThickness);
}

} // namespace

StatusBadge::StatusBadge(Type type, Size size, QWidget* parent)
    : QWidget(parent), m_size(size), m_type(type)
{
    setFocusPolicy(Qt::NoFocus);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

StatusBadge::Type StatusBadge::badgeType() const { return m_type; }

void StatusBadge::setBadgeType(Type type)
{
    if (type == m_type) {
        return;
    }

    m_type = type;
    update();
    emit badgeTypeChanged();
}

StatusBadge::Size StatusBadge::badgeSize() const { return m_size; }

void StatusBadge::setBadgeSize(Size size)
{
    if (size == m_size) {
        return;
    }

    m_size = size;
    updateGeometry();
    update();
    emit badgeSizeChanged();
}

QSize StatusBadge::sizeHint() const
{
    const auto* style = this->style();
    const auto* theStyle = qobject_cast<const QlementineStyle*>(style);

    auto extent = 0;
    switch (m_size) {
        case Size::Small:
            extent = theStyle ? theStyle->theme().iconSize.height()
                              : style->pixelMetric(QStyle::PM_ButtonIconSize);
            break;
        case Size::Medium:
            extent = theStyle ? theStyle->theme().iconSizeMedium.height()
                              : style->pixelMetric(QStyle::PM_LargeIconSize);
            break;
        default:
            extent = theStyle ? theStyle->theme().iconSizeMedium.height()
                              : style->pixelMetric(QStyle::PM_LargeIconSize);
            break;
    }

    return {extent, extent};
}

void StatusBadge::paintEvent(QPaintEvent* /*e*/)
{
    QPainter p(this);
    const auto* style = this->style();
    const auto* theStyle = qobject_cast<const QlementineStyle*>(style);
    const auto& theme = theStyle ? theStyle->theme() : Theme{};
    const auto opacity = isEnabled() ? 1.0 : 0.35;
    p.setOpacity(opacity);
    drawStatusBadge(&p, rect(), m_type, m_size, theme);
}

} // namespace tl

#include "moc_StatusBadge.cpp"