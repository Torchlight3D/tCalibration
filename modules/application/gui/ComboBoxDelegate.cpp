#include "ComboBoxDelegate.h"

#include <QPainter>

#include "primitiveutils.h"
#include "qcolorutils.h"
#include "qfontutils.h"
#include "qimageutils.h"
#include "stylestateutils.h"
#include "TheStyle.h"

namespace tl {

ComboBoxDelegate::ComboBoxDelegate(QWidget* widget, QlementineStyle& style)
    : QItemDelegate(widget), _widget(widget), _qlementineStyle(&style)
{
}

void ComboBoxDelegate::paint(QPainter* p, const QStyleOptionViewItem& opt,
                             const QModelIndex& idx) const
{
    const auto& theme = _qlementineStyle ? _qlementineStyle->theme() : Theme{};

    const auto isSeparator =
        idx.data(Qt::AccessibleDescriptionRole).toString() ==
        QLatin1String("separator");
    if (isSeparator) {
        const auto& rect = opt.rect;
        const auto& color = _qlementineStyle
                                ? _qlementineStyle->toolBarSeparatorColor()
                                : Theme().secondaryAlternativeColorDisabled;
        const auto lineW = theme.borderWidth;
        constexpr auto padding = 0; //_impl->theme.spacing / 2;
        const auto x = rect.x() + (rect.width() - lineW) / 2.;
        const auto y1 = static_cast<double>(rect.y() + padding);
        const auto y2 = static_cast<double>(rect.y() + rect.height() - padding);
        p->setBrush(Qt::NoBrush);
        p->setPen(QPen(color, lineW, Qt::SolidLine, Qt::FlatCap));
        p->drawLine(QPointF{x, y1}, QPointF{x, y2});
    }
    else {
        const auto mouse = getMenuItemMouseState(opt.state);

        // Background.
        const auto hPadding = theme.spacing;
        const auto& bgRect = opt.rect;
        const auto& bgColor =
            _qlementineStyle ? _qlementineStyle->menuItemBackgroundColor(mouse)
                             : Theme().primaryColorTransparent;
        constexpr auto radius = 0;
        p->setRenderHint(QPainter::Antialiasing, true);
        p->setPen(Qt::NoPen);
        p->setBrush(bgColor);
        p->drawRoundedRect(bgRect, radius, radius);

        // Foreground.
        const auto fgRect =
            bgRect.marginsRemoved(QMargins{hPadding, 0, hPadding, 0});
        if (fgRect.isEmpty()) {
            return;
        }

        const auto selected = getSelectionState(opt.state);
        const auto active = getActiveState(opt.state);
        const auto focus = selected == SelectionState::Selected
                               ? FocusState::Focused
                               : FocusState::NotFocused;
        auto availableW = fgRect.width();
        auto availableX = fgRect.x();
        const auto& fgData = idx.data(Qt::ForegroundRole);
        auto fgColor = _qlementineStyle
                           ? _qlementineStyle->menuItemForegroundColor(mouse)
                           : Theme().secondaryColor;
        if (fgData.isValid()) {
            fgColor = fgData.value<QColor>();
        }

        // Icon.
        const auto iconVariant = idx.data(Qt::DecorationRole);
        const auto& icon =
            iconVariant.isValid() && iconVariant.userType() == QMetaType::QIcon
                ? iconVariant.value<QIcon>()
                : QIcon{};
        if (availableW > 0 && !icon.isNull()) {
            const auto spacing = theme.spacing;
            const auto& iconSize = opt.decorationSize; // Get icon size.
            const auto pixmap = getPixmap(icon, iconSize, mouse, Qt::Unchecked);
            const auto* theStyle =
                qobject_cast<QlementineStyle*>(_widget->style());
            const auto colorize =
                theStyle && theStyle->isAutoIconColorEnabled(_widget);
            const auto pixmapPixelRatio = pixmap.devicePixelRatio();
            const auto pixmapW =
                pixmapPixelRatio != 0
                    ? static_cast<int>((qreal)pixmap.width() / pixmapPixelRatio)
                    : 0;
            const auto pixmapH = pixmapPixelRatio != 0
                                     ? static_cast<int>((qreal)pixmap.height() /
                                                        pixmapPixelRatio)
                                     : 0;
            const auto pixmapX =
                availableX + (iconSize.width() - pixmapW) /
                                 2; // Center the icon in the rect.
            const auto pixmapY = fgRect.y() + (fgRect.height() - pixmapH) / 2;
            const auto pixmapRect = QRect{pixmapX, pixmapY, pixmapW, pixmapH};
            availableW -= iconSize.width() + spacing;
            availableX += iconSize.width() + spacing;

            if (mouse == MouseState::Disabled && !colorize) {
                // Change only the icon's tint and opacity, so it looks
                // disabled.
                const auto& bgColor =
                    theStyle ? theStyle->listItemBackgroundColor(
                                   MouseState::Normal, selected, focus, active)
                             : Theme().neutralColorTransparent;
                const auto premultipiedColor =
                    getColorSourceOver(bgColor, fgColor);
                const auto& tintedPixmap =
                    qimg::getTintedPixmap(pixmap, premultipiedColor);
                const auto opacity =
                    selected == SelectionState::Selected ? 1. : 0.25;
                const auto backupOpacity = p->opacity();
                p->setOpacity(opacity * backupOpacity);
                p->drawPixmap(pixmapRect, tintedPixmap);
                p->setOpacity(backupOpacity);
            }
            else {
                // Actually color the whole icon if needed.
                const auto& colorizedPixmap =
                    colorize ? qimg::getColorizedPixmap(pixmap, fgColor)
                             : pixmap;
                p->drawPixmap(pixmapRect, colorizedPixmap);
            }
        }

        // Text.
        const auto textVariant = idx.data(Qt::DisplayRole);
        const auto& text = textVariant.isValid() &&
                                   textVariant.userType() == QMetaType::QString
                               ? textVariant.value<QString>()
                               : QString{};
        if (availableW > 0 && !text.isEmpty()) {
            const auto& fm = opt.fontMetrics;
            const auto elidedText =
                fm.elidedText(text, Qt::ElideRight, availableW);
            const auto textX = availableX;
            const auto textRect =
                QRect{textX, fgRect.y(), availableW, fgRect.height()};
            const auto textFlags = Qt::AlignVCenter | Qt::AlignBaseline |
                                   Qt::TextSingleLine | Qt::AlignLeft;
            p->setBrush(Qt::NoBrush);
            p->setPen(fgColor);
            p->drawText(textRect, textFlags, elidedText, nullptr);
        }
    }
}

QSize ComboBoxDelegate::sizeHint(const QStyleOptionViewItem& opt,
                                 const QModelIndex& idx) const
{
    const auto& theme = _qlementineStyle ? _qlementineStyle->theme() : Theme{};

    const auto isSeparator =
        idx.data(Qt::AccessibleDescriptionRole).toString() ==
        QLatin1String("separator");
    if (isSeparator) {
        const auto h = theme.spacing + theme.borderWidth;
        return QSize{h, h};
    }
    else {
        const auto hPadding = theme.spacing;
        const auto vPadding = theme.spacing / 2;
        const auto iconSize = theme.iconSize;
        const auto spacing = theme.spacing;
        const auto& fm = opt.fontMetrics;

        const auto textVariant = idx.data(Qt::DisplayRole);
        const auto& text = textVariant.isValid() &&
                                   textVariant.userType() == QMetaType::QString
                               ? textVariant.value<QString>()
                               : QString{};
        const auto iconVariant = idx.data(Qt::DecorationRole);
        const auto& icon =
            iconVariant.isValid() && iconVariant.userType() == QMetaType::QIcon
                ? iconVariant.value<QIcon>()
                : QIcon{};
        const auto textW = textWidth(fm, text);
        const auto iconW = !icon.isNull() ? iconSize.width() + spacing : 0;
        const auto w = std::max(0, hPadding + iconW + textW + hPadding);
        const auto h =
            std::max(theme.controlHeightMedium,
                     std::max(iconSize.height(), vPadding) + vPadding);
        return QSize{w, h};
    }
}

} // namespace tl
