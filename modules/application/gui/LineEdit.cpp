#include "LineEdit.h"

#include <QEvent>
#include <QFontDatabase>
#include <QPainter>
#include <QStyle>

#include "primitiveutils.h"
#include "qimageutils.h"
#include "TheStyle.h"

namespace tl {

const QIcon& LineEdit::icon() const { return _icon; }

void LineEdit::setIcon(const QIcon& icon)
{
    _icon = icon;
    update();
    if (_icon.isNull()) {
        setTextMargins({});
    }
    else {
        const auto spacing =
            style()->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);
        const auto iconExtent = style()->pixelMetric(QStyle::PM_SmallIconSize);
        setTextMargins(spacing + iconExtent, 0, 0, 0);
    }
}

void LineEdit::setUseMonoSpaceFont(bool useMonoSpaceFont)
{
    _useMonospaceFont = useMonoSpaceFont;
    ensurePolished();
    updateFont();
}

bool LineEdit::useMonoSpaceFont() const { return _useMonospaceFont; }

void LineEdit::paintEvent(QPaintEvent* e)
{
    QLineEdit::paintEvent(e);

    // Draw icon.
    const auto iconExtent = style()->pixelMetric(QStyle::PM_SmallIconSize);
    const auto spacing =
        style()->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);
    const auto iconSize = QSize{iconExtent, iconExtent};
    const auto pixmap = this->getPixmap();
    const auto pixmapX = spacing;
    const auto pixmapY = (height() - iconSize.height()) / 2;
    const auto pixmapRect = QRect{{pixmapX, pixmapY}, iconSize};

    QPainter p(this);
    p.drawPixmap(pixmapRect, pixmap);
}

bool LineEdit::event(QEvent* e)
{
    if (e->type() == QEvent::Type::PaletteChange) {
        update();
    }
    return QLineEdit::event(e);
}

QPixmap LineEdit::getPixmap() const
{
    const auto iconExtent = style()->pixelMetric(QStyle::PM_SmallIconSize);
    const auto iconSize = QSize{iconExtent, iconExtent};

    const auto* theStyle = qobject_cast<QlementineStyle*>(style());
    const auto autoColorize =
        theStyle && theStyle->isAutoIconColorEnabled(this);
    if (autoColorize) {
        const auto pixmap = ::tl::getPixmap(
            _icon, iconSize, MouseState::Normal, Qt::Unchecked);
        const auto colorGroup = isEnabled() ? QPalette::ColorGroup::Normal
                                            : QPalette::ColorGroup::Disabled;
        const auto& color =
            palette().color(colorGroup, QPalette::ColorRole::Text);
        const auto colorizedPixmap = qimg::colorizePixmap(pixmap, color);
        return colorizedPixmap;
    }
    else {
        const auto mouse =
            isEnabled() ? MouseState::Normal : MouseState::Disabled;
        const auto pixmap =
            ::tl::getPixmap(_icon, iconSize, mouse, Qt::Unchecked);
        return pixmap;
    }
}

void LineEdit::updateFont()
{
    const auto* style = this->style();
    const auto* theStyle = qobject_cast<const QlementineStyle*>(style);

    if (_useMonospaceFont) {
        if (theStyle) {
            setFont(theStyle->theme().fontMonospace);
        }
        else {
            const auto systemFont =
                QFontDatabase::systemFont(QFontDatabase::FixedFont);
            setFont(systemFont);
        }
    }
    else {
        if (theStyle) {
            setFont(theStyle->theme().fontRegular);
        }
        else {
            const auto systemFont =
                QFontDatabase::systemFont(QFontDatabase::GeneralFont);
            setFont(systemFont);
        }
    }
}

Status LineEdit::status() const { return _status; }

void LineEdit::setStatus(Status status)
{
    if (status != _status) {
        _status = status;
        update();
        emit statusChanged();
    }
}

} // namespace tl

#include "moc_LineEdit.cpp"
