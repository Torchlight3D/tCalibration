#include "qwidgetutils.h"

#include <QFrame>
#include <QGuiApplication>
#include <QLayout>
#include <QScreen>

#include "TheStyle.h"

namespace tl {

QWidget* makeHorizontalLine(QWidget* parent, int maxWidth)
{
    const auto* theStyle =
        parent ? qobject_cast<QlementineStyle*>(parent->style()) : nullptr;
    const auto lineThickness = theStyle ? theStyle->theme().borderWidth : 1;

    auto* line = new QFrame(parent);
    line->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    line->setFixedHeight(lineThickness);
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Plain);

    if (maxWidth >= 0) {
        line->setMaximumWidth(maxWidth);
    }

    return line;
}

QWidget* makeVerticalLine(QWidget* parent, int maxHeight)
{
    const auto* theStyle =
        parent ? qobject_cast<QlementineStyle*>(parent->style()) : nullptr;
    const auto lineThickness = theStyle ? theStyle->theme().borderWidth : 1;

    auto* line = new QFrame(parent);
    line->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::MinimumExpanding);
    line->setFixedWidth(lineThickness);
    line->setFrameShape(QFrame::VLine);
    line->setFrameShadow(QFrame::Plain);

    if (maxHeight >= 0) {
        line->setMaximumHeight(maxHeight);
    }

    return line;
}

void centerWidget(QWidget* widget, QWidget* host)
{
    if (!host)
        host = widget->parentWidget();

    if (host) {
        const auto& hostRect = host->geometry();
        widget->move(hostRect.center() - widget->rect().center());
    }
    else {
        const auto screenGeometry =
            QGuiApplication::screens().front()->geometry();
        const auto x = (screenGeometry.width() - widget->width()) / 2;
        const auto y = (screenGeometry.height() - widget->height()) / 2;
        widget->move(x, y);
    }
}

QMargins getDefaultMargins(const QStyle* style)
{
    if (!style) {
        return {};
    }

    const auto left = style->pixelMetric(QStyle::PM_LayoutLeftMargin);
    const auto right = style->pixelMetric(QStyle::PM_LayoutRightMargin);
    const auto top = style->pixelMetric(QStyle::PM_LayoutTopMargin);
    const auto bottom = style->pixelMetric(QStyle::PM_LayoutBottomMargin);
    return {left, top, right, bottom};
}

qreal getDpi(const QWidget* widget)
{
    if (widget) {
        if (const auto* screen = widget->screen()) {
            return screen->logicalDotsPerInch();
        }
    }

    return 72.;
}

QWindow* getWindow(const QWidget* widget)
{
    if (widget) {
        if (auto* window = widget->window()) {
            return window->windowHandle();
        }
    }
    return nullptr;
}

void clearLayout(QLayout* layout)
{
    if (!layout)
        return;

    QLayoutItem* item{nullptr};
    while ((item = layout->takeAt(0))) {
        if (item->layout()) {
            clearLayout(item->layout());
            delete item->layout();
        }
        if (item->widget()) {
            delete item->widget();
        }
        delete item;
    }
}

} // namespace tl
