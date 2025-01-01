#include "qwidgetutils.h"

#include <QGuiApplication>
#include <QLayout>
#include <QScreen>

namespace tl {

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
            QGuiApplication::screens().constFirst()->geometry();
        const auto x = (screenGeometry.width() - widget->width()) / 2;
        const auto y = (screenGeometry.height() - widget->height()) / 2;
        widget->move(x, y);
    }
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

void clearLayout(QLayout* layout, bool deleteWidgets)
{
    if (!layout) {
        return;
    }

    while (auto item = layout->takeAt(0)) {
        if (auto child = item->layout()) {
            clearLayout(child, deleteWidgets);
            child->deleteLater();
        }

        if (auto widget = item->widget(); deleteWidgets && widget) {
            widget->deleteLater();
        }

        delete item;
    }
}

} // namespace tl
