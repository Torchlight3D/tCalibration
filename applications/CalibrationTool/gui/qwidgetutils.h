#pragma once

#include <QWidget>

namespace tl {

void centerWidget(QWidget* widget, QWidget* host = nullptr);

qreal getDpi(const QWidget* widget);

QWindow* getWindow(const QWidget* widget);

void clearLayout(QLayout* layout, bool deleteWidgets = true);

template <class T>
T* findFirstParentOfType(QWidget* child)
{
    auto* parent = child;

    while (parent) {
        parent = parent->parentWidget();
        if (auto* typedPArent = qobject_cast<T*>(parent)) {
            return typedPArent;
        }
    }

    return nullptr;
}

} // namespace tl
