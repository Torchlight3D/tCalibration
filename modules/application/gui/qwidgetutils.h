#pragma once

#include <QWidget>

namespace thoht {

QWidget* makeVerticalLine(QWidget* parentWidget, int maxHeight = -1);
QWidget* makeHorizontalLine(QWidget* parentWidget, int maxWidth = -1);

void centerWidget(QWidget* widget, QWidget* host = nullptr);

QMargins getDefaultMargins(const QStyle* style);

qreal getDpi(const QWidget* widget);

QWindow* getWindow(const QWidget* widget);

void clearLayout(QLayout* layout);

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

} // namespace thoht
