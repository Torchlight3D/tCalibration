#include "guiutils.h"

#include <QLayout>
#include <QtGlobal>
#include <QWidget>

namespace tl {

void clearLayout(QLayout* layout, bool deleteWidgets)
{
    while (auto* item = layout->takeAt(0)) {
        if (deleteWidgets) {
            if (auto* widget = item->widget())
                widget->deleteLater();
        }

        if (auto* child = item->layout())
            clearLayout(child, deleteWidgets);
        delete item;
    }
}

} // namespace tl
