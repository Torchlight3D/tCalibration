#include "guiutils.h"

#include <QLayout>
#include <QtGlobal>
#include <QWidget>

// This must be in global namespace
void qlementineResourceInitialization()
{
    // Loads the QRC content.
    Q_INIT_RESOURCE(qlementine);
    Q_INIT_RESOURCE(qlementine_font_roboto);
#if defined(WIN32)
    Q_INIT_RESOURCE(qlementine_font_inter_windows);
#else
    Q_INIT_RESOURCE(qlementine_font_inter);
#endif
}

namespace tl {

void initializeResources() { qlementineResourceInitialization(); }

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
