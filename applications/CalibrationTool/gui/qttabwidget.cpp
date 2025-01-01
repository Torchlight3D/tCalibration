#include "qttabwidget.h"

namespace tl {

QtTabWidget::QtTabWidget(QWidget *parent) : QTabWidget(parent)
{
    connect(this, &QTabWidget::currentChanged, this, [this](int index) {
        for (int i{0}; i < count(); ++i) {
            if (i != index) {
                widget(i)->setSizePolicy(QSizePolicy::Ignored,
                                         QSizePolicy::Ignored);
            }
        }

        widget(index)->setSizePolicy(QSizePolicy::Preferred,
                                     QSizePolicy::Preferred);
        widget(index)->resize(widget(index)->minimumSizeHint());
        widget(index)->adjustSize();
        adjustSize();
    });
}

} // namespace tl

#include "moc_qttabwidget.cpp"
