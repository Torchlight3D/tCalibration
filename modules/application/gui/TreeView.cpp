#include "TreeView.h"

#include <QHeaderView>
#include <QMenu>

namespace tl {

TreeView::TreeView(QWidget* parent) : QTreeView(parent)
{
    setAllColumnsShowFocus(true);
    setSortingEnabled(true);
    setUniformRowHeights(true);

    header()->setContextMenuPolicy(Qt::CustomContextMenu);

    connect(header(), &QHeaderView::customContextMenuRequested, this, [this] {
        if (!model()) {
            return;
        }

        QMenu menu;
        for (int i{0}; i < model()->columnCount(); ++i) {
            auto* action = menu.addAction(
                model()->headerData(i, Qt::Horizontal).toString());
            action->setCheckable(true);
            action->setChecked(!isColumnHidden(i));
        }

        if (auto* action = menu.exec(QCursor::pos())) {
            const auto column =
                static_cast<int>(menu.actions().indexOf(action));
            if (isColumnHidden(column)) {
                showColumn(column);
            }
            else {
                hideColumn(column);
            }
        }
    });
}

} // namespace tl

#include "moc_TreeView.cpp"
