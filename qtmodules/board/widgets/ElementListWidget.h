#pragma once

#include <QPixmap>
#include <QTreeWidget>

namespace thoht {

class ElementListWidget : public QTreeWidget
{
    Q_OBJECT

public:
    explicit ElementListWidget(QWidget *parent = nullptr);

protected:
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dragMoveEvent(QDragMoveEvent *event) override;
    void startDrag(Qt::DropActions supportedActions) override;

protected:
    QPixmap mDraggedPixmap;
};

} // namespace thoht
