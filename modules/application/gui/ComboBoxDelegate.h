#pragma once

#include <QItemDelegate>
#include <QPointer>

namespace thoht {

class QlementineStyle;

class ComboBoxDelegate : public QItemDelegate
{
public:
    ComboBoxDelegate(QWidget* widget, QlementineStyle& style);

protected:
    void paint(QPainter* p, const QStyleOptionViewItem& opt,
               const QModelIndex& idx) const override;
    QSize sizeHint(const QStyleOptionViewItem& opt,
                   const QModelIndex& idx) const override;

private:
    const QWidget* _widget{nullptr};
    const QPointer<QlementineStyle> _qlementineStyle;
};

} // namespace thoht
