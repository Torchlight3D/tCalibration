#pragma once

#include <QWidget>

#include "../BoardElement.h"

namespace thoht {

class ElementListWidget;

class ElementPickerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ElementPickerWidget(
        QWidget *parent = nullptr,
        BoardElement::ElementType type = BoardElement::etUnknown,
        BoardElement::TimeSeriesType tsType = BoardElement::tstUnknown,
        BoardElement::TimeSeriesSize tsSize = BoardElement::tssUnknown);
    ~ElementPickerWidget();

    QString selectedElement();

signals:
    void elementDoubleClicked();

private:
    ElementListWidget *listWidget;
};

} // namespace thoht
