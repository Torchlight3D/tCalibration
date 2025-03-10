﻿#include "ElementPickerWidget.h"
#include "ElementListWidget.h"
#include "../factory/element_factory_helper.h"

namespace tl {

ElementPickerWidget::ElementPickerWidget(QWidget *parent,
                                         BoardElement::ElementType type,
                                         BoardElement::TimeSeriesType tsType,
                                         BoardElement::TimeSeriesSize tsSize)
    : QWidget(parent), listWidget(new ElementListWidget(this))
{
    connect(listWidget, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)), this,
            SIGNAL(elementDoubleClicked()));

    QStringList elements = ElementFactory::Factory()->pluginslist();
    for (int i = 0; i < elements.count(); i++) {
        if (ElementFactory::Factory()->valid(elements.at(i))) {
            if (type != BoardElement::etUnknown) {
                auto typeElementFactory =
                    ElementFactory::Factory()->elementType(elements.at(i));
                auto timeseriesTypeElementFactory =
                    ElementFactory::Factory()->timeseriesType(elements.at(i));
                auto timeseriesSizeElementFactory =
                    ElementFactory::Factory()->timeseriesSize(elements.at(i));

                if (type != typeElementFactory)
                    continue;

                if (type == BoardElement::etTimeSeries) {
                    if (tsType != BoardElement::tstUnknown)
                        if (timeseriesTypeElementFactory != tsType)
                            continue;
                    if (timeseriesSizeElementFactory ==
                            BoardElement::tssSingle &&
                        tsSize == BoardElement::tssMulti)
                        continue;
                }
            }

            auto category = ElementFactory::Factory()->category(elements.at(i));

            if (category.isEmpty())
                category = QString("Uncategorized");

            QTreeWidgetItem *topLevel = nullptr;
            for (int j = 0; j < listWidget->topLevelItemCount(); j++) {
                auto topItem = listWidget->topLevelItem(j);
                if (topItem->text(0) == category) {
                    topLevel = topItem;
                    break;
                }
            }

            if (!topLevel) {
                topLevel = new QTreeWidgetItem(listWidget);
                topLevel->setText(0, category);
                listWidget->addTopLevelItem(topLevel);
            }

            QTreeWidgetItem *item = new QTreeWidgetItem(topLevel);
            item->setText(0, elements.at(i));
            item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable |
                           Qt::ItemIsDragEnabled);
            // FIXME: recover here
            item->setIcon(0, QIcon{});
            //            item->setIcon(0,
            //            create_icon(ElementFactory::Factory()->iconPixmap(
            //                                 elements.at(i))));
            //            item->setIcon(0,QIcon(ElementFactory::Factory()->iconPixmap(elements.at(i)).scaled(25,
            //            25, Qt::KeepAspectRatio)));
            item->setToolTip(
                0, ElementFactory::Factory()->tooltip(elements.at(i)));
            item->setSizeHint(0, QSize(32, 32));
        }
        else {
            qInfo() << "Element not valid" << elements.at(i);
        }
    }

    listWidget->sortItems(0, Qt::AscendingOrder);

    listWidget->expandAll();
}

ElementPickerWidget::~ElementPickerWidget() = default;

QString ElementPickerWidget::selectedElement()
{
    QString name;
    if (listWidget->selectedItems().count() > 0) {
        auto item = listWidget->selectedItems().at(0);
        if (item->parent())
            name = item->text(0);
    }

    return name;
}

} // namespace tl
