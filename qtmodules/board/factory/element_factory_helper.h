#pragma once

#include <QPixmap>
#include <QString>

#include "../BoardElement.h"

namespace thoht {

class ElementFactory
{
public:
    static ElementFactory *Factory()
    {
        static ElementFactory factory;
        return &factory;
    }

    BoardElement *createElement(const QString &name);
    QString description(const QString &name);
    QString tooltip(const QString &name);
    QString category(const QString &name);
    QPixmap iconPixmap(const QString &name);
    QString iconPath(const QString &name);
    BoardElement::ElementType elementType(const QString &name);
    BoardElement::TimeSeriesType timeseriesType(const QString &name);
    BoardElement::TimeSeriesSize timeseriesSize(const QString &name);
    bool valid(const QString &name);
    QStringList pluginslist() const;

private:
};

} // namespace thoht
