#pragma once

#include <QColor>
#include <QDomElement>
#include <QString>
#include <QStringList>

namespace io {

// POD
qreal qrealFromDom(const QDomElement &e, const QString &attribute, qreal value);
int intFromDom(const QDomElement &e, const QString &attribute, int value);
unsigned int uintFromDom(const QDomElement &e, const QString &attribute,
                         unsigned int value);

bool boolFromDom(const QDomElement &e, const QString &attribute, bool value);
void setBoolAttribute(QDomElement &e, const QString &attribute, bool value);

// QColor
QDomElement QColorDomElement(const QColor &color, const QString &name,
                             QDomDocument &doc);
QColor QColorFromDom(const QDomElement &e);

}; // namespace io
