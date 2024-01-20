#pragma once

#include <QObject>
#include <QPixmap>

namespace tl {

class BoardElement;

class ElementInterface : public QObject
{
public:
    virtual inline ~ElementInterface() = default;

    virtual QPixmap iconPixmap(QString path) { return QPixmap(path); }
    virtual BoardElement* createElement() = 0;
};

} // namespace tl

#define ElementInterfaceIid "board.elementPlugin"

Q_DECLARE_INTERFACE(tl::ElementInterface, ElementInterfaceIid)
