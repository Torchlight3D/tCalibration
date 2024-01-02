#pragma once

#include <QObject>
#include <QPixmap>

namespace thoht {

class BoardElement;

class ElementInterface : public QObject
{
public:
    virtual inline ~ElementInterface() = default;

    virtual QPixmap iconPixmap(QString path) { return QPixmap(path); }
    virtual BoardElement* createElement() = 0;
};

} // namespace thoht

#define ElementInterfaceIid "board.elementPlugin"

Q_DECLARE_INTERFACE(thoht::ElementInterface, ElementInterfaceIid)
