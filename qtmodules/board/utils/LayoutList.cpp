#include "LayoutList.h"
#include "../BoardElement.h"

namespace tl {

LayoutList::LayoutList() : QCPLayout(), mRowHeight(20), mRowSpacing(1)
{
    setMinimumMargins({2, 2, 2, 2});
    setMargins({2, 2, 2, 2});
}

LayoutList::~LayoutList() { clearLayout(); }

void LayoutList::clearLayout()
{
    int size = elementCount();
    for (int i = 0; i < size; i++) {
        if (QCPLayoutElement *el = takeAt(0)) {
            if (auto boardEl = qobject_cast<BoardElement *>(el))
                boardEl->clearElement();
            delete el;
        }
    }
    simplify();
}

void LayoutList::simplify()
{
    int size = elementCount();
    for (int i = size - 1; i >= 0; i--) {
        QCPLayoutElement *el = takeAt(0);
        if (el) {
            auto boardEl = qobject_cast<BoardElement *>(el);
            if (boardEl)
                boardEl->clearElement();
            delete el;
        }
    }
}

QCPLayoutElement *LayoutList::takeAt(int index)
{
    if (QCPLayoutElement *el = elementAt(index)) {
        releaseElement(el);
        mElements.removeAt(index);
        return el;
    }
    else {
        qDebug() << Q_FUNC_INFO << "Attempt to take invalid index:" << index;
        return nullptr;
    }
}

bool LayoutList::take(QCPLayoutElement *element)
{
    if (element) {
        for (int i = 0; i < elementCount(); ++i) {
            if (elementAt(i) == element) {
                takeAt(i);
                return true;
            }
        }
        qDebug() << Q_FUNC_INFO << "Element not in this layout, couldn't take";
    }
    else {
        qDebug() << Q_FUNC_INFO << "Can't take null element";
    }
    return false;
}

void LayoutList::updateLayout()
{
    int yOffset = mRect.top();
    double yOffsetLimit = mRect.bottom() - mRowHeight;
    for (int row = 0; row < mElements.size(); ++row) {
        if (row > 0)
            yOffset += mRowHeight + mRowSpacing;
        int xOffset = mRect.left();

        if (mElements.at(row)) {
            if (yOffset < yOffsetLimit)
                mElements.at(row)->setOuterRect(
                    {xOffset, yOffset, rect().width(), mRowHeight});
            else
                mElements.at(row)->setOuterRect({xOffset, yOffset, 0, 0});
        }
    }
}

int LayoutList::elementCount() const { return mElements.size(); }

QCPLayoutElement *LayoutList::elementAt(int index) const
{
    if (index >= 0 && index < mElements.size())
        return mElements.at(index);

    return nullptr;
}

void LayoutList::addElement(QCPLayoutElement *element)
{
    if (element) {
        mElements.append(element);
        adoptElement(element);
    }
    else {
        qDebug() << Q_FUNC_INFO << "Can't add null element";
    }
}

void LayoutList::takeAllElements()
{
    int size = elementCount();
    for (int i = 0; i < size; i++) takeAt(0);
    simplify();
}

void LayoutList::setRowHeight(int rowHeight) { mRowHeight = rowHeight; }

int LayoutList::rowHeight() const { return mRowHeight; }

int LayoutList::rowSpacing() const { return mRowSpacing; }

void LayoutList::setRowSpacing(int rowSpacing) { mRowSpacing = rowSpacing; }

} // namespace tl
