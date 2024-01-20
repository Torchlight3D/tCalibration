#pragma once

#include <qcp/qcustomplot.h>

namespace tl {

class LayoutList : public QCPLayout
{
    Q_OBJECT

public:
    LayoutList();
    ~LayoutList() override;

    void clearLayout();

    // reimplemented virtual methods:
    void simplify() override;
    QCPLayoutElement *takeAt(int index) override;
    bool take(QCPLayoutElement *element) override;
    void updateLayout() override;
    int elementCount() const override;
    QCPLayoutElement *elementAt(int index) const override;

    void addElement(QCPLayoutElement *element);
    void takeAllElements();

    void setRowHeight(int rowHeight);
    int rowHeight() const;
    void setRowSpacing(int rowSpacing);
    int rowSpacing() const;

protected:
    int mRowHeight;
    int mRowSpacing;

    QList<QCPLayoutElement *> mElements;

    friend class AlarmPanel;
};

} // namespace tl
