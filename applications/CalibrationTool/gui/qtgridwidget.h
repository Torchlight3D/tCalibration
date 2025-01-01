#pragma once

#include <QWidget>

#include "qtcoreutils.h"

namespace tl {

class QtGridWidgetPrivate;
class QtGridWidget : public QWidget
{
    Q_OBJECT

public:
    explicit QtGridWidget(QWidget *parent = nullptr);
    ~QtGridWidget();

    // Use enum to ensure only supported layout style is set.
    enum LayoutStyle
    {
        SingleView = 1,
        DualViews = 2,
        FourViews = 4,
        SixViews = 6,
        EightViews = 8,
        NineViews = 9
    };

    LayoutStyle layoutStyle() const;
    inline int viewsPerPage() const { return layoutStyle(); }
    void setLayoutStyle(LayoutStyle style);

    void bindWidgets(QWidgetList widgets);

public slots:
    void showWidgetAt(qsizetype index);

signals:
    void pageChanged(qsizetype page, int viewsPerPage);

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    Q_DECLARE_PIMPL(QtGridWidget)
};

} // namespace tl
