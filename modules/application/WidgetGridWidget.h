#pragma once

#include <QWidget>

#include "gui/guiutils.h"

namespace tl {

class WidgetGridWidgetPrivate;
class WidgetGridWidget : public QWidget
{
    Q_OBJECT

public:
    explicit WidgetGridWidget(QWidget *parent = nullptr);
    ~WidgetGridWidget();

    enum GridType
    {
        FourView,
        SixView,
        EightView,
        NineView
    };

    GridType gridType() const;
    void setGridType(GridType type);

    int viewCount() const;
    void setViewCount() const;

    // TODO: How to customize view type???

public slots:
    // TODO: How to customize action ???
    void showImage(const QImage &image, int index);

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    Q_DECLARE_PIMPL(WidgetGridWidget)
};

} // namespace tl
