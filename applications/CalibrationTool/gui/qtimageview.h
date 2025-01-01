#pragma once

#include <QGraphicsView>

#include "qtcoreutils.h"

namespace tl {

class QtImageViewPrivate;
class QtImageView : public QGraphicsView
{
    Q_OBJECT

public:
    struct Options
    {
        bool showBackground{false};
        bool showOutline{true};
        bool fitToScreen{true};
    };

    explicit QtImageView(QWidget *parent = nullptr);
    ~QtImageView();

    /// Properties
    void setViewBackground(bool enable);
    void setViewOutline(bool enable);
    void setFitToScreen(bool fit);
    const Options &options() const;

    /// Actions
    void setImage(const QPixmap &pm);

public slots:
    void showImage(const QImage &image);

    void zoomIn();
    void zoomOut();
    void resetToOriginalSize();
    void fitToScreen();

    void clearAll();

signals:
    void scaleFactorChanged(qreal factor);
    void imageSizeChanged(const QSize &size);
    void fitToScreenChanged(bool fit);

protected:
    void drawBackground(QPainter *painter, const QRectF &rect) override;
    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    Q_DECLARE_PIMPL(QtImageView)
};

} // namespace tl
