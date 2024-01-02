#include "ImageView.h"

#include <QFileInfo>
#include <QGraphicsRectItem>
#include <QWheelEvent>

namespace thoht {

namespace {

constexpr qreal kDefaultScaleFactor{1.2};
constexpr qreal kZoomLevels[]{0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 4.0, 8.0};

inline qreal nextLevel(qreal current)
{
    const auto found =
        std::find_if(std::begin(kZoomLevels), std::end(kZoomLevels),
                     [&current](qreal val) { return val > current; });
    return found == std::end(kZoomLevels) ? current : (*found);
}

inline qreal previousLevel(qreal current)
{
    const auto found =
        std::find_if(std::rbegin(kZoomLevels), std::rend(kZoomLevels),
                     [&current](qreal val) { return val < current; });
    return found == std::rend(kZoomLevels) ? current : (*found);
}

} // namespace

//------- ImageViewPrivate starts from here
class ImageViewPrivate
{
    Q_DISABLE_COPY(ImageViewPrivate)
    Q_DECLARE_PUBLIC(ImageView)
    ImageView *const q_ptr;

public:
    explicit ImageViewPrivate(ImageView *q);

    void init();

    void scaleImpl(qreal factor);
    void fitToScreenImpl();
    void emitScaleFactor();

public:
    // TODO: who manage QGraphicsItem live cycle?
    QGraphicsPixmapItem *const m_imageItem{nullptr};
    QGraphicsRectItem *const m_backgroundItem{nullptr};
    QGraphicsRectItem *const m_outlineItem{nullptr};
    ImageView::Options m_settings;
};

ImageViewPrivate::ImageViewPrivate(ImageView *q)
    : q_ptr(q),
      m_imageItem(new QGraphicsPixmapItem),
      m_backgroundItem(new QGraphicsRectItem),
      m_outlineItem(new QGraphicsRectItem)
{
}

void ImageViewPrivate::init()
{
    Q_Q(ImageView);
    q->setScene(new QGraphicsScene(q));
    q->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    q->setDragMode(QGraphicsView::ScrollHandDrag);
    q->setInteractive(false);
    q->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    q->setFrameShape(QFrame::NoFrame);
    q->setRenderHint(QPainter::SmoothPixmapTransform);

    // Draw background chessboard pattern
    QPixmap tilePixmap{64, 64};
    tilePixmap.fill(Qt::white);
    QPainter tilePainter{&tilePixmap};
    constexpr QColor kDefaultColor{220, 220, 220};
    tilePainter.fillRect(0, 0, 0x20, 0x20, kDefaultColor);
    tilePainter.fillRect(0x20, 0x20, 0x20, 0x20, kDefaultColor);
    tilePainter.end();
    q->setBackgroundBrush(tilePixmap);

    m_imageItem->setCacheMode(QGraphicsItem::NoCache);
    m_imageItem->setZValue(0);

    m_backgroundItem->setBrush(Qt::white);
    m_backgroundItem->setPen(Qt::NoPen);
    m_backgroundItem->setVisible(m_settings.showBackground);
    m_backgroundItem->setZValue(-1);

    QPen outline{Qt::black, 1, Qt::DashLine};
    outline.setCosmetic(true);
    m_outlineItem->setPen(outline);
    m_outlineItem->setBrush(Qt::NoBrush);
    m_outlineItem->setVisible(m_settings.showOutline);
    m_outlineItem->setZValue(1);

    auto *s = q->scene();
    s->addItem(m_backgroundItem);
    s->addItem(m_imageItem);
    s->addItem(m_outlineItem);
}

void ImageViewPrivate::scaleImpl(qreal factor)
{
    Q_Q(ImageView);
    q->scale(factor, factor);
    emitScaleFactor();
    if (auto pixmapItem =
            qgraphicsitem_cast<QGraphicsPixmapItem *>(m_imageItem)) {
        pixmapItem->setTransformationMode(q->transform().m11() < 1
                                              ? Qt::SmoothTransformation
                                              : Qt::FastTransformation);
    }
}

void ImageViewPrivate::fitToScreenImpl()
{
    Q_Q(ImageView);
    q->fitInView(m_imageItem, Qt::KeepAspectRatio);
    emitScaleFactor();
}

void ImageViewPrivate::emitScaleFactor()
{
    Q_Q(ImageView);
    emit q->scaleFactorChanged(q->transform().m11());
}

//------- ImageViewPrivate starts from here
ImageView::ImageView(QWidget *parent)
    : QGraphicsView(parent), d_ptr(new ImageViewPrivate(this))
{
    d_ptr->init();
}

ImageView::~ImageView() = default;

void ImageView::setViewBackground(bool enable)
{
    Q_D(ImageView);
    d->m_settings.showBackground = enable;
    d->m_backgroundItem->setVisible(enable);
}

void ImageView::setViewOutline(bool enable)
{
    Q_D(ImageView);
    d->m_settings.showOutline = enable;
    d->m_outlineItem->setVisible(enable);
}

void ImageView::setFitToScreen(bool fit)
{
    Q_D(ImageView);
    if (fit == d->m_settings.fitToScreen) {
        return;
    }

    d->m_settings.fitToScreen = fit;
    if (d->m_settings.fitToScreen) {
        d->fitToScreenImpl();
    }
    emit fitToScreenChanged(d->m_settings.fitToScreen);
}

const ImageView::Options &ImageView::options() const
{
    Q_D(const ImageView);
    return d->m_settings;
}

void ImageView::setImage(const QPixmap &img)
{
    Q_D(ImageView);
    if (img.isNull()) {
        return;
    }

    d->m_imageItem->setPixmap(img);
    d->m_backgroundItem->setRect(d->m_imageItem->boundingRect());
    d->m_outlineItem->setRect(d->m_imageItem->boundingRect());
}

void ImageView::showImage(const QImage &image)
{
    setImage(QPixmap::fromImage(image));
}

void ImageView::zoomIn()
{
    Q_D(ImageView);
    setFitToScreen(false);
    const qreal nextZoomLevel = nextLevel(transform().m11());
    resetTransform();
    d->scaleImpl(nextZoomLevel);
}

void ImageView::zoomOut()
{
    Q_D(ImageView);
    setFitToScreen(false);
    const qreal previousZoomLevel = previousLevel(transform().m11());
    resetTransform();
    d->scaleImpl(previousZoomLevel);
}

void ImageView::resetToOriginalSize()
{
    Q_D(ImageView);
    setFitToScreen(false);
    resetTransform();
    d->emitScaleFactor();
}

void ImageView::fitToScreen()
{
    //
}

void ImageView::clearAll()
{
    scene()->clear();
    resetTransform();
}

void ImageView::drawBackground(QPainter *painter, const QRectF & /*rect*/)
{
    painter->save();
    painter->resetTransform();
    painter->setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter->drawTiledPixmap(viewport()->rect(), backgroundBrush().texture());
    painter->restore();
}

void ImageView::showEvent(QShowEvent * /*event */)
{
    //    m_file->updateVisibility();
}

void ImageView::hideEvent(QHideEvent * /*event */)
{
    //    m_file->updateVisibility();
}

void ImageView::wheelEvent(QWheelEvent *event)
{
    Q_D(ImageView);
    setFitToScreen(false);
    const qreal factor =
        qPow(kDefaultScaleFactor, event->angleDelta().y() / 240.);
    const qreal actualFactor = qBound(0.001, factor, 1000.);
    d->scaleImpl(actualFactor);
    event->accept();
}

void ImageView::resizeEvent(QResizeEvent *event)
{
    Q_D(ImageView);
    QGraphicsView::resizeEvent(event);
    if (d->m_settings.fitToScreen) {
        d->fitToScreenImpl();
    }
}

} // namespace thoht

#include "moc_ImageView.cpp"
