#include "qtimageview.h"

#include <QFileInfo>
#include <QGraphicsRectItem>
#include <QWheelEvent>

namespace tl {

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

//------- QtImageViewPrivate starts from here
class QtImageViewPrivate
{
    Q_DEFINE_PIMPL(QtImageView)

public:
    explicit QtImageViewPrivate(QtImageView *q);

    void init();

    void scaleImpl(qreal factor);
    void fitToScreenImpl();
    void emitScaleFactor();

public:
    // TODO: who manage QGraphicsItem live cycle?
    QGraphicsPixmapItem *const m_imageItem{nullptr};
    QGraphicsRectItem *const m_backgroundItem{nullptr};
    QGraphicsRectItem *const m_outlineItem{nullptr};
    QtImageView::Options m_settings;
};

QtImageViewPrivate::QtImageViewPrivate(QtImageView *q)
    : q_ptr(q),
      m_imageItem(new QGraphicsPixmapItem),
      m_backgroundItem(new QGraphicsRectItem),
      m_outlineItem(new QGraphicsRectItem)
{
}

void QtImageViewPrivate::init()
{
    Q_Q(QtImageView);
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

void QtImageViewPrivate::scaleImpl(qreal factor)
{
    Q_Q(QtImageView);
    q->scale(factor, factor);
    emitScaleFactor();
    if (auto pixmapItem =
            qgraphicsitem_cast<QGraphicsPixmapItem *>(m_imageItem)) {
        pixmapItem->setTransformationMode(q->transform().m11() < 1
                                              ? Qt::SmoothTransformation
                                              : Qt::FastTransformation);
    }
}

void QtImageViewPrivate::fitToScreenImpl()
{
    Q_Q(QtImageView);
    q->fitInView(m_imageItem, Qt::KeepAspectRatio);
    emitScaleFactor();
}

void QtImageViewPrivate::emitScaleFactor()
{
    Q_Q(QtImageView);
    emit q->scaleFactorChanged(q->transform().m11());
}

//------- QtImageViewPrivate starts from here
QtImageView::QtImageView(QWidget *parent)
    : QGraphicsView(parent), d_ptr(new QtImageViewPrivate(this))
{
    d_ptr->init();
}

QtImageView::~QtImageView() = default;

void QtImageView::setViewBackground(bool enable)
{
    Q_D(QtImageView);
    d->m_settings.showBackground = enable;
    d->m_backgroundItem->setVisible(enable);
}

void QtImageView::setViewOutline(bool enable)
{
    Q_D(QtImageView);
    d->m_settings.showOutline = enable;
    d->m_outlineItem->setVisible(enable);
}

void QtImageView::setFitToScreen(bool fit)
{
    Q_D(QtImageView);
    if (fit == d->m_settings.fitToScreen) {
        return;
    }

    d->m_settings.fitToScreen = fit;
    if (d->m_settings.fitToScreen) {
        d->fitToScreenImpl();
    }
    emit fitToScreenChanged(d->m_settings.fitToScreen);
}

const QtImageView::Options &QtImageView::options() const
{
    Q_D(const QtImageView);
    return d->m_settings;
}

void QtImageView::setImage(const QPixmap &img)
{
    Q_D(QtImageView);
    if (img.isNull()) {
        return;
    }

    d->m_imageItem->setPixmap(img);
    d->m_backgroundItem->setRect(d->m_imageItem->boundingRect());
    d->m_outlineItem->setRect(d->m_imageItem->boundingRect());
}

void QtImageView::showImage(const QImage &image)
{
    setImage(QPixmap::fromImage(image));
}

void QtImageView::zoomIn()
{
    Q_D(QtImageView);
    setFitToScreen(false);
    const qreal nextZoomLevel = nextLevel(transform().m11());
    resetTransform();
    d->scaleImpl(nextZoomLevel);
}

void QtImageView::zoomOut()
{
    Q_D(QtImageView);
    setFitToScreen(false);
    const qreal previousZoomLevel = previousLevel(transform().m11());
    resetTransform();
    d->scaleImpl(previousZoomLevel);
}

void QtImageView::resetToOriginalSize()
{
    Q_D(QtImageView);
    setFitToScreen(false);
    resetTransform();
    d->emitScaleFactor();
}

void QtImageView::fitToScreen()
{
    //
}

void QtImageView::clearAll()
{
    scene()->clear();
    resetTransform();
}

void QtImageView::drawBackground(QPainter *painter, const QRectF & /*rect*/)
{
    painter->save();
    painter->resetTransform();
    painter->setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter->drawTiledPixmap(viewport()->rect(), backgroundBrush().texture());
    painter->restore();
}

void QtImageView::showEvent(QShowEvent * /*event */)
{
    //    m_file->updateVisibility();
}

void QtImageView::hideEvent(QHideEvent * /*event */)
{
    //    m_file->updateVisibility();
}

void QtImageView::wheelEvent(QWheelEvent *event)
{
    Q_D(QtImageView);
    setFitToScreen(false);
    const qreal factor =
        qPow(kDefaultScaleFactor, event->angleDelta().y() / 240.);
    const qreal actualFactor = qBound(0.001, factor, 1000.);
    d->scaleImpl(actualFactor);
    event->accept();
}

void QtImageView::resizeEvent(QResizeEvent *event)
{
    Q_D(QtImageView);
    QGraphicsView::resizeEvent(event);
    if (d->m_settings.fitToScreen) {
        d->fitToScreenImpl();
    }
}

} // namespace tl

#include "moc_qtimageview.cpp"
