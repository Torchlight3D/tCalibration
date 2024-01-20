#include "ViewPlaceholderEffect.h"

#include <QAbstractItemModel>
#include <QAbstractItemView>
#include <QApplication>
#include <QEvent>
#include <QPainter>
#include <QPointer>
#include <QTextOption>
#include <QTextDocument>
#include <QAbstractTextDocumentLayout>

namespace tl {

class ViewPlaceholderEffectPrivate
{
    Q_DEFINE_PIMPL(ViewPlaceholderEffect)

public:
    explicit ViewPlaceholderEffectPrivate(ViewPlaceholderEffect *q,
                                          QAbstractItemModel *model = nullptr);

    void init();

    QPointF offsetPoint(const QRectF &rect) const;
    void drawText(QPainter *painter, const QRectF &rect);

public:
    QTextDocument m_document;
    QTextOption m_textOptions;
    QString m_placeholderText;
    QPointF m_displacement;
    qreal m_opacity;
    QPointer<QAbstractItemModel> m_model;
    QWidget *m_viewport;
    int timerId;
};

ViewPlaceholderEffectPrivate::ViewPlaceholderEffectPrivate(
    ViewPlaceholderEffect *q, QAbstractItemModel *m)
    : q_ptr(q), m_document(q), m_opacity(1.0), m_model(m), timerId(-1)
{
}

void ViewPlaceholderEffectPrivate::init()
{
    m_textOptions.setAlignment(Qt::AlignCenter);
    m_textOptions.setWrapMode(QTextOption::WordWrap);
    m_document.setDefaultTextOption(m_textOptions);
}

QPointF ViewPlaceholderEffectPrivate::offsetPoint(const QRectF &rect) const
{
    QPointF pos;

    // document.setTextWidth(rect.width());
    const auto size = m_document.size();
    const auto align = m_textOptions.alignment();
    if (align & Qt::AlignLeft) {
        pos.setY(0);
    }
    if (align & Qt::AlignRight) {
        pos.setX(qMax(rect.width() - size.width(), 0.));
    }
    if (align & Qt::AlignTop) {
        pos.setX(0);
    }
    if (align & Qt::AlignBottom) {
        pos.setX(qMax(rect.height() - size.height(), 0.));
    }
    if (align & Qt::AlignHCenter) {
        pos.setX(qMax((rect.width() - size.width()) / 2, 0.));
    }
    if (align & Qt::AlignVCenter) {
        pos.setY(qMax((rect.height() - size.height()) / 2, 0.));
    }
    return pos;
}

void ViewPlaceholderEffectPrivate::drawText(QPainter *painter,
                                            const QRectF &rect)
{
    m_displacement = offsetPoint(rect);

    painter->save();
    painter->setOpacity(m_opacity);
    painter->translate(m_displacement);
    m_document.drawContents(painter, rect);
    painter->restore();
}

ViewPlaceholderEffect::ViewPlaceholderEffect(QAbstractItemModel *model,
                                             const QString &text,
                                             QObject *parent)
    : QGraphicsEffect(parent),
      d_ptr(new ViewPlaceholderEffectPrivate(this, model))
{
    d_ptr->init();
    setPlaceholderText(text);
}

ViewPlaceholderEffect::ViewPlaceholderEffect(QAbstractItemView *view,
                                             const QString &text,
                                             QObject *parent)
    : QGraphicsEffect(parent), d_ptr(new ViewPlaceholderEffectPrivate(this))
{
    Q_D(ViewPlaceholderEffect);
    if (view) {
        d->m_model = view->model();
        d->m_viewport = view->viewport();
        d->m_viewport->setGraphicsEffect(this);
        d->m_viewport->installEventFilter(this);
    }
    setPlaceholderText(text);
}

ViewPlaceholderEffect::~ViewPlaceholderEffect() = default;

void ViewPlaceholderEffect::setOpacity(qreal opacity)
{
    Q_D(ViewPlaceholderEffect);
    if (d->m_opacity == opacity) {
        return;
    }

    d->m_opacity = opacity;
    emit opacityChanged(d->m_opacity);
}

qreal ViewPlaceholderEffect::opacity() const
{
    Q_D(const ViewPlaceholderEffect);
    return d->m_opacity;
}

void ViewPlaceholderEffect::setModel(QAbstractItemModel *model)
{
    Q_D(ViewPlaceholderEffect);
    if (d->m_model == model) {
        return;
    }

    d->m_model = model;
    update();
}

QAbstractItemModel *ViewPlaceholderEffect::model() const
{
    Q_D(const ViewPlaceholderEffect);
    return d->m_model;
}

void ViewPlaceholderEffect::setPlaceholderText(const QString &text, int timeout)
{
    Q_D(ViewPlaceholderEffect);
    if (d->m_placeholderText != text) {
        d->m_placeholderText = text;
        if (Qt::mightBeRichText(d->m_placeholderText)) {
            d->m_document.setHtml(d->m_placeholderText);
        }
        else {
            d->m_document.setPlainText(d->m_placeholderText);
        }
        update();
    }

    if (timeout != 0) {
        d->timerId = startTimer(timeout);
    }
}

QString ViewPlaceholderEffect::placeholderText() const
{
    Q_D(const ViewPlaceholderEffect);
    return d->m_placeholderText;
}

void ViewPlaceholderEffect::setAlignment(Qt::Alignment align)
{
    Q_D(ViewPlaceholderEffect);
    d->m_textOptions.setAlignment(align);
    d->m_document.setDefaultTextOption(d->m_textOptions);
    update();
}

Qt::Alignment ViewPlaceholderEffect::alignment() const
{
    Q_D(const ViewPlaceholderEffect);
    return d->m_textOptions.alignment();
}

void ViewPlaceholderEffect::draw(QPainter *painter)
{
    Q_D(ViewPlaceholderEffect);

    painter->save();
    drawSource(painter);
    painter->restore();

    if (d->m_model &&
        (d->m_model->rowCount() == 0 || d->m_model->columnCount() == 0)) {
        d->drawText(painter, boundingRect());
    }
}

void ViewPlaceholderEffect::sourceChanged(ChangeFlags flags)
{
    Q_D(ViewPlaceholderEffect);
    QGraphicsEffect::sourceChanged(flags);
    d->m_displacement = d->offsetPoint(boundingRect());
}

bool ViewPlaceholderEffect::eventFilter(QObject *watched, QEvent *event)
{
    Q_D(ViewPlaceholderEffect);
    if (watched == d->m_viewport &&
        event->type() == QEvent::MouseButtonRelease) {
        auto *mouseEvent = static_cast<QMouseEvent *>(event);
        QPoint pos = mouseEvent->pos() - d->m_displacement.toPoint();
        int position =
            d->m_document.documentLayout()->hitTest(pos, Qt::FuzzyHit);
        QTextCursor cursor(&d->m_document);
        cursor.setPosition(position);
        cursor.select(QTextCursor::WordUnderCursor);
        QTextCharFormat fmt = cursor.charFormat();
        if (fmt.isAnchor()) {
            emit linkActivated(fmt.anchorHref());
        }
    }
    return QObject::eventFilter(watched, event);
}

void ViewPlaceholderEffect::timerEvent(QTimerEvent *event)
{
    Q_D(ViewPlaceholderEffect);
    if (event->timerId() != d->timerId) {
        return;
    }

    killTimer(d->timerId);
    d->timerId = -1;
    d->m_placeholderText.clear();
    d->m_document.clear();
    update();
}

} // namespace tl

#include "moc_ViewPlaceholderEffect.cpp"
