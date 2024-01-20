#pragma once

#include <QGraphicsEffect>

#include "guiutils.h"

class QAbstractItemModel;
class QAbstractItemView;

namespace tl {

class ViewPlaceholderEffectPrivate;
class ViewPlaceholderEffect : public QGraphicsEffect
{
    Q_OBJECT

public:
    explicit ViewPlaceholderEffect(QAbstractItemModel* model,
                                   const QString& text = {},
                                   QObject* parent = nullptr);
    explicit ViewPlaceholderEffect(QAbstractItemView* view,
                                   const QString& text = {},
                                   QObject* parent = nullptr);
    ~ViewPlaceholderEffect();

    void setOpacity(qreal opacity);
    qreal opacity() const;

    void setModel(QAbstractItemModel* model);
    QAbstractItemModel* model() const;

    void setPlaceholderText(const QString& text, int timeout = 0);
    QString placeholderText() const;

    void setAlignment(Qt::Alignment alignment);
    Qt::Alignment alignment() const;

signals:
    void opacityChanged(qreal opacity);
    void linkActivated(const QString&);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void timerEvent(QTimerEvent* event) override;

    void draw(QPainter* painter) override;
    void sourceChanged(ChangeFlags flags) override;

private:
    Q_DECLARE_PIMPL(ViewPlaceholderEffect)
};

} // namespace tl
