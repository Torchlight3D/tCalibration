#pragma once

#include <qcp/qcustomplot.h>

namespace thoht {

class Board;

class AdaptiveTextElement : public QCPTextElement
{
public:
    enum AdjustStrategy
    {
        asAdjustPointSize,
        asAdjustAndElide
    };

    explicit AdaptiveTextElement(Board *dashboard);

    void setDefaultColors();

    void setMaxPointSize(int max);
    int maxPointSize() const;
    void setMinPointSize(int min);
    int minPointSize() const;
    void setPointSize(int size);
    int pointSize() const;

    void setBackgroundBrush(const QBrush &background);
    QBrush backgroundBrush() const;

    void setAdjustStrategy(AdjustStrategy strategy);
    AdjustStrategy adjustStrategy() const;

    void needUpdate(bool needUpdate);
    bool needUpdate() const;

    void setBoldText(bool bold);
    void setCachedPixmap(bool pixmap);
    void setBorderPen(const QPen &pen);

    void update(QCPLayoutElement::UpdatePhase phase) override;
    void mousePressEvent(QMouseEvent *event, const QVariant &details) override;
    void mouseMoveEvent(QMouseEvent *event, const QPointF &startPos) override;
    void mouseDoubleClickEvent(QMouseEvent *event,
                               const QVariant &details) override;

protected:
    QSize minimumOuterSizeHint() const override;
    QSize maximumOuterSizeHint() const override;

    void draw(QCPPainter *painter) override;

protected:
    // property members:
    int mMaxPointSize;
    int mMinPointSize;
    QBrush mBackgroundBrush;
    QPen mBorderPen;

    QString mPreviousText;
    QString mTextDisplayed;
    QSizeF mPreviousSize;
    bool mNeedUpdate;
    AdjustStrategy mAdjustStrategy;

    bool mCachedPixmap;
    bool mCachedPixmapRendered;
    QPixmap mTextPixmap;
    Board *mBoard;
};

} // namespace thoht
