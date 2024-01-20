#pragma once

#include <qcp/qcustomplot.h>

namespace tl {

class Board;

class BoardLayout : public QCPLayout
{
    Q_OBJECT
    Q_PROPERTY(int rowCount READ rowCount)
    Q_PROPERTY(int columnCount READ columnCount)
    Q_PROPERTY(int columnSpacing READ columnSpacing WRITE setColumnSpacing)
    Q_PROPERTY(int rowSpacing READ rowSpacing WRITE setRowSpacing)

public:
    explicit BoardLayout(Board *dashboard, int rows = 24, int cols = 24);
    ~BoardLayout() override;

    void clearLayout();

    // reimplemented virtual methods
    void update(UpdatePhase phase) override;
    void simplify() override;
    void updateLayout() override;
    int elementCount() const override;
    int elementIndex(int col, int row) const;
    QCPLayoutElement *elementAt(int index) const override;
    QCPLayoutElement *elementAt(const QPointF &pos) const;
    QCPLayoutElement *elementAt(int col, int row) const;
    QCPLayoutElement *elementAt(int col, int row, int width, int height) const;
    int colIndex(qreal x);
    int rowIndex(qreal y);
    QRect elementGridRectAt(int index) const;
    int elementRowAt(int index) const;
    int elementColumnAt(int index) const;
    int elementWidthAt(int index) const;
    int elementHeightAt(int index) const;
    int elementIndex(QCPLayoutElement *element) const;
    QCPLayoutElement *takeAt(int index) override;
    bool take(QCPLayoutElement *element) override;
    QList<QCPLayoutElement *> elements(bool recursive) const override;
    double selectTest(const QPointF &pos, bool onlySelectable,
                      QVariant *details = nullptr) const override;

    void saveElementRectSettings(int index, QSettings *settings);
    void saveElementRectSettings(QCPLayoutElement *element,
                                 QSettings *settings);
    QRect loadElementRectSettings(QSettings *settings);

    // non-virtual methods:
    bool addElement(QCPLayoutElement *element, bool replot, int col, int row,
                    int width = -1, int height = -1);
    bool addElement(QCPLayoutElement *element, QRect rect, bool replot = false);
    bool addElement(QCPLayoutElement *element, QPoint pos, bool replot = false);

    bool setElementGeometry(int index, int col, int row, int width = -1,
                            int height = -1);

    void updateElementMap();

    void setRowCount(int row);
    int rowCount() const;

    void setColumnCount(int col);
    int columnCount() const;

    int singleElementRowCount() const;
    int singleElementColumnCount() const;

    void setColumnSpacing(int spacing);
    int columnSpacing() const;
    void setRowSpacing(int spacing);
    int rowSpacing() const;

    double rowHeight() const;
    double columnWidth() const;
    QSizeF singleElementSize() const;
    void updateSingleElementSize();

    void setSingleElementRowCount(int singleElementRowCount);
    void setSingleElementColumnCount(int singleElementColumnCount);

    void setBackground(
        const QPixmap &pm, bool scaled,
        Qt::AspectRatioMode mode = Qt::KeepAspectRatioByExpanding);

    void setBackground(const QPixmap &pm);
    QPixmap background() const { return mBackgroundPixmap; }

    void setBackgroundScaled(bool scaled);
    bool backgroundScaled() const { return mBackgroundScaled; }

    void setBackgroundScaledMode(Qt::AspectRatioMode mode);
    Qt::AspectRatioMode backgroundScaledMode() const
    {
        return mBackgroundScaledMode;
    }

    // TODO: change name
    void setBackground(const QBrush &brush);
    QBrush backgroundBrush() const { return mBackgroundBrush; }

    void setBackgroundPen(const QPen &backgroundPen);
    QPen backgroundPen() const;

public slots:

signals:
    void pageModified();

protected:
    int mRowCount;
    int mColumnCount;
    double mRowHeight;
    double mColumnWidth;
    int mRowSpacing;
    int mColumnSpacing;

    int mSingleElementRowCount;
    int mSingleElementColumnCount;
    QSizeF mSingleElementSize;

    QList<QCPLayoutElement *> mElements;
    QList<QList<QCPLayoutElement *>> mElementsMap;
    QList<QRect> mElementsGridRect;

    QBrush mBackgroundBrush;
    QPen mBackgroundPen;
    QPixmap mBackgroundPixmap;
    QPixmap mScaledBackgroundPixmap;
    bool mBackgroundScaled;
    Qt::AspectRatioMode mBackgroundScaledMode;

    Board *mBoard;

    virtual void draw(QCPPainter *painter) override;
    void drawBackground(QCPPainter *painter);

private:
    Q_DISABLE_COPY(BoardLayout)
};

} // namespace tl
