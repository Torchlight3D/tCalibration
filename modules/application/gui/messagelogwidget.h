#pragma once

#include <QAbstractScrollArea>
#include "guiutils.h"

class MessageLogWidgetPrivate;
class MessageLogWidget : public QAbstractScrollArea
{
    Q_OBJECT

public:
    explicit MessageLogWidget(QWidget* parent = nullptr);
    ~MessageLogWidget();

    /// Properties
    // Specifies the maximum number of lines this widget will hold before
    // dropping old lines. Default INT_MAX.
    void setHistorySize(int size);
    unsigned int historySize() const;

    // Specifies the number of lines that should be visible at any one time.
    // Default 1.
    void setMinimumVisibleLines(int num);
    unsigned int minimumVisibleLines() const;

    // Specifies the number of columns that should be visible at any one time.
    // Default 1. The width is calculated using
    // QFontMetrics::averageCharWidth(), if that is available. Otherwise, we use
    // the width of character 'M'.
    void setMinimumVisibleColumns(int num);
    unsigned int minimumVisibleColumns() const;

    // Specifies whether the background should be drawn using row-alternating
    // colors. Default false.
    void setAlternatingRowColors(bool on);
    bool alternatingRowColors() const;

    int setupStyle(const QRegularExpression& regex, const QColor& background,
                   const QColor& foreground);
    int removeStyle(int id);
    void clearStyles();

    void message(const QString& msg);
    QString text() const;
    void clear();

    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;

protected:
    void scrollContentsBy(int dx, int dy) override;

    void paintEvent(QPaintEvent* event) override;
    void timerEvent(QTimerEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void changeEvent(QEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    Q_DECLARE_PIMPL(MessageLogWidget)
};
