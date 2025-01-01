#pragma once

#include <QWidget>

#include "qtcoreutils.h"

namespace tl {

class QtWaitingSpinnerPrivate;
class QtWaitingSpinner : public QWidget
{
    Q_OBJECT

public:
    explicit QtWaitingSpinner(QWidget *parent = nullptr,
                              bool centerOnParent = true,
                              bool disableParentWhenSpinning = true);

    explicit QtWaitingSpinner(Qt::WindowModality modality,
                              QWidget *parent = nullptr,
                              bool centerOnParent = true,
                              bool disableParentWhenSpinning = true);
    ~QtWaitingSpinner();

    /// Properties
    QColor color() const;
    void setColor(const QColor &color);

    qreal roundness() const;
    void setRoundness(qreal roundness);

    qreal minimumTrailOpacity() const;
    void setMinimumTrailOpacity(qreal val);

    qreal trailFadePercentage() const;
    void setTrailFadePercentage(qreal val);

    qreal revolutionsPersSecond() const;
    void setRevolutionsPerSecond(qreal val);

    int numberOfLines() const;
    void setNumberOfLines(int lines);

    int lineLength() const;
    void setLineLength(int length);

    int lineWidth() const;
    void setLineWidth(int width);

    int innerRadius() const;
    void setInnerRadius(int radius);

    QString text() const;
    void setText(const QString &text);

    /// Actions
    void start();
    void stop();
    bool isSpinning() const;

protected:
    void paintEvent(QPaintEvent *event) override;

private slots:
    void rotate();

private:
    Q_DECLARE_PIMPL(QtWaitingSpinner)
};

} // namespace tl
