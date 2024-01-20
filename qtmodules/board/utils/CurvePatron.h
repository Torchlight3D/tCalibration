#pragma once

#include <QColor>
#include <QObject>
#include <QPen>

class QSettings;

namespace tl {

class CurvePatron : public QObject
{
    Q_OBJECT

public:
    explicit CurvePatron(QObject *parent = nullptr);

    Qt::PenStyle penStyle() const;
    QColor color() const;
    QPen pen() const;
    QList<double> xValues() const;
    QList<double> yValues() const;
    QString name() const;
    int penWidth() const;

    void clearPoints();
    void addPoint(double x, double y);
    void setPenStyle(const Qt::PenStyle &penStyle);
    void setColor(const QColor &color);
    void setName(const QString &name);
    void setPenWidth(int penWidth);

    void save(QSettings *settings);
    void load(QSettings *settings);

signals:

public slots:

protected:
    QString mName;
    QList<double> mXValues;
    QList<double> mYValues;
    Qt::PenStyle mPenStyle;
    QColor mColor;
    QPen mPen;
    int mPenWidth;
};
} // namespace tl
