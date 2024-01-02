#pragma once

#include <QAbstractButton>

#include "TheStyleTypes.h"

namespace thoht {

class ColorButton : public QAbstractButton
{
    Q_OBJECT
    Q_PROPERTY(QColor color READ color WRITE setColor NOTIFY colorChanged)
    Q_PROPERTY(ColorMode colorMode READ colorMode WRITE setColorMode NOTIFY
                   colorModeChanged)

public:
    ColorButton(QWidget* parent = nullptr);
    ColorButton(const QColor& color, QWidget* parent = nullptr);
    ColorButton(const QColor& color, ColorMode mode, QWidget* parent = nullptr);

    const QColor& color() const;
    void setColor(const QColor& color);

    ColorMode colorMode() const;
    void setColorMode(ColorMode mode);

    QSize sizeHint() const override;

signals:
    void colorChanged();
    void colorModeChanged();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void setup();
    QColor adaptColorToMode(const QColor& color) const;

private:
    QColor _color;
    ColorMode _colorMode{ColorMode::RGBA};
};

} // namespace thoht
