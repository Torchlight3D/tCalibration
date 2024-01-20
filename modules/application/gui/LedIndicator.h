#pragma once

#include <QWidget>
#include "guiutils.h"

namespace tl {

class LedIndicatorPrivate;
class LedIndicator : public QWidget
{
    Q_OBJECT

public:
    explicit LedIndicator(QWidget* parent = nullptr);
    ~LedIndicator();

    bool state() const;
    void setState(bool state);
    inline void toggleState() { setState(!state()); }

    enum Shape
    {
        Elipse = 0,
        Rect,
        RoundRect,
        Triangle
    };
    Q_ENUM(Shape)
    Shape shape() const;
    void setShape(Shape shape);

    const QColor& onColor() const;
    void setOnColor(const QColor& color);
    static QColor defaultOnColor();

    const QColor& offColor() const;
    void setOffColor(const QColor& offColor);
    static QColor defaultOffColor();

    bool blinking() const;
    void setBlinking(bool on);

    // In ms
    int blinkCycle() const;
    void setBlinkCycle(int cycle);

    enum class Status
    {
        Off,
        On,
        Blink,
    };
    // Convenient function to change status
    void setStatus(Status status);

    // Returns if the led state can be changed with a click
    bool clickable() const;
    void setClickable(bool on);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;

private:
    Q_DECLARE_PIMPL(LedIndicator)
};

} // namespace tl
