#pragma once

#include <QWidget>

#include "qtcoreutils.h"

namespace tl {

class QtLedIndicatorPrivate;
class QtLedIndicator : public QWidget
{
    Q_OBJECT

public:
    explicit QtLedIndicator(QWidget* parent = nullptr);
    ~QtLedIndicator();

    void setSize(const QSize& size);

    enum Shape
    {
        Elipse = 0,
        Rect, // Default
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

    bool on() const;
    void setOn(bool on = true);
    inline void setOff() { setOn(false); }
    inline void toggleOnOff() { setOn(!on()); }

    bool blinking() const;
    void setBlinking(bool on);

    int blinkCycle() const;
    void setBlinkCycle(int msec);

    enum class Status
    {
        Off = 0, // Default
        On,
        Blink,
    };
    // Convenient function to change status
    void setStatus(Status status);

    // Returns if the led state can be changed with a click
    bool clickable() const;
    void setClickable(bool on);

    bool flat() const;
    void setFlat(bool on);

    enum class State
    {
        Idle = 0, // Light off
        Busy,     // Light blinks, green
        Warning,  // Light blinks, yellow
        Success,  // Light on, green
        Failure,  // Light on, red
    };
    void setState(State state);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;

private:
    Q_DECLARE_PIMPL(QtLedIndicator)
};

} // namespace tl
