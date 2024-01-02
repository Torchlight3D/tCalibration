#pragma once

#include <QEasingCurve>
#include <QEvent>
#include <QObject>
#include <QVariantAnimation>
#include <QWidget>

namespace thoht {

template <typename T>
// This is just a wrapper around QVariantAnimation to get typed animated values.
class WidgetAnimation : public QObject
{
public:
    explicit WidgetAnimation(QWidget* parent = nullptr) : QObject(parent)
    {
        Q_ASSERT(parent);
        if (parent) {
            _qVariantAnimation.setEasingCurve(QEasingCurve::Type::OutCubic);
            QObject::connect(
                &_qVariantAnimation, &QVariantAnimation::valueChanged, parent,
                [parent](const QVariant&) {
                    // Force widget repaint.
                    parent->update();
                },
                Qt::ConnectionType::QueuedConnection);
            parent->installEventFilter(this);
        }
    }

    ~WidgetAnimation() override { stop(); }

    void start()
    {
        _qVariantAnimation.setLoopCount(_loopEnabled ? -1 : 1);
        _qVariantAnimation.start();
    }

    void stop()
    {
        _qVariantAnimation.stop();
        _qVariantAnimation.setLoopCount(1);
        if (hasFinalValue()) {
            setStartValue(_finalValue);
        }
    }

    void setLoopEnabled(bool enabled) { _loopEnabled = enabled; }

    bool loopEnabled() const { return _loopEnabled; }

    void restart(const T& value)
    {
        stop();

        // Ensure it has a start value.
        if (!hasStartValue()) {
            if (loopEnabled()) {
                setStartValue(T());
            }
            else {
                setStartValue(value);
            }
        }
        else {
            setStartValue(this->value());
        }

        setFinalValue(value);
        start();
    }

    void restartIfNeeded(const T& value)
    {
        if (value != finalValue() || !hasFinalValue()) {
            restart(value);
        }
    }

    bool isRunning() const
    {
        return _qVariantAnimation.state() == QVariantAnimation::Running;
    }

    void setDuration(int const milliseconds)
    {
        if (milliseconds != _qVariantAnimation.duration()) {
            stop();
            _qVariantAnimation.setDuration(milliseconds);
        }
    }

    int duration() const { return _qVariantAnimation.duration(); }

    const T& finalValue() const { return _finalValue; }

    void setFinalValue(const T& value)
    {
        if (value != _finalValue || !hasFinalValue()) {
            // Ensure it has a start value.
            if (!hasStartValue()) {
                setStartValue(value);
            }

            _finalValue = value;
            _qVariantAnimation.setEndValue(QVariant::fromValue<T>(value));
            _finalValueInitialized = true;
        }
    }

    const T& startValue() const { return _startValue; }

    void setStartValue(const T& value)
    {
        _startValue = value;
        _qVariantAnimation.setStartValue(QVariant::fromValue<T>(value));
        _startValueInitialized = true;
    }

    T value() const
    {
        if (isRunning()) {
            const auto variant = _qVariantAnimation.currentValue();
            return variant.template canConvert<T>()
                       ? variant.template value<T>()
                       : _finalValue;
        }

        return _finalValue;
    }

    void setEasing(const QEasingCurve& easing)
    {
        _qVariantAnimation.setEasingCurve(easing);
    }

protected:
    bool eventFilter(QObject* obj, QEvent* evt) override
    {
        if (evt->type() == QEvent::Hide) {
            stop();
        }
        return QObject::eventFilter(obj, evt);
    }

    bool hasStartValue() const { return _startValueInitialized; }

    bool hasFinalValue() const { return _finalValueInitialized; }

private:
    bool _startValueInitialized{false};
    bool _finalValueInitialized{false};
    bool _loopEnabled{false};
    QVariantAnimation _qVariantAnimation{};
    T _startValue{};
    T _finalValue{};
};

} // namespace thoht
