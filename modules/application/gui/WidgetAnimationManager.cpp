#include "WidgetAnimationManager.h"

namespace tl {

WidgetAnimationManager::WidgetAnimationManager() { initializeEasingCurves(); }

bool WidgetAnimationManager::enabled() const { return _animationsEnabled; }

void WidgetAnimationManager::setEnabled(bool enabled)
{
    _animationsEnabled = enabled;
    if (!enabled) {
        stopAll();
    }
}

const WidgetAnimator* WidgetAnimationManager::getAnimator(
    const QWidget* w) const
{
    const auto* res = findWidget(w);
    return res;
}

WidgetAnimator* WidgetAnimationManager::getOrCreateAnimator(const QWidget* w)
{
    auto* animator = findWidget(w);
    if (!animator) {
        animator = new WidgetAnimator(const_cast<QWidget*>(w));
        addWidget(w, animator);
    }
    return animator;
}

void WidgetAnimationManager::addWidget(const QWidget* widget,
                                       WidgetAnimator* widgetAnimator)
{
    if (!findWidget(widget)) {
        _map.insert_or_assign(widget, widgetAnimator);

        QObject::connect(widget, &QObject::destroyed, widget,
                         [this, widget]() { removeWidget(widget); });
    }
}

void WidgetAnimationManager::removeWidget(const QWidget* widget)
{
    auto it = _map.find(widget);
    if (it != _map.end()) {
        auto* widgetAnimator = it->second;
        _map.erase(it);
        widgetAnimator->deleteLater();
    }
}

WidgetAnimator* WidgetAnimationManager::findWidget(const QWidget* widget) const
{
    auto it = _map.find(widget);
    if (it != _map.end()) {
        return it->second;
    }
    return nullptr;
}

void WidgetAnimationManager::forEarch(
    const std::function<void(const QWidget* w, WidgetAnimator* a)>& cb)
{
    if (!cb) {
        return;
    }

    std::for_each(_map.begin(), _map.end(),
                  [&cb](const auto& kvp) { cb(kvp.first, kvp.second); });
}

void WidgetAnimationManager::stopAll()
{
    forEarch([this](const QWidget* w, WidgetAnimator*) {
        // Stop all animations happening on the widget.
        removeWidget(w);
    });
}

void WidgetAnimationManager::initializeEasingCurves()
{
    _focusEasingCurve.setOvershoot(5.);
    _focusEasingCurve.setType(QEasingCurve::OutBack);
    _defaultEasingCurve.setType(QEasingCurve::OutCubic);
    _linearEasingCurve.setType(QEasingCurve::Linear);
}

const QEasingCurve& WidgetAnimationManager::focusEasingCurve() const
{
    return _focusEasingCurve;
}

const QEasingCurve& WidgetAnimationManager::defaultEasingCurve() const
{
    return _defaultEasingCurve;
}

} // namespace tl
