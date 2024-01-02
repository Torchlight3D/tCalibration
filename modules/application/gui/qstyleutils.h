#pragma once

#include <tuple>

class QWidget;

namespace thoht {

/// Hover events (enter/leave events) are disabled by default on widgets.
/// However, some widgets need them.
bool shouldHaveHoverEvents(const QWidget* w);

/// Mouse tracking events (mouse moved events) are disabled by default on
/// widgets. However, some widgets need them.
bool shouldHaveMouseTracking(const QWidget* w);

/// Should the widget text be displayed in bold.
bool shouldHaveBoldFont(const QWidget* w);

/// Focus border outside the widget.
bool shouldHaveExternalFocusFrame(const QWidget* w);

/// Should the widget be focusable only with Tab.
bool shouldHaveTabFocus(const QWidget* w);

/// Should we prevent the widget to resize vertically.
bool shouldNotBeVerticallyCompressed(const QWidget* w);

/// Horizontal paddings (left, right) are different according to the content of
/// the widget.
std::tuple<int, int> getHPaddings(bool hasIcon, bool hasText, bool hasIndicator,
                                  int padding);

/// Should the widget not receive wheel events when not focused.
bool shouldNotHaveWheelEvents(const QWidget* w);

} // namespace thoht
