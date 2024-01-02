#pragma once

#include <QStyle>
#include <QStyleOption>

#include "TheStyleTypes.h"

namespace thoht {

MouseState getMouseState(QStyle::State state);
MouseState getMouseState(bool pressed, bool hovered, bool enabled);
MouseState getToolButtonMouseState(QStyle::State state);
MouseState getMenuItemMouseState(QStyle::State state);
MouseState getTabItemMouseState(QStyle::State state, bool tabIsHovered);
ColorRole getColorRole(QStyle::State state, bool isDefault);
ColorRole getColorRole(bool checked, bool isDefault);
ColorRole getColorRole(Qt::CheckState checked);
MouseState getSliderHandleState(QStyle::State state,
                                QStyle::SubControls activeSubControls);
MouseState getScrollBarHandleState(QStyle::State state,
                                   QStyle::SubControls activeSubControls);
FocusState getFocusState(QStyle::State state);
FocusState getFocusState(bool focused);
Qt::CheckState getCheckState(QStyle::State state);
Qt::CheckState getCheckState(bool checked);
ActiveState getActiveState(QStyle::State state);
SelectionState getSelectionState(QStyle::State state);
AlternateState getAlternateState(QStyleOptionViewItem::ViewItemFeatures state);
QStyle::State getState(bool enabled, bool hover, bool pressed);
QIcon::Mode getIconMode(MouseState mouse);
QIcon::State getIconState(Qt::CheckState checked);
QPalette::ColorGroup getPaletteColorGroup(QStyle::State state);
QPalette::ColorGroup getPaletteColorGroup(MouseState mouse);

QString printState(QStyle::State state);

} // namespace thoht
