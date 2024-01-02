#include "stylestateutils.h"

#include <QMetaEnum>

namespace thoht {

MouseState getMouseState(QStyle::State state)
{
    if (!state.testFlag(QStyle::State_Enabled)) {
        return MouseState::Disabled;
    }
    if (state.testFlag(QStyle::State_Sunken)) {
        return MouseState::Pressed;
    }
    if (state.testFlag(QStyle::State_MouseOver)) {
        return MouseState::Hovered;
    }

    return MouseState::Normal;
}

MouseState getMouseState(bool pressed, bool hovered, bool enabled)
{
    if (!enabled) {
        return MouseState::Disabled;
    }
    if (pressed) {
        return MouseState::Pressed;
    }
    if (hovered) {
        return MouseState::Hovered;
    }

    return MouseState::Normal;
}

MouseState getToolButtonMouseState(QStyle::State state)
{
    if (!state.testFlag(QStyle::State_Enabled)) {
        return MouseState::Disabled;
    }
    if (state.testFlag(QStyle::State_Sunken)) {
        return MouseState::Pressed;
    }
    if (state.testFlag(QStyle::State_MouseOver) ||
        state.testFlag(QStyle::State_Selected)) {
        return MouseState::Hovered;
    }

    return MouseState::Transparent;
}

MouseState getMenuItemMouseState(QStyle::State state)
{
    if (!state.testFlag(QStyle::State_Enabled)) {
        return MouseState::Disabled;
    }
    if (state.testFlag(QStyle::State_Sunken)) {
        return MouseState::Pressed;
    }
    if (state.testFlag(QStyle::State_MouseOver) ||
        state.testFlag(QStyle::State_Selected)) {
        return MouseState::Hovered;
    }

    return MouseState::Transparent;
}

MouseState getTabItemMouseState(QStyle::State state, bool tabIsHovered)
{
    const auto selected = state.testFlag(QStyle::State_Selected);
    if (selected || tabIsHovered) {
        if (!state.testFlag(QStyle::State_Enabled)) {
            return MouseState::Disabled;
        }
        if (state.testFlag(QStyle::State_Sunken)) {
            return MouseState::Pressed;
        }
        if (state.testFlag(QStyle::State_MouseOver)) {
            return MouseState::Hovered;
        }

        return MouseState::Normal;
    }

    return state.testFlag(QStyle::State_Enabled) ? MouseState::Transparent
                                                 : MouseState::Disabled;
}

ColorRole getColorRole(QStyle::State state, bool isDefault)
{
    return getColorRole(state.testFlag(QStyle::State_On), isDefault);
}

ColorRole getColorRole(bool checked, bool isDefault)
{
    return checked || isDefault ? ColorRole::Primary : ColorRole::Secondary;
}

ColorRole getColorRole(Qt::CheckState checked)
{
    return getColorRole(checked == Qt::Checked, false);
}

MouseState getSliderHandleState(QStyle::State state,
                                QStyle::SubControls activeSubControls)
{
    const auto handleActive =
        activeSubControls == QStyle::SC_SliderHandle && state;
    return handleActive ? getMouseState(state) : MouseState::Normal;
}

MouseState getScrollBarHandleState(QStyle::State state,
                                   QStyle::SubControls activeSubControls)
{
    const auto handleActive =
        activeSubControls == QStyle::SC_ScrollBarSlider && state;
    if (handleActive) {
        return getMouseState(state);
    }

    if (!state.testFlag(QStyle::State_Enabled)) {
        return MouseState::Disabled;
    }
    if (state.testFlag(QStyle::State_MouseOver)) {
        return MouseState::Normal;
    }

    return MouseState::Transparent;
}

FocusState getFocusState(QStyle::State state)
{
    return state.testFlag(QStyle::State_HasFocus) ? FocusState::Focused
                                                  : FocusState::NotFocused;
}

FocusState getFocusState(bool focused)
{
    return focused ? FocusState::Focused : FocusState::NotFocused;
}

Qt::CheckState getCheckState(QStyle::State state)
{
    if (state.testFlag(QStyle::State_On)) {
        return Qt::Checked;
    }
    if (state.testFlag(QStyle::State_NoChange)) {
        return Qt::PartiallyChecked;
    }

    return Qt::Unchecked;
}

Qt::CheckState getCheckState(bool checked)
{
    return checked ? Qt::Checked : Qt::Unchecked;
}

ActiveState getActiveState(QStyle::State state)
{
    return state.testFlag(QStyle::State_Active) ? ActiveState::Active
                                                : ActiveState::NotActive;
}

SelectionState getSelectionState(QStyle::State state)
{
    return state.testFlag(QStyle::State_Selected) ? SelectionState::Selected
                                                  : SelectionState::NotSelected;
}

AlternateState getAlternateState(QStyleOptionViewItem::ViewItemFeatures state)
{
    return state.testFlag(QStyleOptionViewItem::Alternate)
               ? AlternateState::Alternate
               : AlternateState::NotAlternate;
}

QStyle::State getState(bool enabled, bool hover, bool pressed)
{
    QStyle::State result;
    result.setFlag(QStyle::State_Enabled, enabled);
    result.setFlag(QStyle::State_Sunken, pressed);
    result.setFlag(QStyle::State_MouseOver, hover);
    return result;
}

QIcon::Mode getIconMode(MouseState mouse)
{
    switch (mouse) {
        case MouseState::Disabled:
            return QIcon::Mode::Disabled;
        case MouseState::Hovered:
        case MouseState::Pressed:
            return QIcon::Mode::Active;
        case MouseState::Normal:
        case MouseState::Transparent:
        default:
            return QIcon::Mode::Normal;
    }
}

QIcon::State getIconState(Qt::CheckState checked)
{
    switch (checked) {
        case Qt::Checked:
            return QIcon::State::On;
        case Qt::Unchecked:
        case Qt::PartiallyChecked:
        default:
            return QIcon::State::Off;
    }
}

QPalette::ColorGroup getPaletteColorGroup(QStyle::State state)
{
    if (!state.testFlag(QStyle::State_Enabled)) {
        return QPalette::ColorGroup::Disabled;
    }

    return QPalette::ColorGroup::Normal;
}

QPalette::ColorGroup getPaletteColorGroup(MouseState mouse)
{
    switch (mouse) {
        case MouseState::Disabled:
            return QPalette::ColorGroup::Disabled;
        case MouseState::Hovered:
            return QPalette::ColorGroup::Current;
        case MouseState::Pressed:
            return QPalette::ColorGroup::Active;
        default:
            return QPalette::ColorGroup::Normal;
    }
}

// TODO:
// 1. Move to enum utils
// 2. Use std::enable_if, std::is_enum_v
template <typename Enum_t>
QByteArray enumToQByteArray(Enum_t value)
{
    return QMetaEnum::fromType<Enum_t>().valueToKey(static_cast<int>(value));
}

QString printState(QStyle::State state)
{
    const auto mouse = getMouseState(state);
    const auto focused = getFocusState(state);
    const auto active = getActiveState(state);
    const auto selected = getSelectionState(state);
    const auto checked = getCheckState(state);

    QString result;
    result += "{ ";
    result += "mouse: " + enumToQByteArray(mouse) + ", ";
    result += "focus: " + enumToQByteArray(focused) + ", ";
    result += "active: " + enumToQByteArray(active) + ", ";
    result += "selected: " + enumToQByteArray(selected) + ", ";
    result += "checked: " + enumToQByteArray(checked);
    result += " }";

    return result;
}
} // namespace thoht
