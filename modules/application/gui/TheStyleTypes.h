#pragma once

#include <QtCore/qtmetamacros.h>

QT_BEGIN_NAMESPACE
struct QMetaObject;
QT_END_NAMESPACE

namespace tl {

Q_NAMESPACE

// Color family to highlght or not the widget.
enum class ColorRole
{
    Primary,
    Secondary,
};

// Mouse interaction state.
enum class MouseState
{
    Transparent,
    Normal,
    Hovered,
    Pressed,
    Disabled,
};

// Has the widget keyboard focus or not.
enum class FocusState
{
    NotFocused,
    Focused,
};

// Is the list element the current item.
enum class ActiveState
{
    NotActive,
    Active,
};

// Is the widget selected or not.
enum class SelectionState
{
    NotSelected,
    Selected,
};

// Does the ListView row need to be painted in alternate color or not.
enum class AlternateState
{
    NotAlternate,
    Alternate,
};

// Does the button is the default button.
enum class DefaultState
{
    NotDefault,
    Default,
};

// Feedback status displayed to the user.
enum class Status
{
    Default,

    Info,
    Success,
    Warning,
    Error,
};

// Role given to the text or QLabel.
enum class TextRole : int
{
    Caption = -1,
    Default = 0,
    H1,
    H2,
    H3,
    H4,
    H5,
};

// Color mode.
enum class ColorMode
{
    RGB,
    RGBA,
};

// Declare QMetaEnum
Q_ENUM_NS(ActiveState)
Q_ENUM_NS(FocusState)
Q_ENUM_NS(MouseState)
Q_ENUM_NS(SelectionState)

} // namespace tl

#include <QMetaType>

Q_DECLARE_METATYPE(tl::Status);
Q_DECLARE_METATYPE(tl::ColorMode);
