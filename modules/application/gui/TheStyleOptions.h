#pragma once

#include <QStyleOption>

#include "CornerRadius.h"

namespace tl {

/// Allows to customize the radius of the focus border.
class QStyleOptionFocusRoundedRect : public QStyleOptionFocusRect
{
public:
    CornerRadius radiuses;
    int hMargin{0};
    int vMargin{0};
    QColor borderColor;

    QStyleOptionFocusRoundedRect() = default;

    static QStyleOptionFocusRoundedRect fromBase(const QStyleOption& opt,
                                                 const QRect& rect,
                                                 const CornerRadius& radiuses)
    {
        QStyleOptionFocusRoundedRect newOpt;
        newOpt.QStyleOption::operator=(opt);
        newOpt.radiuses = radiuses;
        newOpt.rect = rect;
        return newOpt;
    }

    QStyleOptionFocusRoundedRect(const QStyleOptionFocusRoundedRect& other)
        : QStyleOptionFocusRect(other)
    {
        *this = other;
    }

    QStyleOptionFocusRoundedRect& operator=(
        const QStyleOptionFocusRoundedRect&) = default;
};

/// Allows to customize the radius of a button.
class QStyleOptionRoundedButton : public QStyleOptionButton
{
public:
    enum StyleOptionType
    {
        Type = SO_CustomBase + 1
    };

    CornerRadius radiuses;

    QStyleOptionRoundedButton()
    {
        type = Type;
        radiuses = 0.;
    }

    QStyleOptionRoundedButton(const QStyleOptionRoundedButton& other)
        : QStyleOptionButton(other)
    {
        type = Type;
        radiuses = other.radiuses;
    }

    QStyleOptionRoundedButton& operator=(const QStyleOptionRoundedButton&) =
        default;
};

/// Adds the ability to transition from one visual position to another.
class QStyleOptionSliderF : public QStyleOptionSlider
{
public:
    static constexpr auto INITIALIZED = 2;
    qreal sliderPositionF{0.};
    int status{0}; // Needed to track that it was created by us.

    QStyleOptionSliderF() = default;

    QStyleOptionSliderF(const QStyleOptionSliderF& other)
        : QStyleOptionSlider(other)
    {
        *this = other;
    }

    QStyleOptionSliderF& operator=(const QStyleOptionSliderF&) = default;
};

/// Adds the ability to have a second line of text in the button.
class QStyleOptionCommandLinkButton : public QStyleOptionButton
{
public:
    QString description;

    using QStyleOptionButton::QStyleOptionButton;
};

} // namespace tl
