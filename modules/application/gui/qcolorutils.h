#pragma once

#include <optional>

#include <QColor>

namespace tl {

/// Gets the color the new alpha chanel (from 0 to 255).
QColor colorWithAlpha(QColor const &color, int alpha);

/// Gets the resulting color by applying the foreground color over the
/// background color with 'SourceOver' composition mode.
QColor getColorSourceOver(const QColor &bg, const QColor &fg);

// Attempts to parse the QVariant array to get a QColor from the array's
// values. The order is R, G, B, A.
// TODO: Why not QColor to/from QVaraint directly
std::optional<QColor> tryGetColorFromVariantList(const QVariantList &variants);

// Attempts to parse the QVariant to get a QColor.
std::optional<QColor> tryGetColorFromVariant(const QVariant &variant);

/// Gives the color's hexadecimal RGB string.
QString toHexRGB(const QColor &color);

/// Gives the color's hexadecimal RGBA string.
QString toHexRGBA(const QColor &color);

} // namespace tl
