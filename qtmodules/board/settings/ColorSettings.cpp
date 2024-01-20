#include "ColorSettings.h"

#include <utility>

namespace tl {

namespace {

double luminance(const QColor &color)
{
    double sR = color.redF();
    double sG = color.greenF();
    double sB = color.blueF();

    auto sRGBToLinear = [](double v) -> double {
        return (v <= 0.03928) ? v / 12.92 : qPow((v + 0.055) / 1.055, 2.4);
    };

    double r = sRGBToLinear(sR);
    double g = sRGBToLinear(sG);
    double b = sRGBToLinear(sB);

    return 0.2126 * r + 0.7152 * g + 0.0722 * b;
}

double contrast(const QColor &color1, const QColor &color2)
{
    double lum1 = luminance(color1);
    double lum2 = luminance(color2);
    if (lum1 > lum2) {
        return (lum1 + 0.05) / (lum2 + 0.05);
    }

    return (lum2 + 0.05) / (lum1 + 0.05);
}

} // namespace

ColorSettings::ColorSettings(const QColor &lightColor, const QColor &darkColor)
    : mLightColor(lightColor),
      mDarkColor(darkColor),
      mColor(mLightColor),
      mMode(cmFilledBackground),
      mModified(false)
{
    updateColors();
}

void ColorSettings::setColor(const QColor &color)
{
    if (color != mColor) {
        mColor = color;
        updateColors();
        mModified = true;
    }
}

QColor ColorSettings::color() const { return mColor; }

void ColorSettings::setMode(ColorMode mode)
{
    if (mode != mMode) {
        mMode = mode;
        updateColors();
        mModified = true;
    }
}

ColorSettings::ColorMode ColorSettings::mode() const { return mMode; }

QColor ColorSettings::foregroundColor() const { return mForegroundColor; }

QBrush ColorSettings::backgroundBrush() const { return mBackgroundBrush; }

QColor ColorSettings::lightColor() const { return mLightColor; }

QColor ColorSettings::darkColor() const { return mDarkColor; }

void ColorSettings::updateColors()
{
    switch (mMode) {
        case cmFilledBackground:
            mForegroundColor =
                (contrast(mColor, mLightColor) > contrast(mColor, mDarkColor))
                    ? mLightColor
                    : mDarkColor;
            mBackgroundBrush = QBrush(mColor);
            break;
        case cmNoBackground:
            mForegroundColor = mColor;
            mBackgroundBrush = Qt::NoBrush;
            break;
    }
}

void ColorSettings::setModified(bool modified) { mModified = modified; }

bool ColorSettings::modified() const { return mModified; }

void ColorSettings::setLightColor(const QColor &color) { mLightColor = color; }

void ColorSettings::setDarkColor(const QColor &color) { mDarkColor = color; }

} // namespace tl
