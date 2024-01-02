#pragma once

#include <QBrush>
#include <QColor>

namespace thoht {

class ColorSettings
{
public:
    enum ColorMode
    {
        cmFilledBackground = 1,
        cmNoBackground = 2
    };
    ColorSettings(const QColor &lightColor = Qt::white,
                  const QColor &darkColor = Qt::black);

    void setColor(const QColor &color);
    QColor color() const;

    void setMode(ColorMode mode);
    ColorMode mode() const;

    QColor foregroundColor() const;
    QBrush backgroundBrush() const;

    void setLightColor(const QColor &color);
    QColor lightColor() const;

    void setDarkColor(const QColor &color);
    QColor darkColor() const;

    void setModified(bool modified);
    bool modified() const;

protected:
    QColor mLightColor;
    QColor mDarkColor;

    QColor mColor;
    QColor mForegroundColor;
    QBrush mBackgroundBrush;

    ColorMode mMode;

    void updateColors();
    bool mModified;
};

} // namespace thoht
