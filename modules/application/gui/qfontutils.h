#pragma once

class QFontMetrics;
class QString;

namespace tl {

inline static constexpr auto kDefaultDpi = 72.;

constexpr double pointSizeToPixelSize(double pointSize, double dpi)
{
    return pointSize / kDefaultDpi * dpi;
}

constexpr double pixelSizeToPointSize(double pixelSize, double dpi)
{
    return pixelSize * kDefaultDpi / dpi;
}

int textWidth(const QFontMetrics& fm, const QString& text);

} // namespace tl
