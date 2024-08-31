#pragma once

#include <iomanip>
#include <sstream>
#include <type_traits>

#include <QColor>
#include <QPixmap>
#include <QString>

namespace tl {
namespace qimg {

// Converts an integer to its std::string hex form.
template <typename T,
          typename std::enable_if_t<std::is_integral_v<T>, T>* = nullptr>
std::string to_hex(T i, bool prefix = false)
{
    std::stringstream stream;
    if (prefix) {
        stream << "0x";
    }
    stream << std::setfill('0') << std::setw(sizeof(T) * 2) << std::hex << i;
    return stream.str();
}

// Convets an integer to its QString hex form.
template <typename T,
          typename std::enable_if_t<std::is_integral_v<T>, T>* = nullptr>
inline QString toHex(T i, bool prefix = false)
{
    return QString::fromStdString(to_hex(i, prefix));
}

// Colorize the QPixmap and returns a QImage.
QImage colorizeImage(const QPixmap& input, const QColor& color);

// Colorize the QPixmap.
inline QPixmap colorizePixmap(const QPixmap& input, const QColor& color)
{
    return QPixmap::fromImage(colorizeImage(input, color));
}

// Tints the QPixmap, preserving contrast between shades.
QPixmap tintPixmap(const QPixmap& input, const QColor& color);

enum class ColorizeMode
{
    // Replaces all {R,G,B} values with another, thus loosing luminance, but
    // preserve alpha.
    // Useful for flat symbolic icons.
    Colorize,
    // Applies an {R,G,B} tint, but keeps luminance and alpha.
    // Useful for complex icons with multiple colors and shades.
    Tint,
};

/// Makes an icon from the file located at the path in parameter. Fixes the
/// standard Qt behavior.
QIcon makeIconFromSvg(const QString& svgPath, const QSize& size);

/// Makes a QPixmap from the file located at the path in parameter at the
/// desired size.
QPixmap makePixmapFromSvg(const QString& svgPath, const QSize& size);

/// Makes a QPixmap from the file located at the path in parameter at the
/// desired size.
QPixmap makePixmapFromSvg(const QString& backgroundSvgPath,
                          const QColor& backgroundSvgColor,
                          const QString& foregroundSvgPath,
                          const QColor& foregroundSvgColor, const QSize& size);

// Makes a QPixmap with rounded corners.
QPixmap makeRoundedPixmap(const QPixmap& input, double topLeft, double topRight,
                          double bottomRight, double bottomLeft);

inline QPixmap makeRoundedPixmap(const QPixmap& input, double radius)
{
    return makeRoundedPixmap(input, radius, radius, radius, radius);
}

// Makes a pixmap that fits the requested size.
QPixmap makeFitPixmap(const QPixmap& input, const QSize& size);

// Aspect ratio (width/height) of a image without loading it into memory.
double getImageAspectRatio(const QString& path);

// Get a version of the pixmap with padding around.
QImage getExtendedImage(const QPixmap& input, int padding);

// Get a version of the image with padding around.
QImage getExtendedImage(const QImage& input, int padding);

// Get a blurred version of the input pixmap
QPixmap getBlurredPixmap(const QPixmap& input, double blurRadius, bool extend);

// Get a drop shadow for the input pixmap (i.e. a blurred colorized version).
QPixmap getDropShadowPixmap(const QPixmap& input, double blurRadius,
                            const QColor& color = Qt::black);

// Get a drop shadow for a QRect.
QPixmap getDropShadowPixmap(const QSize& size, double borderRadius,
                            double blurRadius, const QColor& color = Qt::black);

// Merge two image
QImage mergeImages(const QImage& src1, const QImage& src2,
                   Qt::Orientation orientation = Qt::Horizontal);

inline auto hconcat(const QImage& src1, const QImage& src2)
{
    return mergeImages(src1, src2, Qt::Horizontal);
}
inline auto vconcat(const QImage& src1, const QImage& src2)
{
    return mergeImages(src1, src2, Qt::Vertical);
}

} // namespace qimg
} // namespace tl
