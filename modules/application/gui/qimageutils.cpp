#include "qimageutils.h"

#include <array>

#include <QIcon>
#include <QImageReader>
#include <QLatin1Char>
#include <QLatin1String>
#include <QPainter>
#include <QPixmap>
#include <QPixmapCache>
#include <QStringBuilder>
#include <QSvgRenderer>

#include "blurutils.h"
#include "primitiveutils.h"
#include "qstringutils.h"

namespace qtprivate {
// Taken from qpixmapfilter.cpp, line 946, Qt 5.15.2
// Grayscales the image to dest (could be same). If rect isn't defined
// destination image size is used to determine the dimension of grayscaling
// process.
void grayscale(const QImage& image, QImage& dest, const QRect& rect = {})
{
    QRect destRect = rect;
    QRect srcRect = rect;

    if (rect.isNull()) {
        srcRect = dest.rect();
        destRect = dest.rect();
    }
    if (&image != &dest) {
        destRect.moveTo(QPoint(0, 0));
    }

    const unsigned int* data = (const unsigned int*)image.bits();
    unsigned int* outData = (unsigned int*)dest.bits();

    if (dest.size() == image.size() && image.rect() == srcRect) {
        // A bit faster loop for grayscaling everything.
        int pixels = dest.width() * dest.height();
        for (int i = 0; i < pixels; ++i) {
            int val = qGray(data[i]);
            outData[i] = qRgba(val, val, val, qAlpha(data[i]));
        }
    }
    else {
        int yd = destRect.top();
        for (int y = srcRect.top(); y <= srcRect.bottom() && y < image.height();
             y++) {
            data = (const unsigned int*)image.scanLine(y);
            outData = (unsigned int*)dest.scanLine(yd++);
            int xd = destRect.left();
            for (int x = srcRect.left();
                 x <= srcRect.right() && x < image.width(); x++) {
                int val = qGray(data[x]);
                outData[xd++] = qRgba(val, val, val, qAlpha(data[x]));
            }
        }
    }
}
} // namespace qtprivate

namespace tl {
namespace qimg {

QImage colorizeImage(const QPixmap& input, const QColor& color)
{
    if (input.isNull()) {
        return {};
    }

    // Convert input QPixmap to QImage, because it is better for fast pixel
    // manipulation.
    const auto imageSize = input.size();
    auto inputImage = input.toImage();
    inputImage = std::move(inputImage).convertToFormat(QImage::Format_ARGB32);

    // Create output QImage with same format and size as input QImage.
    auto outputImage = QImage(imageSize, inputImage.format());
    outputImage.setDevicePixelRatio(inputImage.devicePixelRatioF());
    const auto outputRgb = color.rgba();
    const auto outputR = qRed(outputRgb);
    const auto outputG = qGreen(outputRgb);
    const auto outputB = qBlue(outputRgb);
    const auto outputAf = qAlpha(outputRgb) / 255.;

    // Modify the pixels.
    for (auto x = 0; x < imageSize.width(); ++x) {
        for (auto y = 0; y < imageSize.height(); ++y) {
            const auto inputPixel = inputImage.pixel(x, y);
            const auto inputA = qAlpha(inputPixel);
            const auto outputA = static_cast<int>(inputA * outputAf);
            const auto outputPixel = qRgba(outputR, outputG, outputB, outputA);
            outputImage.setPixel(x, y, outputPixel);
        }
    }

    return outputImage;
}

QPixmap tintPixmap(const QPixmap& input, const QColor& color)
{
    if (input.isNull()) {
        return {};
    }

    // QImage is made for faster pixel manipulation.
    auto inputImage = input.toImage();
    const auto format = inputImage.hasAlphaChannel()
                            ? QImage::Format_ARGB32_Premultiplied
                            : QImage::Format_RGB32;
    inputImage = std::move(inputImage).convertToFormat(format);

    auto outputImage = QImage(inputImage.size(), inputImage.format());
    outputImage.setDevicePixelRatio(inputImage.devicePixelRatioF());

    // Convert to gray scale, then apply the color over with a Screen
    // composition mode.
    QPainter outputPainter(&outputImage);
    qtprivate::grayscale(inputImage, outputImage, inputImage.rect());
    outputPainter.setCompositionMode(QPainter::CompositionMode_Screen);
    outputPainter.fillRect(inputImage.rect(), color);
    outputPainter.end();

    // Keep the alpha.
    if (inputImage.hasAlphaChannel()) {
        Q_ASSERT(outputImage.format() == QImage::Format_ARGB32_Premultiplied);
        QPainter maskPainter(&outputImage);
        maskPainter.setCompositionMode(QPainter::CompositionMode_DestinationIn);
        maskPainter.drawImage(0, 0, inputImage);
    }

    return QPixmap::fromImage(outputImage);
}

QString getColorizedPixmapKey(const QPixmap& pixmap, const QColor& color)
{
    return u"qlementine_color_"_s + qimg::toHex(pixmap.cacheKey()) %
                                        QLatin1Char('_') %
                                        qimg::toHex(color.rgba());
}

QString getTintedPixmapKey(const QPixmap& pixmap, const QColor& color)
{
    return u"qlementine_tint_"_s + qimg::toHex(pixmap.cacheKey()) %
                                       QLatin1Char('_') %
                                       qimg::toHex(color.rgba());
}

QPixmap getCachedPixmap(const QPixmap& input, const QColor& color,
                        qimg::ColorizeMode mode)
{
    if (input.isNull()) {
        return input;
    }

    const auto tint = mode == qimg::ColorizeMode::Tint;
    // Look if pixmap already exists in cache.
    const auto& pixmapKey = tint ? getTintedPixmapKey(input, color)
                                 : getColorizedPixmapKey(input, color);
    QPixmap pixmapInCache;
    const auto found = QPixmapCache::find(pixmapKey, &pixmapInCache);

    // Add colorized pixmap to cache, if not present.
    if (!found) {
        const auto& newPixmap = tint ? tintPixmap(input, color)
                                     : qimg::colorizePixmap(input, color);
        const auto successfulInsert =
            QPixmapCache::insert(pixmapKey, newPixmap);
        if (successfulInsert) {
            QPixmapCache::find(pixmapKey, &pixmapInCache);
        }
    }

    // Returns the new pixmap, or the input if any error.
    return pixmapInCache.isNull() ? input : pixmapInCache;
}

QIcon makeIconFromSvg(const QString& svgPath, const QSize& size)
{
    if (svgPath.isEmpty()) {
        return {};
    }

    QIcon icon;
    QSvgRenderer renderer(svgPath);
    constexpr auto ratios = std::array<int, 2>{1, 2};
    for (const auto& ratio : ratios) {
        const auto pixmapSize = size * ratio;
        QPixmap pixmap(pixmapSize);
        pixmap.fill(Qt::transparent);
        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);
        renderer.render(&painter, pixmap.rect());
        pixmap.setDevicePixelRatio(static_cast<double>(ratio));
        icon.addPixmap(pixmap);
    }
    return icon;
}

QPixmap makePixmapFromSvg(const QString& svgPath, const QSize& size)
{
    if (svgPath.isEmpty()) {
        return {};
    }

    QSvgRenderer renderer(svgPath);
    QPixmap pixmap(size);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    renderer.render(&painter, pixmap.rect());
    return pixmap;
}

QPixmap makePixmapFromSvg(const QString& backgroundSvgPath,
                          const QColor& backgroundColor,
                          const QString& foregroundSvgPath,
                          const QColor& foregroundColor, const QSize& size)
{
    const auto bgPixmap = makePixmapFromSvg(backgroundSvgPath, size);
    const auto fgPixmap = makePixmapFromSvg(foregroundSvgPath, size);
    const auto coloredBgPixmap =
        qimg::colorizePixmap(bgPixmap, backgroundColor);
    const auto coloredFgPixmap =
        qimg::colorizePixmap(fgPixmap, foregroundColor);

    QPixmap pixmap(size);
    pixmap.fill(Qt::transparent);

    QPainter p(&pixmap);
    p.drawPixmap(0, 0, coloredBgPixmap);
    p.drawPixmap(0, 0, coloredFgPixmap);

    return pixmap;
}

QPixmap makeRoundedPixmap(const QPixmap& input, double topLeft, double topRight,
                          double bottomRight, double bottomLeft)
{
    if (input.isNull()) {
        return {};
    }

    QPixmap result(input.size());
    result.fill(Qt::transparent);

    QPainter p(&result);

    // Mask.
    p.setRenderHint(QPainter::Antialiasing, true);
    drawRoundedRect(&p, result.rect(), Qt::white,
                    {topLeft, topRight, bottomRight, bottomLeft});
    p.setCompositionMode(QPainter::CompositionMode_SourceIn);
    // Draw input pixmap over.
    p.drawPixmap(result.rect(), input);

    result.setDevicePixelRatio(input.devicePixelRatio());
    return result;
}

QPixmap makeFitPixmap(const QPixmap& input, const QSize& size)
{
    if (input.isNull()) {
        return {};
    }

    QPixmap result(size);
    result.fill(Qt::transparent);

    QPainter p(&result);
    const auto scaledInput =
        input.scaled(size, Qt::AspectRatioMode::KeepAspectRatioByExpanding,
                     Qt::SmoothTransformation);
    const auto x = (result.width() - scaledInput.width()) / 2.;
    const auto y = (result.height() - scaledInput.height()) / 2.;

    p.setRenderHint(QPainter::Antialiasing, true);
    p.drawPixmap(int(x), int(y), scaledInput);

    return result;
}

double getImageAspectRatio(const QString& path)
{
    if (path.isEmpty()) {
        return 1.;
    }

    const QImageReader reader(path);
    const auto size = reader.size();
    const auto aspectRatio = size.width() / static_cast<double>(size.height());
    return aspectRatio;
}

QImage getExtendedImage(const QPixmap& input, int padding)
{
    if (input.isNull()) {
        return {};
    }

    const QSize extendedSize{input.width() + padding, input.height() + padding};
    QImage inputImage(extendedSize, QImage::Format_ARGB32_Premultiplied);
    inputImage.fill(Qt::transparent);
    {
        QPainter p(&inputImage);
        p.setRenderHint(QPainter::Antialiasing, true);
        const auto x = (extendedSize.width() - input.width()) / 2;
        const auto y = (extendedSize.height() - input.height()) / 2;
        p.drawPixmap(x, y, input);
    }
    return inputImage;
}

QImage getExtendedImage(const QImage& input, int padding)
{
    if (input.isNull()) {
        return {};
    }

    const QSize extendedSize{input.width() + padding, input.height() + padding};
    QImage inputImage{extendedSize, QImage::Format_ARGB32_Premultiplied};
    inputImage.fill(Qt::transparent);
    {
        QPainter p(&inputImage);
        p.setRenderHint(QPainter::Antialiasing, true);
        const auto x = (extendedSize.width() - input.width()) / 2;
        const auto y = (extendedSize.height() - input.height()) / 2;
        p.drawImage(x, y, input);
    }
    return inputImage;
}

QImage getBlurredImage(const QImage& inputImage, double sigma)
{
    if (inputImage.isNull()) {
        return {};
    }

    auto input = inputImage.copy();
    auto output = inputImage.copy();
    auto* inputData = input.bits();
    auto* outputData = output.bits();
    constexpr auto channelCount = 4; // ARGB
    fast_gaussian_blur(inputData, outputData, input.width(), input.height(),
                       channelCount, sigma);
    // Since fast_gaussian_blur does an unnecessary std::swap, the actual result
    // is in input.
    return input;
}

QPixmap getBlurredPixmap(const QPixmap& input, double blurRadius, bool extend)
{
    if (input.isNull())
        return {};

    blurRadius /= pixelToSigma;
    if (blurRadius < 1) {
        return input;
    }

    const auto padding = static_cast<int>(std::ceil(blurRadius * 4));
    auto inputImage =
        extend ? getExtendedImage(input, padding) : input.toImage();
    const auto outputImage = getBlurredImage(inputImage, blurRadius);
    return QPixmap::fromImage(outputImage);
}

QPixmap getDropShadowPixmap(const QPixmap& src, double blurRadius,
                            const QColor& color)
{
    if (src.isNull()) {
        return {};
    }

    if (blurRadius <= 0.) {
        QPixmap result{src.size()};
        result.fill(Qt::transparent);
        return result;
    }

    const auto colorizedImage = colorizeImage(src, color);
    const auto padding = static_cast<int>(std::ceil(blurRadius * 4));
    const auto extendedImage = getExtendedImage(colorizedImage, padding);
    const auto shadowImage = getBlurredImage(extendedImage, blurRadius);
    return QPixmap::fromImage(shadowImage);
}

QPixmap getDropShadowPixmap(const QSize& size, double borderRadius,
                            double blurRadius, const QColor& color)
{
    if (size.isEmpty()) {
        return {};
    }

    QPixmap input{size};
    {
        QPainter p(&input);
        drawRoundedRect(&p, QRect{QPoint{}, size}, color, borderRadius);
    }
    return getDropShadowPixmap(input, blurRadius, color);
}

QImage mergeImages(const QImage& src1, const QImage& src2,
                   Qt::Orientation orientation)
{
    if (src1.isNull() || src2.isNull()) {
        return {};
    }

    if ((orientation == Qt::Horizontal && src1.height() != src2.height()) ||
        (orientation == Qt::Vertical && src1.width() != src2.width())) {
        return {};
    }

    switch (orientation) {
        case Qt::Horizontal: {
            QImage merge{src1.width() + src2.width(), src1.height(),
                         QImage::Format_RGB888};
            QPainter p{&merge};
            p.drawImage(0, 0, src1);
            p.drawImage(src1.width(), 0, src2);
            return merge;
        } break;
        case Qt::Vertical: {
            QImage merge{src1.width(), src1.height() + src2.height(),
                         QImage::Format_RGB888};
            QPainter p{&merge};
            p.drawImage(0, 0, src1);
            p.drawImage(0, src1.height(), src2);
            return merge;
        } break;
        default:
            break;
    }

    return {};
}

} // namespace qimg
} // namespace tl
