#include "qimageutils.h"

#include <QIcon>
#include <QImageReader>
#include <QPainter>
#include <QPixmap>
#include <QSvgRenderer>

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

QImage colorizeImage(const QPixmap& src, const QColor& color)
{
    if (src.isNull()) {
        return {};
    }

    // Convert input QPixmap to QImage, because it is better for fast pixel
    // manipulation.
    const auto imageSize = src.size();
    auto inputImage = src.toImage();
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

QIcon makeIconFromSvg(const QString& filename, const QSize& size)
{
    if (filename.isEmpty()) {
        return {};
    }

    QIcon icon;
    QSvgRenderer renderer{filename};
    for (const auto& ratio : {1, 2}) {
        const auto pixmapSize = size * ratio;
        QPixmap pm{pixmapSize};
        pm.fill(Qt::transparent);
        QPainter painter{&pm};
        painter.setRenderHint(QPainter::Antialiasing, true);
        renderer.render(&painter, pm.rect());
        pm.setDevicePixelRatio(static_cast<double>(ratio));
        icon.addPixmap(pm);
    }
    return icon;
}

QPixmap makePixmapFromSvg(const QString& filename, const QSize& size)
{
    if (filename.isEmpty()) {
        return {};
    }

    QPixmap pm{size};
    pm.fill(Qt::transparent);
    QPainter painter{&pm};
    painter.setRenderHint(QPainter::Antialiasing, true);
    QSvgRenderer renderer(filename);
    renderer.render(&painter, pm.rect());
    return pm;
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
    return size.width() / static_cast<double>(size.height());
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
