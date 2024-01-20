#include "QtOpenCV.h"

#include <opencv2/imgproc.hpp>

namespace tl {

QImage cvMatToQImage(const cv::Mat &src, bool deepCopy)
{
    QImage dst;
    switch (src.type()) {
        case CV_8UC1: // Gray image
            dst = {src.data, src.cols, src.rows, static_cast<int>(src.step),
                   QImage::Format_Indexed8};
            dst.setColorCount(256);
            for (int i = 0; i < 256; i++) {
                dst.setColor(i, qRgb(i, i, i));
            }
            break;
        case CV_8UC3: // BRG image
            dst = {src.data, src.cols, src.rows, static_cast<int>(src.step),
                   QImage::Format_BGR888};
            break;
        case CV_8UC4: // BGRA image
            dst = {src.data, src.cols, src.rows, static_cast<int>(src.step),
                   QImage::Format_ARGB32};
            break;
        default:
            break;
    }

    if (deepCopy) {
        return dst.copy();
    }

    return dst;
}

namespace {

cv::Mat argb2bgra(const cv::Mat &src)
{
    CV_Assert(src.channels() == 4);

    cv::Mat dst{src.rows, src.cols, src.type()};
    constexpr int fromTo[]{0, 3, 1, 2, 2, 1, 3, 0};
    cv::mixChannels(&src, 1, &dst, 1, fromTo, 4);
    return dst;
}

cv::Mat argb2rgba(const cv::Mat &src)
{
    CV_Assert(src.channels() == 4);

    cv::Mat dst{src.rows, src.cols, src.type()};
    constexpr int fromTo[]{0, 3, 1, 0, 2, 1, 3, 2};
    cv::mixChannels(&src, 1, &dst, 1, fromTo, 4);
    return dst;
}

cv::Mat rgba2argb(const cv::Mat &src)
{
    CV_Assert(src.channels() == 4);

    cv::Mat dst{src.rows, src.cols, src.type()};
    constexpr int fromTo[]{0, 1, 1, 2, 2, 3, 3, 0};
    cv::mixChannels(&src, 1, &dst, 1, fromTo, 4);
    return dst;
}

cv::Mat adjustChannelsOrder(const cv::Mat &src, MatColorOrder srcOrder,
                            MatColorOrder dstOrder)
{
    CV_Assert(src.channels() == 4);

    if (srcOrder == dstOrder) {
        return src.clone();
    }

    cv::Mat dst;
    if ((srcOrder == ARGB && dstOrder == BGRA) ||
        (srcOrder == BGRA && dstOrder == ARGB)) {
        dst = argb2bgra(src);
    }
    else if (srcOrder == ARGB && dstOrder == RGBA) {
        dst = argb2rgba(src);
    }
    else if (srcOrder == RGBA && dstOrder == ARGB) {
        dst = rgba2argb(src);
    }
    else {
        cv::cvtColor(src, dst, cv::COLOR_BGRA2RGBA);
    }
    return dst;
}

QImage::Format findClosestFormat(QImage::Format formatHint)
{
    QImage::Format format;
    switch (formatHint) {
        case QImage::Format_Indexed8:
        case QImage::Format_RGB32:
        case QImage::Format_ARGB32:
        case QImage::Format_ARGB32_Premultiplied:
        case QImage::Format_RGB888:
        case QImage::Format_RGBX8888:
        case QImage::Format_RGBA8888:
        case QImage::Format_RGBA8888_Premultiplied:
        case QImage::Format_Alpha8:
        case QImage::Format_Grayscale8:
            format = formatHint;
            break;
        case QImage::Format_Mono:
        case QImage::Format_MonoLSB:
            format = QImage::Format_Indexed8;
            break;
        case QImage::Format_RGB16:
            format = QImage::Format_RGB32;
            break;
        case QImage::Format_RGB444:
        case QImage::Format_RGB555:
        case QImage::Format_RGB666:
            format = QImage::Format_RGB888;
            break;
        case QImage::Format_ARGB4444_Premultiplied:
        case QImage::Format_ARGB6666_Premultiplied:
        case QImage::Format_ARGB8555_Premultiplied:
        case QImage::Format_ARGB8565_Premultiplied:
            format = QImage::Format_ARGB32_Premultiplied;
            break;
        default:
            format = QImage::Format_ARGB32;
            break;
    }
    return format;
}

constexpr MatColorOrder getColorOrderOfRGB32Format()
{
#if Q_BYTE_ORDER == Q_LITTLE_ENDIAN
    return BGRA;
#else
    return MCO_ARGB;
#endif
}

} // namespace

cv::Mat image2Mat(const QImage &img, int requiredMatType,
                  MatColorOrder requriedOrder)
{
    int targetDepth = CV_MAT_DEPTH(requiredMatType);
    int targetChannels = CV_MAT_CN(requiredMatType);
    CV_Assert(targetChannels == CV_CN_MAX || targetChannels == 1 ||
              targetChannels == 3 || targetChannels == 4);
    CV_Assert(targetDepth == CV_8U || targetDepth == CV_16U ||
              targetDepth == CV_32F);

    if (img.isNull()) {
        return {};
    }

    // Find the closest image format that can be used in image2Mat_shared()
    QImage::Format format = findClosestFormat(img.format());
    QImage image = (format == img.format()) ? img : img.convertToFormat(format);

    MatColorOrder srcOrder{MatColorOrder::Unknown};
    cv::Mat mat0 = image2Mat_shared(image, srcOrder);

    // Adjust mat channells if needed.
    cv::Mat mat_adjustCn;
    const float maxAlpha =
        targetDepth == CV_8U ? 255 : (targetDepth == CV_16U ? 65535 : 1.0);
    if (targetChannels == CV_CN_MAX) {
        targetChannels = mat0.channels();
    }

    switch (targetChannels) {
        case 1:
            if (mat0.channels() == 3) {
                cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_RGB2GRAY);
            }
            else if (mat0.channels() == 4) {
                if (srcOrder == BGRA) {
                    cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_BGRA2GRAY);
                }
                else if (srcOrder == RGBA) {
                    cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_RGBA2GRAY);
                }
                else {
                    cv::cvtColor(argb2bgra(mat0), mat_adjustCn,
                                 cv::COLOR_BGRA2GRAY);
                }
            }
            break;
        case 3:
            if (mat0.channels() == 1) {
                cv::cvtColor(mat0, mat_adjustCn,
                             requriedOrder == BGR ? cv::COLOR_GRAY2BGR
                                                      : cv::COLOR_GRAY2RGB);
            }
            else if (mat0.channels() == 3) {
                if (requriedOrder != srcOrder)
                    cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_RGB2BGR);
            }
            else if (mat0.channels() == 4) {
                if (srcOrder == ARGB) {
                    mat_adjustCn = cv::Mat(mat0.rows, mat0.cols,
                                           CV_MAKE_TYPE(mat0.type(), 3));
                    int ARGB2RGB[] = {1, 0, 2, 1, 3, 2};
                    int ARGB2BGR[] = {1, 2, 2, 1, 3, 0};
                    cv::mixChannels(
                        &mat0, 1, &mat_adjustCn, 1,
                                requriedOrder == BGR ? ARGB2BGR : ARGB2RGB, 3);
                }
                else if (srcOrder == BGRA) {
                    cv::cvtColor(mat0, mat_adjustCn,
                                 requriedOrder == BGR ? cv::COLOR_BGRA2BGR
                                                          : cv::COLOR_BGRA2RGB);
                }
                else { // RGBA
                    cv::cvtColor(mat0, mat_adjustCn,
                                 requriedOrder == BGR ? cv::COLOR_RGBA2BGR
                                                          : cv::COLOR_RGBA2RGB);
                }
            }
            break;
        case 4:
            if (mat0.channels() == 1) {
                if (requriedOrder == ARGB) {
                    cv::Mat alphaMat(mat0.rows, mat0.cols,
                                     CV_MAKE_TYPE(mat0.type(), 1),
                                     cv::Scalar(maxAlpha));
                    mat_adjustCn = cv::Mat(mat0.rows, mat0.cols,
                                           CV_MAKE_TYPE(mat0.type(), 4));
                    cv::Mat in[] = {alphaMat, mat0};
                    int from_to[] = {0, 0, 1, 1, 1, 2, 1, 3};
                    cv::mixChannels(in, 2, &mat_adjustCn, 1, from_to, 4);
                }
                else if (requriedOrder == RGBA) {
                    cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_GRAY2RGBA);
                }
                else { // MCO_BGRA
                    cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_GRAY2BGRA);
                }
            }
            else if (mat0.channels() == 3) {
                if (requriedOrder == ARGB) {
                    cv::Mat alphaMat(mat0.rows, mat0.cols,
                                     CV_MAKE_TYPE(mat0.type(), 1),
                                     cv::Scalar(maxAlpha));
                    mat_adjustCn = cv::Mat(mat0.rows, mat0.cols,
                                           CV_MAKE_TYPE(mat0.type(), 4));
                    cv::Mat in[] = {alphaMat, mat0};
                    int from_to[] = {0, 0, 1, 1, 2, 2, 3, 3};
                    cv::mixChannels(in, 2, &mat_adjustCn, 1, from_to, 4);
                }
                else if (requriedOrder == RGBA) {
                    cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_RGB2RGBA);
                }
                else { // MCO_BGRA
                    cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_RGB2BGRA);
                }
            }
            else if (mat0.channels() == 4) {
                if (srcOrder != requriedOrder)
                    mat_adjustCn =
                        adjustChannelsOrder(mat0, srcOrder, requriedOrder);
            }
            break;
        default:
            break;
    }

    // Adjust depth if needed.
    if (targetDepth == CV_8U) {
        return mat_adjustCn.empty() ? mat0.clone() : mat_adjustCn;
    }

    if (mat_adjustCn.empty()) {
        mat_adjustCn = mat0;
    }

    cv::Mat mat_adjustDepth;
    mat_adjustCn.convertTo(mat_adjustDepth,
                           CV_MAKE_TYPE(targetDepth, mat_adjustCn.channels()),
                           targetDepth == CV_16U ? 255.0 : 1 / 255.0);
    return mat_adjustDepth;
}

QImage mat2Image(const cv::Mat &mat, MatColorOrder order,
                 QImage::Format formatHint)
{
    CV_Assert(mat.channels() == 1 || mat.channels() == 3 ||
              mat.channels() == 4);
    CV_Assert(mat.depth() == CV_8U || mat.depth() == CV_16U ||
              mat.depth() == CV_32F);

    if (mat.empty()) {
        return {};
    }

    // Adjust mat channels if needed, and find proper QImage format.
    QImage::Format format{QImage::Format_Invalid};
    cv::Mat mat_adjustCn;
    if (mat.channels() == 1) {
        format = formatHint;
        if (formatHint != QImage::Format_Indexed8 &&
            formatHint != QImage::Format_Alpha8 &&
            formatHint != QImage::Format_Grayscale8) {
            format = QImage::Format_Indexed8;
        }
    }
    else if (mat.channels() == 3) {
        format = QImage::Format_RGB888;
        if (order == BGR)
            cv::cvtColor(mat, mat_adjustCn, cv::COLOR_BGR2RGB);
    }
    else if (mat.channels() == 4) {
        // Find best format if the formatHint can not be applied.
        format = findClosestFormat(formatHint);
        if (format != QImage::Format_RGB32 && format != QImage::Format_ARGB32 &&
            format != QImage::Format_ARGB32_Premultiplied &&
            format != QImage::Format_RGBX8888 &&
            format != QImage::Format_RGBA8888 &&
            format != QImage::Format_RGBA8888_Premultiplied) {
            format = order == RGBA ? QImage::Format_RGBA8888
                                       : QImage::Format_ARGB32;
        }

        // Channel order requried by the target QImage
        MatColorOrder requiredOrder = getColorOrderOfRGB32Format();
        if (formatHint == QImage::Format_RGBX8888 ||
            formatHint == QImage::Format_RGBA8888 ||
            formatHint == QImage::Format_RGBA8888_Premultiplied) {
            requiredOrder = RGBA;
        }

        if (order != requiredOrder) {
            mat_adjustCn = adjustChannelsOrder(mat, order, requiredOrder);
        }
    }

    if (mat_adjustCn.empty()) {
        mat_adjustCn = mat;
    }

    // Adjust mat depth if needed.
    cv::Mat mat_adjustDepth = mat_adjustCn;
    if (mat.depth() != CV_8U) {
        mat_adjustCn.convertTo(mat_adjustDepth, CV_8UC(mat_adjustCn.channels()),
                               mat.depth() == CV_16U ? 1 / 255.0 : 255.0);
    }

    // Should we convert the image to the format specified by formatHint?
    QImage image = mat2Image_shared(mat_adjustDepth, format);
    if (format == formatHint || formatHint == QImage::Format_Invalid) {
        return image.copy();
    }

    return image.convertToFormat(formatHint);
}

cv::Mat image2Mat_shared(const QImage &img, MatColorOrder &order)
{
    if (img.isNull()) {
        return {};
    }

    switch (img.format()) {
        case QImage::Format_Indexed8:
            break;
        case QImage::Format_RGB888:
            order = RGB;
            break;
        case QImage::Format_RGB32:
        case QImage::Format_ARGB32:
        case QImage::Format_ARGB32_Premultiplied:
            order = getColorOrderOfRGB32Format();
            break;
        case QImage::Format_RGBX8888:
        case QImage::Format_RGBA8888:
        case QImage::Format_RGBA8888_Premultiplied:
            order = RGBA;
            break;
        case QImage::Format_Alpha8:
        case QImage::Format_Grayscale8:
            break;
        default:
            return {};
    }

    return {img.height(), img.width(), CV_8UC(img.depth() / 8),
            (uchar *)img.bits(), static_cast<size_t>(img.bytesPerLine())};
}

QImage mat2Image_shared(const cv::Mat &mat, QImage::Format formatHint)
{
    CV_Assert(mat.type() == CV_8UC1 || mat.type() == CV_8UC3 ||
              mat.type() == CV_8UC4);

    if (mat.empty()) {
        return {};
    }

    // Adjust formatHint if needed.
    switch (mat.type()) {
        case CV_8UC1: {
            if (formatHint != QImage::Format_Indexed8 &&
                formatHint != QImage::Format_Alpha8 &&
                formatHint != QImage::Format_Grayscale8) {
                formatHint = QImage::Format_Indexed8;
            }
        } break;
        case CV_8UC3: {
            formatHint = QImage::Format_RGB888;
        } break;
        case CV_8UC4: {
            if (formatHint != QImage::Format_RGB32 &&
                formatHint != QImage::Format_ARGB32 &&
                formatHint != QImage::Format_ARGB32_Premultiplied &&
                formatHint != QImage::Format_RGBX8888 &&
                formatHint != QImage::Format_RGBA8888 &&
                formatHint != QImage::Format_RGBA8888_Premultiplied) {
                formatHint = QImage::Format_ARGB32;
            }
        } break;
        default:
            break;
    }

    QImage img{mat.data, mat.cols, mat.rows, static_cast<qsizetype>(mat.step),
               formatHint};

    if (formatHint == QImage::Format_Indexed8) {
        QList<QRgb> colorTable;
        for (int i = 0; i < 256; ++i) {
            colorTable.append(qRgb(i, i, i));
        }
        img.setColorTable(colorTable);
    }

    return img;
}

} // namespace tl
