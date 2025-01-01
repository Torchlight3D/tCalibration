#pragma once

#include <opencv2/core/mat.hpp>

#include <QImage>

namespace tl {

// This is a simple implementation
QImage cvMatToQImage(const cv::Mat &src, bool deepCopy = false);
inline auto cvMatCopyToQImage(const cv::Mat &src)
{
    return cvMatToQImage(src, true);
}

enum MatColorOrder
{
    Unknown,

    BGR,
    RGB,
    BGRA = BGR,
    RGBA = RGB,
    ARGB
};

// Convert QImage to/from cv::Mat
//
// - cv::Mat
//   - Supported channels
//     - 1 channel （Grayscale)
//     - 3 channels (B G R), (R G B)
//     - 4 channels (B G R A), (R G B A), (A R G B)
//   - Supported depth
//     - CV_8U  [0, 255]
//     - CV_16U [0, 65535]
//     - CV_32F [0, 1.0]
//
// - QImage
//   - All of the formats of QImage are supported.
//
cv::Mat image2Mat(const QImage &img, int requiredMatType = CV_8UC(0),
                  MatColorOrder requiredOrder = BGR);
QImage mat2Image(const cv::Mat &mat, MatColorOrder order = BGR,
                 QImage::Format formatHint = QImage::Format_Invalid);

// Convert QImage to/from cv::Mat without data copy
//
// - Supported QImage formats and cv::Mat types are:
//   - QImage::Format_Indexed8               <==> CV_8UC1
//   - QImage::Format_Alpha8                 <==> CV_8UC1
//   - QImage::Format_Grayscale8             <==> CV_8UC1
//   - QImage::Format_RGB888                 <==> CV_8UC3 (RGB)
//   - QImage::Format_RGB32                  <==> CV_8UC4 (ARGB/BGRA)
//   - QImage::Format_ARGB32                 <==> CV_8UC4 (ARGB/BGRA)
//   - QImage::Format_ARGB32_Premultiplied   <==> CV_8UC4 (ARGB/BGRA)
//   - QImage::Format_RGBX8888               <==> CV_8UC4 (RGBA)
//   - QImage::Format_RGBA8888               <==> CV_8UC4 (RGBA)
//   - QImage::Format_RGBA8888_Premultiplied <==> CV_8UC4 (RGBA)
//
// - For QImage
//   QImage::Format_RGB32
//   QImage::Format_ARGB32
//   QImage::Format_ARGB32_Premultiplied
//   The color channel order of cv::Mat will be BGRA in little
//   endian system or ARGB in big endian system.
//
// - User must make sure that the color channels order is the same as
//   the color channels order requried by QImage.
//
cv::Mat image2Mat_shared(const QImage &img, MatColorOrder &order);
QImage mat2Image_shared(const cv::Mat &mat,
                        QImage::Format formatHint = QImage::Format_Invalid);

} // namespace tl
