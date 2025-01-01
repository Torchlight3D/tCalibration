#pragma once

#include <QImage>

namespace tl {
namespace qimg {

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
