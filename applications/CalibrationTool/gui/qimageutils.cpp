#include "qimageutils.h"

#include <QPainter>

namespace tl {
namespace qimg {

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
