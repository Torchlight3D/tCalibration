#include "qfontutils.h"

#include <QFontMetrics>

namespace thoht {

int textWidth(const QFontMetrics& fm, const QString& text)
{
    // After some tests, it looks like QFontMetrics::boundingRect() with these
    // parameters gives the correct results, and ensure the text isn't
    // truncated. Qt's documentation recommends
    // QFontMetrics::horizontalAdvance() but this one gives incorrect results
    // (i.e. a value not big enough) most of the time.
    return fm.boundingRect({}, Qt::AlignCenter, text, 0, nullptr).width();
}

} // namespace thoht
