#include "qstringutils.h"

#include <QTextStream>

namespace thoht {
namespace qstr {

QTextStream& noPaddingNewline(QTextStream& s, int fieldWidth)
{
    s << qSetFieldWidth(0) << "\n" << qSetFieldWidth(fieldWidth);
    return s;
}

} // namespace qstr
} // namespace thoht
