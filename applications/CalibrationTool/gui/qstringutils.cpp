#include "qstringutils.h"

#include <QTextStream>

namespace tl {
namespace qstr {

QTextStream& noPaddingNewline(QTextStream& s, int fieldWidth)
{
    s << qSetFieldWidth(0) << "\n" << qSetFieldWidth(fieldWidth);
    return s;
}

} // namespace qstr
} // namespace tl
