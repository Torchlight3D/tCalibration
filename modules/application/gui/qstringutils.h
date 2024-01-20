#pragma once

#include <QString>
#include <tMath/MathBase>

using namespace Qt::Literals::StringLiterals;

class QTextStream;

namespace tl {

namespace qstr {

inline QString percentage(double val, int precision = 6)
{
    return u"%1%"_s.arg(QString::number(math::toPercent(val), 'f', precision));
}

inline QString permille(double val, int precision = 6)
{
    return u"%1%2"_s.arg(QString::number(math::toPermille(val), 'f', precision),
                         QChar{0x2030});
}

inline QString lessThan(double max, char format = 'g', int precision = 6)
{
    return u"< %1"_s.arg(QString::number(max, format, precision));
};

inline QString lessThanPercentage(double max, int precision = 6)
{
    return u"< %1%"_s.arg(
        QString::number(math::toPercent(max), 'f', precision));
}

inline QString lessThanPermille(double max, int precision = 6)
{
    return u"< %1%2"_s.arg(
        QString::number(math::toPermille(max), 'f', precision), QChar{0x2030});
}

inline QString lessEqualThan(double max, char format = 'g', int precision = 6)
{
    return u"%1 %2"_s.arg(QChar{0x2264}, QString::number(max, format, 6));
};

inline QString greaterThan(double min, char format = 'g', int precision = 6)
{
    return u"> %1"_s.arg(QString::number(min, format, precision));
}

inline QString greaterEqualThan(double min, char format = 'g',
                                int precision = 6)
{
    return u"%1 %2"_s.arg(QChar{0x2265},
                          QString::number(min, format, precision));
}

inline QString inRange(double val, double tol, char format = 'g',
                       int precision = 6)
{
    return u"%1 %2 %3"_s.arg(QString::number(val, format, precision),
                             QChar{0x00B1},
                             QString::number(tol, format, precision));
};

// Workaround for Qt's weird formatting
QTextStream& noPaddingNewline(QTextStream& s, int fieldWidth);

} // namespace qstr

} // namespace tl
