#pragma once

#include <QString>

#include <AxMath/MathBase>

QT_BEGIN_NAMESPACE
class QTextStream;
QT_END_NAMESPACE

namespace tl {

enum class FloatFormat : char
{
    CapScientific = 'E',
    CapDecimal = 'F', // Capitalize inf & nan
    CapAuto = 'G',    // Capitalize e
    Scientific = 'e',
    Decimal = 'f',
    Auto = 'g', // Use whichever is more concise
};

namespace qstr {

// TODO: How to reduce redundant code
// 1. <= and <, >= and >
// 2. Value dimension (Use parameter pack)
// 3. Possible constraints (e.g. abs)
// 4. Possible units/suffices/objects (e.g. percentage)
// 5. Combination above

namespace {
inline constexpr QChar cPermille{0x2030};
inline constexpr QChar cLessEqual{0x2264};
inline constexpr QChar cGreaterEqual{0x2265};
inline constexpr QChar cPlusMinus{0x00B1};
} // namespace

inline QString point2(double x, double y)
{
    using namespace Qt::Literals::StringLiterals;
    auto fFloat = [](double v) { return QString::number(v, 'f', 2); };
    return u"(%1, %2)"_s.arg(fFloat(x), fFloat(y));
};

inline QString point3(double x, double y, double z)
{
    using namespace Qt::Literals::StringLiterals;
    auto fFloat = [](double v) { return QString::number(v, 'f', 4); };
    return u"(%1, %2, %3)"_s.arg(fFloat(x), fFloat(y), fFloat(z));
}

inline QString percentage(double val, int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    return u"%1%"_s.arg(QString::number(math::toPercent(val), 'f', precision));
}

inline QString permille(double val, int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    return u"%1%2"_s.arg(QString::number(math::toPermille(val), 'f', precision),
                         cPermille);
}

inline QString lessThan(double max, char format = 'g', int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    return u"< %1"_s.arg(QString::number(max, format, precision));
};

inline QString lessThanAbs(double max, char format = 'g', int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    return u"abs < %1"_s.arg(QString::number(max, format, precision));
}

inline QString lessThan2D(double max1, double max2, char format = 'g',
                          int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    auto fFloat = [&](double v) {
        return QString::number(v, format, precision);
    };
    return u"< (%1, %2)"_s.arg(fFloat(max1), fFloat(max2));
}

inline QString lessEqualThan(double max, char format = 'g', int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    return u"%1 %2"_s.arg(cLessEqual, QString::number(max, format, 6));
};

inline QString lessEqualThan2DAbs(double max1, double max2, char format = 'g',
                                  int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    auto fFloat = [&](double v) {
        return QString::number(v, format, precision);
    };
    return u"abs %1 (%2, %3)"_s.arg(cLessEqual, fFloat(max1), fFloat(max2));
}

inline QString lessThanPercentage(double max, int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    return u"< %1%"_s.arg(
        QString::number(math::toPercent(max), 'f', precision));
}

inline QString lessThanPermille(double max, int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    return u"< %1%2"_s.arg(
        QString::number(math::toPermille(max), 'f', precision), cPermille);
}

inline QString greaterThan(double min, char format = 'g', int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    return u"> %1"_s.arg(QString::number(min, format, precision));
}

inline QString greaterEqualThan(double min, char format = 'g',
                                int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    return u"%1 %2"_s.arg(cGreaterEqual,
                          QString::number(min, format, precision));
}

inline QString inRange(double val, double tol, char format = 'g',
                       int precision = 6)
{
    using namespace Qt::Literals::StringLiterals;
    auto fFloat = [&](double v) {
        return QString::number(v, format, precision);
    };
    return u"%1 %2 %3"_s.arg(fFloat(val), cPlusMinus, fFloat(tol));
};

inline QString inRange2D(double x, double y, double x_tol, double y_tol)
{
    using namespace Qt::Literals::StringLiterals;
    auto fFloat = [](double v) { return QString::number(v, 'f', 2); };
    return u"(%1, %2) %3 (%4, %5)"_s.arg(fFloat(x), fFloat(y), cPlusMinus,
                                         QString::number(x_tol),
                                         QString::number(y_tol));
};

inline QString inRange3D(double x, double y, double z, double x_tol,
                         double y_tol, double z_tol)
{
    using namespace Qt::Literals::StringLiterals;
    auto fFloat = [](double v) { return QString::number(v, 'f', 4); };
    return u"(%1, %2, %3) %4 (%5, %6, %7)"_s.arg(
        fFloat(x), fFloat(y), fFloat(z), cPlusMinus, QString::number(x_tol),
        QString::number(y_tol), QString::number(z_tol));
}

// Workaround for Qt's weird formatting
QTextStream& noPaddingNewline(QTextStream& s, int fieldWidth);

} // namespace qstr
} // namespace tl
