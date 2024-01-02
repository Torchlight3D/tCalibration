#include "vec.h"
#include "dom_utils.h"

using namespace viewer;

void Vec::projectOnAxis(const Vec &direction)
{
#ifndef QT_NO_DEBUG
    if (direction.squaredNorm() < 1.0e-10)
        qWarning(
            "Vec::projectOnAxis: axis direction is not normalized (norm=%f).",
            direction.norm());
#endif

    *this = (((*this) * direction) / direction.squaredNorm()) * direction;
}

void Vec::projectOnPlane(const Vec &normal)
{
#ifndef QT_NO_DEBUG
    if (normal.squaredNorm() < 1.0e-10)
        qWarning(
            "Vec::projectOnPlane: plane normal is not normalized (norm=%f).",
            normal.norm());
#endif

    *this -= (((*this) * normal) / normal.squaredNorm()) * normal;
}

namespace key {
constexpr char kX[]{"x"};
constexpr char kY[]{"y"};
constexpr char kZ[]{"z"};
} // namespace key

Vec Vec::orthogonalVec() const
{
    // Find smallest component. Keep equal case for null values.
    if ((fabs(y) >= 0.9 * fabs(x)) && (fabs(z) >= 0.9 * fabs(x)))
        return {0.0, -z, y};
    if ((fabs(x) >= 0.9 * fabs(y)) && (fabs(z) >= 0.9 * fabs(y)))
        return {-z, 0.0, x};

    return {-y, x, 0.0};
}

Vec::Vec(const QDomElement &element)
{
    const QStringList attributes{key::kX, key::kY, key::kZ};
    for (qsizetype i{0}; i < attributes.size(); ++i) {
        v_[i] = io::qrealFromDom(element, attributes[i], 0.0);
    }
}

QDomElement Vec::domElement(const QString &name, QDomDocument &document) const
{
    auto elem = document.createElement(name);
    elem.setAttribute(key::kX, QString::number(x));
    elem.setAttribute(key::kY, QString::number(y));
    elem.setAttribute(key::kZ, QString::number(z));
    return elem;
}

void Vec::initFromDOMElement(const QDomElement &element)
{
    const Vec v(element);
    *this = v;
}

std::ostream &operator<<(std::ostream &o, const Vec &v)
{
    return o << v.x << '\t' << v.y << '\t' << v.z;
}
