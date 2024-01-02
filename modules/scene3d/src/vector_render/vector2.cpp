#include "vector2.h"
#include "vector3.h"

#include <algorithm>
#include <cmath>

#ifdef WIN32
#include <windows.h>
#endif

using namespace vrender;

const Vector2 Vector2::inf(FLT_MAX, FLT_MAX);

Vector2::Vector2(const Vector2& u) { setXY(u[0], u[1]); }

Vector2::Vector2(const Vector3& u)
{
    _xyz[0] = u[0];
    _xyz[1] = u[1];
}

Vector2::Vector2(double x, double y) { setXY(x, y); }

Vector2::~Vector2() = default;

double Vector2::norm() const
{
    return std::sqrt(_xyz[0] * _xyz[0] + _xyz[1] * _xyz[1]);
}

double Vector2::squareNorm() const
{
    return _xyz[0] * _xyz[0] + _xyz[1] * _xyz[1];
}

double Vector2::infNorm() const
{
    return std::max(fabs(_xyz[0]), fabs(_xyz[1]));
}

Vector2 Vector2::mini(const Vector2& v1, const Vector2& v2)
{
    return {std::min(v1[0], v2[0]), std::min(v1[1], v2[1])};
}

Vector2 Vector2::maxi(const Vector2& v1, const Vector2& v2)
{
    return {std::max(v1[0], v2[0]), std::max(v1[1], v2[1])};
}

Vector2 vrender::operator*(double r, const Vector2& u)
{
    return {r * u[0], r * u[1]};
}

Vector2 vrender::operator-(const Vector2& u) { return {-u[0], -u[1]}; }

std::ostream& vrender::operator<<(std::ostream& os, const Vector2& vec2)
{
    os << vec2[0] << ", " << vec2[1];
    return os;
}
