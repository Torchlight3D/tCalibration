#include "vector3.h"
#include "nvector3.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#ifdef WIN32
#include <windows.h>
#endif

using namespace vrender;

const Vector3 Vector3::inf(FLT_MAX, FLT_MAX, FLT_MAX);

Vector3::Vector3(const NVector3& u) { setXYZ(u[0], u[1], u[2]); }

Vector3::Vector3(double x, double y, double z) { setXYZ(x, y, z); }

Vector3::~Vector3() = default;

Vector3::Vector3(const Vector3& u) { setXYZ(u[0], u[1], u[2]); }

Vector3& Vector3::operator=(const NVector3& u)
{
    _xyz[0] = u[0];
    _xyz[1] = u[1];
    _xyz[2] = u[2];
    return (*this);
}

Vector3& Vector3::operator+=(const NVector3& u)
{
    _xyz[0] += u[0];
    _xyz[1] += u[1];
    _xyz[2] += u[2];
    return (*this);
}

Vector3& Vector3::operator-=(const NVector3& u)
{
    _xyz[0] -= u[0];
    _xyz[1] -= u[1];
    _xyz[2] -= u[2];
    return (*this);
}

double Vector3::norm() const
{
    return sqrt(_xyz[0] * _xyz[0] + _xyz[1] * _xyz[1] + _xyz[2] * _xyz[2]);
}

double Vector3::squareNorm() const
{
    return _xyz[0] * _xyz[0] + _xyz[1] * _xyz[1] + _xyz[2] * _xyz[2];
}

double Vector3::infNorm() const
{
    return std::max(std::max(fabs(_xyz[0]), fabs(_xyz[1])), fabs(_xyz[2]));
}

Vector3 Vector3::mini(const Vector3& v1, const Vector3& v2)
{
    return {std::min(v1[0], v2[0]), std::min(v1[1], v2[1]),
            std::min(v1[2], v2[2])};
}

Vector3 Vector3::maxi(const Vector3& v1, const Vector3& v2)
{
    return {std::max(v1[0], v2[0]), std::max(v1[1], v2[1]),
            std::max(v1[2], v2[2])};
}

std::ostream& vrender::operator<<(std::ostream& out, const Vector3& u)
{
    out << u[0] << " " << u[1] << " " << u[2];
    return (out);
}
