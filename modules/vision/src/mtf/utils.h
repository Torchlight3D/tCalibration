#pragma once

#include <cmath>

namespace tl {

inline double angular_diff(double a, double b)
{
    return std::acos(std::cos(a) * std::cos(b) + std::sin(a) * std::sin(b));
}

inline double angle_reduce(double x)
{
    double quad1 = std::abs(std::fmod(x, M_PI / 2.0));
    if (quad1 > M_PI / 4.0) {
        quad1 = M_PI / 2.0 - quad1;
    }
    quad1 = quad1 / M_PI * 180;
    return quad1;
}

} // namespace tl
