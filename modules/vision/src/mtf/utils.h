#pragma once

#include <cmath>

namespace tl {

inline double angular_diff(double a, double b)
{
    return std::acos(std::cos(a) * std::cos(b) + std::sin(a) * std::sin(b));
}

} // namespace tl
