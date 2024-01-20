#pragma once

#include <algorithm>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#define undef_USE_MATH_DEFINES
#endif

#include <cmath>

#ifdef undef_USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#undef undef_USE_MATH_DEFINES
#endif

#include <limits>
#include <numeric>
#include <stdexcept>

#include "tl_math_global.h"

namespace tl {

namespace math {

// TODO: Use <numbers> after Cpp20
template <typename T>
constexpr T pi = T(3.1415926535897932385L);

template <typename T>
constexpr T two_pi = 2 * pi<T>;

template <typename T>
constexpr T two_pi_inv = T(1.0) / two_pi<T>;

template <typename T>
constexpr T four_pi = 4 * pi<T>;

template <typename T>
constexpr T half_pi = T(0.5) * pi<T>;

static constexpr auto kMaxDouble = std::numeric_limits<double>::max();
static constexpr auto kMinDouble = std::numeric_limits<double>::min();
static constexpr auto kInfDouble = std::numeric_limits<double>::infinity();
static constexpr auto kMaxFloat = std::numeric_limits<float>::max();
static constexpr auto kMinFloat = std::numeric_limits<float>::min();
static constexpr auto kInfFloat = std::numeric_limits<float>::infinity();
static constexpr auto kMaxInt = std::numeric_limits<int>::max();
static constexpr auto kMinInt = std::numeric_limits<int>::min();

template <typename T>
constexpr bool isApprox(T a, T b, T ratio = std::numeric_limits<T>::epsilon())
{
    return std::abs(a - b) <= (ratio * std::max(std::abs(a), std::abs(b)));
}

template <typename T>
constexpr bool isApprox0(T v, T ratio = std::numeric_limits<T>::epsilon())
{
    return isApprox(v, T(0), ratio);
}

template <typename T>
constexpr bool inSymmetricRange(T val, T expect, T tolerance)
{
    return std::abs(val - expect) < tolerance;
}

template <typename T>
constexpr bool inRange(T val, T min, T max)
{
    return (val - min) * (val - max) < 0;
}

// FIXME: Maybe overflow for int??
template <class T>
constexpr T square(const T& x)
{
    return x * x;
}

inline constexpr double toPercent(double val) { return val * 1e2; }
inline constexpr double toPermille(double val) { return val * 1e3; }
inline constexpr double fromPercent(double percent) { return percent * 1e-2; }
inline constexpr double fromPermille(double permille)
{
    return permille * 1e-3;
}

template <typename T>
T combination(T n, T k)
{
    static_assert(std::is_integral_v<T>,
                  "The input argument must be integer type.");
    assert(k <= n);

    T r = 1;
    for (T d = 1; d <= k; ++d, --n) {
        T g = std::gcd(r, d);
        r /= g;
        T t = n / (d / g);
        if (r > std::numeric_limits<T>::max() / t) {
            throw std::overflow_error(
                "Overflow when calculate combination k of n.");
        }
        r *= t;
    }

    return r;
}

// Returns a result in [-Pi, Pi]
template <typename T>
constexpr T mod2pi(T val)
{
    const T abs_val = std::abs(val);
    const int qi = static_cast<int>(abs_val * two_pi_inv<T> + T(0.5));
    const T r = abs_val - qi * two_pi<T>;
    return (val < T(0)) ? -r : r;
}

// Returns a value of v such that ref and v differ by no more than +/-Pi
template <typename T>
constexpr T mod2pi(T ref, T val)
{
    return ref + mod2pi(val - ref);
}

template <typename T>
constexpr T degToRad(T deg)
{
    return deg * pi<T> / T(180);
}

template <typename T>
constexpr T radToDeg(T rad)
{
    return rad * T(180) / pi<T>;
}

constexpr double m2cm(double m) { return m * 1e2; }
constexpr double cm2m(double cm) { return cm * 1e-2; }

// Lousy approximation of arctan function,
// but good enough for common purposes (4 degrees)
double fast_atan2(double y, double x);

// Solve ax^2 + bx + c = 0
bool solveQuadraticEquation(double a, double b, double c, double& x1,
                            double& x2);

} // namespace math

} // namespace tl
