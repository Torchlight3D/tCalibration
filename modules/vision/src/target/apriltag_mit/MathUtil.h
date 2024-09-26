#pragma once

#include <cmath>
#include <cstdlib>

namespace AprilTags {

//! Miscellaneous math utilities and fast exp functions.
class MathUtil
{
public:
    //! Returns the square of a value.
    static inline float square(float x) { return x * x; }

    //! Returns a result in [-Pi, Pi]
    static inline float mod2pi(float vin)
    {
        const float twopi = 2 * (float)M_PI;
        const float twopi_inv = 1.f / (2.f * (float)M_PI);
        float absv = std::abs(vin);
        float q = absv * twopi_inv + 0.5f;
        int qi = (int)q;
        float r = absv - qi * twopi;
        return (vin < 0) ? -r : r;
    }

    //! Returns a value of v wrapped such that ref and v differ by no more than
    //! +/- Pi
    static inline float mod2pi(float ref, float v)
    {
        return ref + mod2pi(v - ref);
    }

    // lousy approximation of arctan function, but good enough for our purposes
    // (about 4 degrees)
    static inline double fast_atan2(double y, double x)
    {
        double coeff_1 = M_PI / 4;
        double coeff_2 = 3 * coeff_1;
        double abs_y = fabs(y) + 1e-10; // kludge to prevent 0/0 condition

        double angle;

        if (x >= 0) {
            double r = (x - abs_y) / (x + abs_y);
            angle = coeff_1 - coeff_1 * r;
        }
        else {
            double r = (x + abs_y) / (abs_y - x);
            angle = coeff_2 - coeff_1 * r;
        }

        if (y < 0)
            return -angle; // negate if in quad III or IV
        else
            return angle;
    }
};

} // namespace AprilTags
