#pragma once

#include <vector>

namespace orp {
namespace calibration {

template <typename PointType>
class Quadangle
{
public:
    typedef PointType point_type;

    Quadangle(const std::vector<PointType> &p)
    {
        p0 = p[0];
        p3 = p[3];
        p01 = p[1] - p[0];
        p32 = p[2] - p[3];
    }

    //! Interpolate given that the lower left corner of the lower left cell is
    //! at
    //! (-1,-1) and the upper right corner of the upper right cell is at (1,1).
    PointType interpolate(float x, float y) const
    {
        PointType r1 = p0 + p01 * ((x + 1.0f) / 2.0f);
        PointType r2 = p3 + p32 * ((x + 1.0f) / 2.0f);
        return r1 + (r2 - r1) * ((y + 1.0f) / 2.0f);
    }

    //! Same as interpolate, except that the coordinates are interpreted between
    //! 0 and 1, instead of -1 and 1.
    PointType interpolate01(float x, float y)
    {
        return interpolate(2 * x - 1, 2 * y - 1);
    }

private:
    PointType p0, p3, p01, p32;
};

} // namespace calibration
} // namespace orp
