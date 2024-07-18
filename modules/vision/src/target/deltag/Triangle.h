#pragma once

#include <vector>

namespace orp {
namespace calibration {

template <typename PointType>
class Triangle
{
public:
    typedef PointType point_type;
    //! Constructor
    Triangle(const std::vector<PointType> &p)
    {
        p0 = p[0];
        p01 = p[1] - p[0];
        p02 = p[2] - p[0];
    }

    //! Same as interpolate, except that the coordinates are interpreted between
    //! 0 and 1, instead of -1 and 1.
    PointType interpolate01(float x, float y) const
    {
        return p0 + p01 * x + p02 * y;
    }

private:
    PointType p0, p01, p02;
};

} // namespace calibration
} // namespace orp
