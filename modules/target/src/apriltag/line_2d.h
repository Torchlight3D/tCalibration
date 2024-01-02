#pragma once

#include <vector>
#include <opencv2/core/types.hpp>

#include "apriltag_types.h"

namespace apriltags {

class GLine2D
{
public:
    GLine2D();
    GLine2D(float slope, float b);
    GLine2D(const cv::Point2f& point1, const cv::Point2f& point2);
    GLine2D(float dx, float dy, const cv::Point2f& point);

    float dx() const { return _dx; }
    float dy() const { return _dy; }
    float getFirst() const { return _pt.x; }
    float getSecond() const { return _pt.y; }

    //! Get the coordinate of a point (on this line), with zero corresponding to
    //! the point on the that is perpendicular toa line passing through the
    //! origin and the line.
    /*  This allows easy computation if one point is between two other points on
     * the line: compute the line coordinate of all three points and test if
     * a<=b<=c. This is implemented by computing the dot product of the vector
     * 'p' with the line's direct unit vector.
     */
    float lineCoordinate(const cv::Point2f& point) const;

    //! The inverse of getLineCoordinate.
    cv::Point2f pointOfCoordinate(float coord) const;

    //! Compute the point where two lines intersect, or (-1,0) if the lines are
    //! parallel.
    cv::Point2f intersectWith(const GLine2D& line) const;

    //! Use least square to fit line
    static GLine2D fitLine(const std::vector<WeightedPointF>& weightedPoints);

private:
    float _dx, _dy;
    cv::Point2f
        _pt; //!< A point the line passes through; when normalized, it is the
             //!< point closest to the origin (hence perpendicular to the line)
};

} // namespace apriltags
