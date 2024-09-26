#pragma once

#include <opencv2/core/types.hpp>

#include "XYWeight.h"

namespace AprilTags {

//! A 2D line
class GLine2D
{
public:
    //! Create a new line.
    GLine2D();

    //! Create a new line.
    /*  @param slope the slope
     *  @param b the y intercept
     */
    GLine2D(float slope, float b);

    //! Create a new line.
    /*  @param dx A change in X corresponding to dy
     *  @param dy A change in Y corresponding to dx
     *  @param p A point that the line passes through
     */
    GLine2D(float dX, float dY, const cv::Point2f& pt);

    //! Create a new line through two points.
    /*  @param p1 the first point
     *  @param p2 the second point
     */
    GLine2D(const cv::Point2f& p1, const cv::Point2f& p2);

    //! Get the coordinate of a point (on this line), with zero corresponding to
    //! the point on the that is perpendicular toa line passing through the
    //! origin and the line.
    /*  This allows easy computation if one point is between two other points on
     * the line: compute the line coordinate of all three points and test if
     * a<=b<=c. This is implemented by computing the dot product of the vector
     * 'p' with the line's direct unit vector.
     */
    float getLineCoordinate(const cv::Point2f& p);

    //! The inverse of getLineCoordinate.
    cv::Point2f getPointOfCoordinate(float coord);

    //! Compute the point where two lines intersect, or (-1,0) if the lines are
    //! parallel.
    cv::Point2f intersectionWith(const GLine2D& line) const;

    static GLine2D lsqFitXYW(const std::vector<XYWeight>& xyweights);

    inline float getDx() const { return delta.x; }
    inline float getDy() const { return delta.y; }
    inline float getFirst() const { return p.x; }
    inline float getSecond() const { return p.y; }

protected:
    void normalizeSlope();
    void normalizeP();

private:
    cv::Point2f delta;
    //!< A point the line passes through; when normalized, it is the
    //!< point closest to the origin (hence perpendicular to the line)
    cv::Point2f p;
    bool didNormalizeSlope;
    bool didNormalizeP;
};

} // namespace AprilTags
