#pragma once

#include "apriltag_types.h"
#include "line_2d.h"

namespace apriltags {

class GLineSegment2D
{
public:
    GLineSegment2D(const cv::Point2f &point0, const cv::Point2f &point1);

    cv::Point2f point0() const { return _p0; }
    cv::Point2f point1() const { return _p1; }

    static GLineSegment2D fitLine(
        const std::vector<WeightedPointF> &weightedPoints);

private:
    GLine2D _line;
    cv::Point2f _p0, _p1;
    int _weight;
};

} // namespace apriltags
