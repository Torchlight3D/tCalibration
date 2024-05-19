#include "line_segment_2d.h"

#include <algorithm>

#include <tCore/Math>

using namespace tl;

namespace apriltags {

GLineSegment2D::GLineSegment2D(const cv::Point2f& p0, const cv::Point2f& p1)
    : _line(p0, p1), _p0(p0), _p1(p1), _weight(0)
{
}

GLineSegment2D GLineSegment2D::fitLine(
    const std::vector<WeightedPointF>& weightedPoints)
{
    const auto line = GLine2D::fitLine(weightedPoints);

    auto maxCoord{tl::kMinFloat};
    auto minCoord{tl::kMaxFloat};
    for (const auto& weightedPoint : weightedPoints) {
        float coord = line.lineCoordinate(weightedPoint.point);
        maxCoord = std::max(maxCoord, coord);
        minCoord = std::min(minCoord, coord);
    }

    const auto minValue = line.pointOfCoordinate(minCoord);
    const auto maxValue = line.pointOfCoordinate(maxCoord);

    return {minValue, maxValue};
}

} // namespace apriltags
