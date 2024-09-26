#include "GLineSegment2D.h"

#include <limits>

#include "GLine2D.h"

namespace AprilTags {

GLineSegment2D GLineSegment2D::lsqFitXYW(const std::vector<XYWeight>& xyweight)
{
    GLine2D gline = GLine2D::lsqFitXYW(xyweight);
    float maxcoord = -std::numeric_limits<float>::infinity();
    float mincoord = std::numeric_limits<float>::infinity();
    for (const auto& wpoint : xyweight) {
        float coord = gline.getLineCoordinate(wpoint.pos);
        maxcoord = std::max(maxcoord, coord);
        mincoord = std::min(mincoord, coord);
    }

    const auto minValue = gline.getPointOfCoordinate(mincoord);
    const auto maxValue = gline.getPointOfCoordinate(maxcoord);
    return {.p0 = minValue, .p1 = maxValue};
}

} // namespace AprilTags
