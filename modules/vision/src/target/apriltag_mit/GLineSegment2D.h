#pragma once

#include "XYWeight.h"

namespace AprilTags {

//! A 2D line with endpoints.
struct GLineSegment2D
{
    cv::Point2f p0, p1;

    static GLineSegment2D lsqFitXYW(const std::vector<XYWeight> &xyweight);
};

} // namespace AprilTags
