#pragma once

#include <opencv2/core/types.hpp>

namespace AprilTags {

struct XYWeight
{
    cv::Point2f pos;
    float weight;
};

} // namespace AprilTags
