#pragma once

#include "common_types.h"
#include "gradient.h"

cv::Point2d centroid(const Pointlist& p);
cv::Point2d average_dir(const Gradient& g, int x, int y);
cv::Point2d normalize(const cv::Point2d& p);
