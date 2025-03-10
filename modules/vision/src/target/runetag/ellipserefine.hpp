#pragma once

#include <opencv2/core/mat.hpp>

namespace tl {
namespace runetag {

bool ellipserefine(const cv::RotatedRect& ellipse, const cv::Mat& gradient_x,
                   const cv::Mat& gradient_y, double cx, double cy,
                   cv::Matx33d& out, cv::Mat dbg);

} // namespace runetag
} // namespace tl
