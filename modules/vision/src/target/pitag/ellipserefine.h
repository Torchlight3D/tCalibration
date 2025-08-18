#pragma once

#include <opencv2/core/mat.hpp>

// RUNETag fiducial markers library
namespace cv {
namespace runetag {
namespace ipa_Fiducials {

bool ellipserefine(const cv::RotatedRect& ellipse,
                          const cv::Mat& gradient_x, const cv::Mat& gradient_y,
                          cv::Matx33d& out);
cv::Point2d ellipseCenter(const cv::Matx33d& e);

} // namespace ipa_Fiducials
} // namespace runetag
} // namespace cv
