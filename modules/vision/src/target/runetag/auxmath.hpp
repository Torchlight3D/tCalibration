#pragma once

// #include "precomp.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace cv {
namespace runetag {

template <typename T>
inline bool almost_zero(T a, double e)
{
    return (a == T(0)) || (a > 0 && a < e) || (a < 0 && a > -e);
}

// VideoDemo utility
extern double TDelta(const cv::Mat& T1, const cv::Mat& T2);

// VideoDemo utility
extern double RDelta(const cv::Mat& R1, const cv::Mat& R2);

// VideoDemo - AuxRenderer utility
cv::RotatedRect ellipseToRotatedRect(const cv::Matx33d& ellipse);

// EllipseFitter - SlotFitter utility
extern cv::Matx33d transformToEllipse(const cv::Matx33d& Qc,
                                      const cv::Matx33d& VR, double k);

// EllipseFitter - EllipsePoint - EllipseRefine - MarkerPose - SlotFitter
// utility
extern cv::Point2d ellipseCenter(const cv::Matx33d& e);

// SlotFitter utility (nella parte commentata)
extern double ellipseRadius(const cv::Matx33d& Qc);

// SlotFitter utility
extern cv::Point2d transformPointFromCircleToEllipse(cv::Point2d p,
                                                     const cv::Matx33d& VR,
                                                     double f);

extern cv::Matx33d& scalarDivision(cv::Matx33d& targetMatrix, double scalar);

} // namespace runetag
} // namespace cv
