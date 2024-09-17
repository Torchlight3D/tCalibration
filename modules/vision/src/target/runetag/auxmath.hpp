#pragma once

#include <opencv2/core/mat.hpp>

namespace tl {
namespace runetag {

template <typename T>
inline bool almost_zero(T a, double e)
{
    return (a == T(0)) || (a > 0 && a < e) || (a < 0 && a > -e);
}

double TDelta(const cv::Mat& T1, const cv::Mat& T2);

double RDelta(const cv::Mat& R1, const cv::Mat& R2);

cv::RotatedRect ellipseToRotatedRect(const cv::Matx33d& ellipse);

cv::Matx33d transformToEllipse(const cv::Matx33d& Qc, const cv::Matx33d& VR,
                               double k);

cv::Point2d ellipseCenter(const cv::Matx33d& e);

double ellipseRadius(const cv::Matx33d& Qc);

cv::Point2d transformPointFromCircleToEllipse(cv::Point2d p,
                                              const cv::Matx33d& VR, double f);

cv::Matx33d& scalarDivision(cv::Matx33d& targetMatrix, double scalar);

} // namespace runetag
} // namespace tl
