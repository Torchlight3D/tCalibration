#pragma once

#include "undistort.h"

class Undistort_rectilinear : public Undistort
{
public:
    Undistort_rectilinear(const cv::Rect& r,
                          const std::vector<double>& in_coeffs);

    cv::Point2d slow_transform_point(double col, double row) override;
    cv::Point2d inverse_transform_point(double col, double row) override;
    cv::Mat unmap(const cv::Mat& in_src, cv::Mat& rawimg) override;

public:
    std::vector<double> coeffs_;
    double radius_norm_;
};
