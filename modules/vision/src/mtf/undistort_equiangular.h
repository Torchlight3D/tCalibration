#pragma once

#include "undistort.h"

class Undistort_equiangular : public Undistort
{
public:
    Undistort_equiangular(const cv::Rect& r, double f, double pitch);

    cv::Point2d slow_transform_point(double col, double row) override;
    cv::Point2d inverse_transform_point(double col, double row) override;

    cv::Mat unmap(const cv::Mat& in_src, cv::Mat& rawimg) override;

public:
    double f;
    double pitch;
};
