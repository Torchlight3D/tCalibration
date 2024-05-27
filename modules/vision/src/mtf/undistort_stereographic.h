#pragma once

#include "undistort.h"

class Undistort_stereographic : public Undistort
{
public:
    Undistort_stereographic(const cv::Rect& r, double f, double pitch);

    cv::Point2d slow_transform_point(const cv::Point2d& point) const override;
    cv::Point2d inverse_transform_point(
        const cv::Point2d& point) const override;

    cv::Mat unmap(const cv::Mat& in_src, cv::Mat& rawimg) override;

public:
    double f;
    double pitch;
};
