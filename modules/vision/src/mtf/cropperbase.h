#pragma once

#include <opencv2/core/mat.hpp>

class Cropper
{
public:
    explicit Cropper(const cv::Mat& img);

protected:
    double calcOtsuThreshold(const std::vector<double>& data) const;

protected:
    int rstart_, cstart_;
    int width_, height_;

    std::vector<double> int_rs;
    std::vector<double> int_cs;
};
