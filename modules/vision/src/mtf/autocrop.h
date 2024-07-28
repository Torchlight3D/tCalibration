#pragma once

#include "cropperbase.h"

class AutoCropper : public Cropper
{
public:
    explicit AutoCropper(const cv::Mat& img);

    cv::Mat subset(const cv::Mat& X, cv::Rect* dimensions = nullptr) const;

protected:
    cv::Point calcOtsuBound(const std::vector<double>& data, int border) const;
};
