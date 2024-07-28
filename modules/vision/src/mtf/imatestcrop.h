#pragma once

#include "cropperbase.h"

class Imatest_cropper : public Cropper
{
public:
    Imatest_cropper(cv::Mat& in_img);

    void fill_bars(const cv::Mat& X);

protected:
    cv::Point black_bar_bounds(const std::vector<double>& data);
    uint16_t max_brightness = 32768;
};
